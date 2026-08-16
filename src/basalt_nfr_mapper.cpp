// Copyright 2026 Caio Mauro
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Caio Mauro nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

#include <cereal/archives/json.hpp>

#include <basalt/calibration/calibration.hpp>
#include <basalt/io/marg_data_io.h>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/vi_estimator/nfr_mapper.h>

namespace {

void usage(const char* program) {
  std::cerr << "usage: " << program
            << " MARG_DATA_DIR CALIB_JSON CONFIG_JSON OUTPUT_TUM [ITERATIONS]\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 5 || argc > 6) {
    usage(argv[0]);
    return 2;
  }

  const std::string marg_data_path = argv[1];
  const std::string calib_path = argv[2];
  const std::string config_path = argv[3];
  const std::string output_path = argv[4];
  const int iterations = argc == 6 ? std::max(1, std::atoi(argv[5])) : 15;

  if (!std::filesystem::is_directory(marg_data_path) ||
      !std::filesystem::is_directory(marg_data_path + "/images")) {
    std::cerr << "invalid marginalization-data directory: " << marg_data_path
              << "\n";
    return 2;
  }

  basalt::Calibration<double> calibration;
  {
    std::ifstream stream(calib_path);
    if (!stream) {
      std::cerr << "cannot open calibration: " << calib_path << "\n";
      return 2;
    }
    cereal::JSONInputArchive archive(stream);
    archive(calibration);
  }

  basalt::VioConfig config;
  config.load(config_path);
  auto mapper = std::make_shared<basalt::NfrMapper>(calibration, config);

  tbb::concurrent_bounded_queue<basalt::MargData::Ptr> queue;
  queue.set_capacity(32);
  basalt::MargDataLoader loader;
  loader.out_marg_queue = &queue;
  loader.start(marg_data_path);

  std::map<int64_t, basalt::MargData::Ptr> marginalizations;
  while (true) {
    basalt::MargData::Ptr data;
    queue.pop(data);
    if (!data) {
      break;
    }
    if (data->kfs_to_marg.empty()) {
      continue;
    }
    marginalizations.emplace(*data->kfs_to_marg.begin(), std::move(data));
  }

  if (marginalizations.empty()) {
    std::cerr << "no usable marginalization records found\n";
    return 1;
  }
  for (auto& [timestamp, data] : marginalizations) {
    (void)timestamp;
    mapper->addMargData(data);
  }

  std::cout << "Loaded " << marginalizations.size() << " marginalizations and "
            << mapper->img_data.size() << " keyframe images\n";
  mapper->feature_corners.clear();
  mapper->feature_matches.clear();
  mapper->detect_keypoints();
  mapper->match_stereo();
  mapper->match_all();
  if (mapper->feature_matches.empty()) {
    std::cerr << "mapping produced no candidate matches\n";
    return 1;
  }
  mapper->build_tracks();
  if (mapper->feature_tracks.empty()) {
    std::cerr << "mapping produced no valid feature tracks\n";
    return 1;
  }
  mapper->setup_opt();
  mapper->optimize(iterations);
  mapper->filterOutliers(3.0, 4);
  mapper->optimize(iterations);

  std::ofstream output(output_path);
  if (!output) {
    std::cerr << "cannot create trajectory: " << output_path << "\n";
    return 2;
  }
  output << "# timestamp tx ty tz qx qy qz qw\n";
  for (const auto& [timestamp, state] : mapper->getFramePoses()) {
    const Sophus::SE3d pose = state.getPose();
    const Eigen::Quaterniond quaternion = pose.unit_quaternion();
    output << std::scientific << std::setprecision(18) << timestamp * 1e-9 << " "
           << pose.translation().x() << " " << pose.translation().y() << " "
           << pose.translation().z() << " " << quaternion.x() << " "
           << quaternion.y() << " " << quaternion.z() << " " << quaternion.w()
           << "\n";
  }

  std::cout << "Saved " << mapper->getFramePoses().size()
            << " optimized keyframe poses to " << output_path << "\n";
  return 0;
}
