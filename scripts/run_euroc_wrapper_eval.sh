#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: $0 <dataset_root_or_mav0> [output_prefix]" >&2
  exit 2
fi

DATASET_INPUT="$1"
OUTPUT_PREFIX="${2:-$HOME/V1_02_medium}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$HOME/ros2_ws"
CALIB="$WS_DIR/src/basalt_wrapper/third_party/basalt/data/euroc_eucm_calib.json"
CONFIG="$WS_DIR/src/basalt_wrapper/third_party/basalt/data/euroc_config.json"

resolve_mav0_path() {
  local input_path="$1"
  if [[ "$(basename "$input_path")" == "mav0" && -d "$input_path" ]]; then
    printf '%s\n' "$input_path"
    return 0
  fi
  if [[ -d "$input_path/mav0" ]]; then
    printf '%s\n' "$input_path/mav0"
    return 0
  fi
  local child
  for child in "$input_path"/*; do
    if [[ -d "$child/mav0" ]]; then
      printf '%s\n' "$child/mav0"
      return 0
    fi
  done
  return 1
}

if ! MAV0_PATH="$(resolve_mav0_path "$DATASET_INPUT")"; then
  echo "could not resolve mav0 under: $DATASET_INPUT" >&2
  exit 1
fi

GT_CSV="$MAV0_PATH/state_groundtruth_estimate0/data.csv"
GT_TUM="${OUTPUT_PREFIX}_groundtruth.tum"
BAG_URI="${OUTPUT_PREFIX}_offline_ros2"
EST_TUM="${OUTPUT_PREFIX}_wrapper_estimated.tum"
EST_TUM_CLEAN="${OUTPUT_PREFIX}_wrapper_estimated_clean.tum"
INGEST_CSV="${OUTPUT_PREFIX}_basalt_ingest_log.csv"
PLOT_DIR="${OUTPUT_PREFIX}_evo_plots"
TRAJ_PLOT="$PLOT_DIR/traj_overlay.pdf"
ERR_PLOT="$PLOT_DIR/ape_plot.pdf"
RVIZ_CONFIG="$HOME/basalt_wrapper/rviz/euroc_wrapper_eval.rviz"

if [[ ! -f "$GT_CSV" ]]; then
  echo "ground-truth csv not found: $GT_CSV" >&2
  exit 1
fi

mkdir -p "$PLOT_DIR"

cd "$WS_DIR"
set +u
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"
set -u

python3 "$SCRIPT_DIR/euroc_groundtruth_to_tum.py" "$GT_CSV" "$GT_TUM"

if [[ -d "$BAG_URI" ]] && compgen -G "$BAG_URI/*" > /dev/null; then
  echo "reusing existing offline rosbag2: $BAG_URI"
else
  rm -rf "$BAG_URI"
  python3 "$SCRIPT_DIR/euroc_to_rosbag2.py" \
    --ros-args \
    -p dataset_path:="$DATASET_INPUT" \
    -p output_uri:="$BAG_URI" \
    -p storage_id:=sqlite3
fi

rm -f "$EST_TUM" "$EST_TUM_CLEAN" "$INGEST_CSV"
python3 "$SCRIPT_DIR/euroc_groundtruth_publisher.py" "$GT_TUM" "basalt_world" &
GT_PUBLISHER_PID=$!
cleanup() {
  if [[ -n "${GT_PUBLISHER_PID:-}" ]] && kill -0 "$GT_PUBLISHER_PID" 2>/dev/null; then
    kill "$GT_PUBLISHER_PID" 2>/dev/null || true
    wait "$GT_PUBLISHER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

ros2 launch basalt_wrapper basalt_node.launch.py \
  input_mode:=rosbag2 \
  bag_uri:="$BAG_URI" \
  bag_storage_id:=sqlite3 \
  left_image_topic:=/cam0/image_raw \
  right_image_topic:=/cam1/image_raw \
  imu_topic:=/imu0 \
  calib_path:="$CALIB" \
  config_path:="$CONFIG" \
  trajectory_output_path:="$EST_TUM" \
  ingest_log_path:="$INGEST_CSV" \
  publish_debug_visuals:=true \
  use_rviz:=false \
  use_sim_time:=false

awk 'NF == 8 {print $0}' "$EST_TUM" > "$EST_TUM_CLEAN"

echo
python3 "$SCRIPT_DIR/compare_tum_trajectories.py" \
  "$GT_TUM" \
  "$EST_TUM_CLEAN" \
  "$TRAJ_PLOT" \
  "$ERR_PLOT"

echo
echo "Outputs:"
echo "  ground truth tum: $GT_TUM"
echo "  wrapper estimate: $EST_TUM_CLEAN"
echo "  error plot:       $ERR_PLOT"
echo "  traj overlay:     $TRAJ_PLOT"
echo
echo "RViz:"
echo "  source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash && rviz2 -d $RVIZ_CONFIG"
