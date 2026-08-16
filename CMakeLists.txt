cmake_minimum_required(VERSION 3.16)
project(basalt_wrapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Basalt is bootstrapped as RelWithDebInfo. Keep the wrapper's template-heavy
# Eigen/TBB code on the same NDEBUG/optimization ABI unless a caller explicitly
# selects another build type for both projects.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(OpenSSL REQUIRED)

option(BASALT_WRAPPER_BUILD_RVIZ_PLUGIN
  "Build the optional RViz camera-preview display." ON)
option(BASALT_WRAPPER_BUNDLE_BASALT_LIBRARY
  "Install libbasalt.so with the ROS package for a relocatable install." ON)

if(BASALT_WRAPPER_BUILD_RVIZ_PLUGIN)
  find_package(pluginlib REQUIRED)
  find_package(rviz_common REQUIRED)
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
endif()

set(BASALT_BUNDLED_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/basalt")
set(BASALT_COMMIT "0f3b2b52c807f70ff4e2973ce253c73329eea7bc")
set(BASALT_ROOT "$ENV{BASALT_ROOT}" CACHE PATH
    "Path to an existing Basalt checkout containing build/relwithdebinfo")
if(NOT BASALT_ROOT)
  if(EXISTS "${BASALT_BUNDLED_ROOT}/include")
    set(BASALT_ROOT "${BASALT_BUNDLED_ROOT}")
  else()
    message(FATAL_ERROR
      "Basalt was not found. Run scripts/setup_basalt.sh once, set the "
      "BASALT_ROOT environment variable, or pass "
      "-DBASALT_ROOT=/path/to/basalt. The CMake configure step never downloads "
      "or modifies dependencies.")
  endif()
endif()

set(BASALT_BUILD_DIR "${BASALT_ROOT}/build/relwithdebinfo")
set(BASALT_THIRDPARTY_DIR "${BASALT_ROOT}/thirdparty")

# Basalt exposes Eigen- and TBB-heavy types across libbasalt.so. Compiling the
# wrapper with a different -march setting can change alignment/layout and cause
# an ABI failure while constructing optical flow. Read the setting recorded by
# Basalt's CMake configure unless the caller explicitly overrides it.
set(BASALT_DETECTED_MARCH "")
if(EXISTS "${BASALT_BUILD_DIR}/CMakeCache.txt")
  file(STRINGS "${BASALT_BUILD_DIR}/CMakeCache.txt" BASALT_MARCH_CACHE_LINE
    REGEX "^CXX_MARCH(:[^=]+)?=" LIMIT_COUNT 1)
  if(BASALT_MARCH_CACHE_LINE)
    string(REGEX REPLACE "^[^=]+=" "" BASALT_DETECTED_MARCH
      "${BASALT_MARCH_CACHE_LINE}")
  endif()
endif()
if(NOT BASALT_DETECTED_MARCH)
  set(BASALT_DETECTED_MARCH "native")
endif()
set(BASALT_WRAPPER_MARCH "${BASALT_DETECTED_MARCH}" CACHE STRING
  "Architecture passed to -march; must match the Basalt library build")
if(NOT BASALT_WRAPPER_MARCH STREQUAL BASALT_DETECTED_MARCH)
  message(WARNING
    "BASALT_WRAPPER_MARCH='${BASALT_WRAPPER_MARCH}' differs from Basalt's "
    "recorded CXX_MARCH='${BASALT_DETECTED_MARCH}'. This can be ABI-incompatible.")
endif()
message(STATUS "Using Basalt-compatible -march=${BASALT_WRAPPER_MARCH}")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(BASALT_VCPKG_TRIPLET "arm64-linux")
else()
  set(BASALT_VCPKG_TRIPLET "x64-linux")
endif()
set(BASALT_VCPKG_INCLUDE_DIR
    "${BASALT_BUILD_DIR}/vcpkg_installed/${BASALT_VCPKG_TRIPLET}/include")
set(BASALT_VCPKG_EIGEN_DIR "${BASALT_VCPKG_INCLUDE_DIR}/eigen3")
set(BASALT_VCPKG_LIB_DIR "${BASALT_BUILD_DIR}/vcpkg_installed/${BASALT_VCPKG_TRIPLET}/lib")
set(BASALT_LIBRARY "${BASALT_BUILD_DIR}/libbasalt.so" CACHE FILEPATH
    "Path to the Basalt shared library")

set(BASALT_PUBLIC_INCLUDE_DIR "${BASALT_ROOT}/include")
set(BASALT_SOURCE_ROOT_INCLUDE_DIR "${BASALT_ROOT}")

set(BASALT_HEADER_INCLUDE_DIRS "")
foreach(candidate
    "${BASALT_PUBLIC_INCLUDE_DIR}"
    "${BASALT_VCPKG_INCLUDE_DIR}"
    "${BASALT_SOURCE_ROOT_INCLUDE_DIR}"
)
  if(EXISTS "${candidate}")
    list(APPEND BASALT_HEADER_INCLUDE_DIRS "${candidate}")
  endif()
endforeach()

if(NOT EXISTS "${BASALT_PUBLIC_INCLUDE_DIR}/basalt/optical_flow/optical_flow.h")
  message(FATAL_ERROR
    "Could not find basalt/optical_flow/optical_flow.h under ${BASALT_PUBLIC_INCLUDE_DIR}.")
endif()
if(NOT EXISTS "${BASALT_VCPKG_INCLUDE_DIR}/basalt/calibration/calibration.hpp"
  AND NOT EXISTS "${BASALT_PUBLIC_INCLUDE_DIR}/basalt/calibration/calibration.hpp"
  AND NOT EXISTS "${BASALT_SOURCE_ROOT_INCLUDE_DIR}/basalt/calibration/calibration.hpp")
  message(FATAL_ERROR
    "Could not find basalt/calibration/calibration.hpp under ${BASALT_ROOT}. "
    "Checked ${BASALT_PUBLIC_INCLUDE_DIR}, ${BASALT_VCPKG_INCLUDE_DIR}, and ${BASALT_SOURCE_ROOT_INCLUDE_DIR}.")
endif()

message(STATUS "Using Basalt include search roots: ${BASALT_HEADER_INCLUDE_DIRS}")

if(NOT EXISTS "${BASALT_LIBRARY}")
  message(FATAL_ERROR
    "Basalt library not found at ${BASALT_LIBRARY}. Run scripts/setup_basalt.sh, "
    "build the checkout at BASALT_ROOT, or pass -DBASALT_LIBRARY=/path/to/libbasalt.so.")
endif()

add_library(Basalt::basalt SHARED IMPORTED GLOBAL)
set_target_properties(Basalt::basalt PROPERTIES
  IMPORTED_LOCATION "${BASALT_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${BASALT_HEADER_INCLUDE_DIRS}"
)

if(EXISTS "${BASALT_VCPKG_EIGEN_DIR}/Eigen/Core")
  set(EIGEN3_INCLUDE_DIR "${BASALT_VCPKG_EIGEN_DIR}")
elseif(EXISTS "${BASALT_VCPKG_INCLUDE_DIR}/eigen3/Eigen/Core")
  set(EIGEN3_INCLUDE_DIR "${BASALT_VCPKG_INCLUDE_DIR}/eigen3")
else()
  set(EIGEN3_INCLUDE_DIR "")
endif()

if(EIGEN3_INCLUDE_DIR)
  message(STATUS "Using Basalt-provided Eigen headers from ${EIGEN3_INCLUDE_DIR}")
else()
  message(STATUS "Using system Eigen3 headers")
endif()

add_executable(basalt_node
  src/main.cpp
  src/basalt_node_core.cpp
  src/image_pipeline.cpp
  src/imu_pipeline.cpp
  src/publishers.cpp
  src/debug_service.cpp
)

target_include_directories(basalt_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_include_directories(basalt_node SYSTEM PRIVATE
  ${BASALT_HEADER_INCLUDE_DIRS}
  "${BASALT_THIRDPARTY_DIR}"
  "${BASALT_VCPKG_INCLUDE_DIR}"
  "${BASALT_VCPKG_EIGEN_DIR}"
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(basalt_node PRIVATE
    -Wall
    -Wextra
    -Wpedantic
  )
  if(BASALT_WRAPPER_MARCH)
    target_compile_options(basalt_node PRIVATE "-march=${BASALT_WRAPPER_MARCH}")
  endif()
endif()

ament_target_dependencies(basalt_node
  geometry_msgs
  message_filters
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  cv_bridge
)

target_link_libraries(basalt_node
  Eigen3::Eigen
  Basalt::basalt
  ${OpenCV_LIBS}
  OpenSSL::Crypto
)

set_target_properties(basalt_node PROPERTIES
  BUILD_RPATH "${BASALT_BUILD_DIR};${BASALT_VCPKG_LIB_DIR}"
  INSTALL_RPATH "$ORIGIN/.."
)

install(TARGETS basalt_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(basalt_nfr_mapper
  src/basalt_nfr_mapper.cpp
)
target_include_directories(basalt_nfr_mapper SYSTEM PRIVATE
  ${BASALT_HEADER_INCLUDE_DIRS}
  "${BASALT_THIRDPARTY_DIR}"
  "${BASALT_VCPKG_INCLUDE_DIR}"
  "${BASALT_VCPKG_EIGEN_DIR}"
)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(basalt_nfr_mapper PRIVATE -Wall -Wextra -Wpedantic)
  if(BASALT_WRAPPER_MARCH)
    target_compile_options(basalt_nfr_mapper PRIVATE "-march=${BASALT_WRAPPER_MARCH}")
  endif()
endif()
target_link_libraries(basalt_nfr_mapper
  Eigen3::Eigen
  Basalt::basalt
)
set_target_properties(basalt_nfr_mapper PROPERTIES
  BUILD_RPATH "${BASALT_BUILD_DIR};${BASALT_VCPKG_LIB_DIR}"
  INSTALL_RPATH "$ORIGIN/.."
)
install(TARGETS basalt_nfr_mapper
  DESTINATION lib/${PROJECT_NAME}
)

if(BASALT_WRAPPER_BUNDLE_BASALT_LIBRARY)
  install(FILES "${BASALT_LIBRARY}" DESTINATION lib)
endif()

if(BASALT_WRAPPER_BUILD_RVIZ_PLUGIN)
  add_library(camera_preview_display SHARED
    src/camera_preview_display.cpp
    include/basalt_wrapper/camera_preview_display.hpp
  )
  set_target_properties(camera_preview_display PROPERTIES AUTOMOC ON)
  target_include_directories(camera_preview_display PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
  ament_target_dependencies(camera_preview_display
    cv_bridge
    pluginlib
    rclcpp
    rviz_common
    sensor_msgs
  )
  target_link_libraries(camera_preview_display
    Qt5::Widgets
    ${OpenCV_LIBS}
  )
  pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

  install(TARGETS camera_preview_display
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

install(PROGRAMS
  scripts/euroc_to_rosbag2.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME euroc_to_rosbag2
)

install(PROGRAMS
  scripts/euroc_replay_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME euroc_replay_node
)

install(PROGRAMS
  scripts/euroc_groundtruth_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME euroc_groundtruth_publisher
)

install(PROGRAMS
  scripts/compare_basalt_truth_offline.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME compare_basalt_truth_offline
)

install(PROGRAMS
  scripts/gz_ground_truth_odometry.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gz_ground_truth_odometry
)

install(PROGRAMS
  scripts/odom_from_rosbag2_to_tum.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odom_from_rosbag2_to_tum
)

install(PROGRAMS
  scripts/verify_sim_vio_calibration.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME verify_sim_vio_calibration
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  LICENSE
  THIRD_PARTY_NOTICES.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright(cmake include launch scripts src test)
  ament_cppcheck(include src)
  ament_cpplint(
    include
    src
    MAX_LINE_LENGTH 120
    FILTERS
      -build/include_order
      -build/include_what_you_use
      -runtime/int
      -whitespace/line_length
  )
  ament_flake8(
    launch
    scripts
    test
    CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.flake8"
  )
  ament_lint_cmake(CMakeLists.txt cmake)
  ament_pep257(launch scripts test)
  ament_add_pytest_test(public_assets test/test_public_assets.py)
  ament_xmllint(package.xml rviz_plugins.xml)
endif()

ament_package()
