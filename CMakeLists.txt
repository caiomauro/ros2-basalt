cmake_minimum_required(VERSION 3.16)
project(basalt_wrapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

set(BASALT_BUNDLED_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/basalt")
set(BASALT_REPO_URL "https://github.com/VladyslavUsenko/basalt.git")
set(BASALT_COMMIT "0f3b2b52c807f70ff4e2973ce253c73329eea7bc")
option(BASALT_VENDOR_BUNDLED
  "Automatically clone and build the pinned Basalt dependency into third_party/basalt."
  ON)
set(BASALT_ROOT "$ENV{BASALT_ROOT}" CACHE PATH
    "Path to an existing Basalt checkout containing build/relwithdebinfo")
if(NOT BASALT_ROOT)
  if(BASALT_VENDOR_BUNDLED)
    set(BASALT_ROOT "${BASALT_BUNDLED_ROOT}")
  else()
    message(FATAL_ERROR
      "BASALT_ROOT was not provided and BASALT_VENDOR_BUNDLED is OFF. Either "
      "set -DBASALT_ROOT=/path/to/basalt or enable BASALT_VENDOR_BUNDLED.")
  endif()
endif()

set(BASALT_BUILD_DIR "${BASALT_ROOT}/build/relwithdebinfo")
set(BASALT_THIRDPARTY_DIR "${BASALT_ROOT}/thirdparty")
set(BASALT_VCPKG_INCLUDE_DIR
    "${BASALT_BUILD_DIR}/vcpkg_installed/x64-linux/include")
set(BASALT_VCPKG_EIGEN_DIR "${BASALT_VCPKG_INCLUDE_DIR}/eigen3")
set(BASALT_VCPKG_LIB_DIR "${BASALT_BUILD_DIR}/vcpkg_installed/x64-linux/lib")
set(BASALT_EXPECTS_VENDOR_BUILD OFF)
if(BASALT_VENDOR_BUNDLED AND "${BASALT_ROOT}" STREQUAL "${BASALT_BUNDLED_ROOT}")
  set(BASALT_EXPECTS_VENDOR_BUILD ON)
endif()

if(BASALT_EXPECTS_VENDOR_BUILD AND NOT EXISTS "${BASALT_BUNDLED_ROOT}/include")
  find_program(GIT_EXECUTABLE git REQUIRED)
  message(STATUS "Fetching Basalt source into ${BASALT_BUNDLED_ROOT}")
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" clone --recursive "${BASALT_REPO_URL}" "${BASALT_BUNDLED_ROOT}"
    COMMAND_ERROR_IS_FATAL ANY
  )
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" -C "${BASALT_BUNDLED_ROOT}" checkout "${BASALT_COMMIT}"
    COMMAND_ERROR_IS_FATAL ANY
  )
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" -C "${BASALT_BUNDLED_ROOT}" submodule update --init --recursive
    COMMAND_ERROR_IS_FATAL ANY
  )
endif()

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

if(NOT BASALT_EXPECTS_VENDOR_BUILD)
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
endif()

message(STATUS "Using Basalt include search roots: ${BASALT_HEADER_INCLUDE_DIRS}")

if(BASALT_EXPECTS_VENDOR_BUILD)
  find_program(BASH_EXECUTABLE bash REQUIRED)
  if(NOT EXISTS "${BASALT_BUILD_DIR}/libbasalt.so"
     OR NOT EXISTS "${BASALT_VCPKG_INCLUDE_DIR}/basalt/calibration/calibration.hpp")
    message(STATUS
      "Bootstrapping vendored Basalt into ${BASALT_BUNDLED_ROOT} during configure.")
    execute_process(
      COMMAND "${BASH_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/setup_basalt.sh"
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
      COMMAND_ERROR_IS_FATAL ANY
    )
  endif()
elseif(NOT EXISTS "${BASALT_BUILD_DIR}/libbasalt.so")
  message(FATAL_ERROR
    "Basalt library not found at ${BASALT_BUILD_DIR}/libbasalt.so. Build Basalt first in "
    "${BASALT_ROOT}/build/relwithdebinfo or pass -DBASALT_ROOT=/path/to/basalt.")
endif()

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
    -march=native
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

ament_target_dependencies(basalt_node
  geometry_msgs
  message_filters
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  cv_bridge
)

target_link_directories(basalt_node PRIVATE
  "${BASALT_BUILD_DIR}"
  "${BASALT_VCPKG_LIB_DIR}"
)

target_link_libraries(basalt_node
  Eigen3::Eigen
  basalt
  ${OpenCV_LIBS}
)

set_target_properties(basalt_node PROPERTIES
  BUILD_RPATH "${BASALT_BUILD_DIR};${BASALT_VCPKG_LIB_DIR}"
  INSTALL_RPATH "${BASALT_BUILD_DIR};${BASALT_VCPKG_LIB_DIR}"
)

install(TARGETS basalt_node
  DESTINATION lib/${PROJECT_NAME}
)

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

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
