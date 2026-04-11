cmake_minimum_required(VERSION 3.16)
project(basalt_wrapper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(ExternalProject)

find_package(ament_cmake REQUIRED)
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
set(BASALT_INCLUDE_DIR "${BASALT_ROOT}/include")
set(BASALT_THIRDPARTY_DIR "${BASALT_ROOT}/thirdparty")
set(BASALT_VCPKG_INCLUDE_DIR
    "${BASALT_BUILD_DIR}/vcpkg_installed/x64-linux/include")
set(BASALT_VCPKG_EIGEN_DIR "${BASALT_VCPKG_INCLUDE_DIR}/eigen3")
set(BASALT_VCPKG_LIB_DIR "${BASALT_BUILD_DIR}/vcpkg_installed/x64-linux/lib")
set(BASALT_LIBRARY "${BASALT_BUILD_DIR}/libbasalt.so")

set(BASALT_EXPECTS_VENDOR_BUILD OFF)
if(BASALT_VENDOR_BUNDLED AND "${BASALT_ROOT}" STREQUAL "${BASALT_BUNDLED_ROOT}")
  set(BASALT_EXPECTS_VENDOR_BUILD ON)
endif()

if(BASALT_EXPECTS_VENDOR_BUILD)
  find_program(BASH_EXECUTABLE bash REQUIRED)
  ExternalProject_Add(basalt_vendor
    SOURCE_DIR "${BASALT_BUNDLED_ROOT}"
    DOWNLOAD_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND "${BASH_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/setup_basalt.sh"
    INSTALL_COMMAND ""
    BUILD_BYPRODUCTS
      "${BASALT_LIBRARY}"
      "${BASALT_VCPKG_LIB_DIR}/libtbb.so"
    USES_TERMINAL_BUILD TRUE
  )
  message(STATUS
    "Basalt will be vendored automatically into ${BASALT_BUNDLED_ROOT} during build.")
elseif(NOT EXISTS "${BASALT_LIBRARY}")
  message(FATAL_ERROR
    "Basalt library not found at ${BASALT_LIBRARY}. Build Basalt first in "
    "${BASALT_ROOT}/build/relwithdebinfo or pass -DBASALT_ROOT=/path/to/basalt.")
endif()

if(BASALT_EXPECTS_VENDOR_BUILD)
  set(EIGEN3_INCLUDE_DIR "${BASALT_VCPKG_EIGEN_DIR}")
elseif(EXISTS "${BASALT_VCPKG_EIGEN_DIR}/Eigen/Core")
  set(EIGEN3_INCLUDE_DIR "${BASALT_VCPKG_EIGEN_DIR}")
elseif(EXISTS "${BASALT_VCPKG_INCLUDE_DIR}/eigen3/Eigen/Core")
  set(EIGEN3_INCLUDE_DIR "${BASALT_VCPKG_INCLUDE_DIR}/eigen3")
else()
  message(FATAL_ERROR
    "Could not locate Eigen headers in Basalt local vcpkg tree. Checked "
    "${BASALT_VCPKG_EIGEN_DIR} and ${BASALT_VCPKG_INCLUDE_DIR}/eigen3")
endif()

message(STATUS "Using Eigen headers from ${EIGEN3_INCLUDE_DIR}")

if(BASALT_EXPECTS_VENDOR_BUILD AND NOT EXISTS "${BASALT_VCPKG_LIB_DIR}/libtbb.so")
  set(TBB_LIBRARY "${BASALT_VCPKG_LIB_DIR}/libtbb.so")
else()
  find_library(TBB_LIBRARY
    NAMES tbb
    HINTS "${BASALT_VCPKG_LIB_DIR}" "/usr/lib/x86_64-linux-gnu"
    NO_DEFAULT_PATH
  )

  if(NOT TBB_LIBRARY)
    find_library(TBB_LIBRARY NAMES tbb REQUIRED)
  endif()
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
  "${BASALT_INCLUDE_DIR}"
  "${BASALT_THIRDPARTY_DIR}"
  "${BASALT_VCPKG_INCLUDE_DIR}"
  "${BASALT_VCPKG_EIGEN_DIR}"
  "${EIGEN3_INCLUDE_DIR}"
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
  "${BASALT_LIBRARY}"
  "${TBB_LIBRARY}"
  ${OpenCV_LIBS}
)

if(TARGET basalt_vendor)
  add_dependencies(basalt_node basalt_vendor)
endif()

set_target_properties(basalt_node PROPERTIES
  BUILD_RPATH "${BASALT_BUILD_DIR};${BASALT_VCPKG_LIB_DIR}"
  INSTALL_RPATH "${BASALT_BUILD_DIR};${BASALT_VCPKG_LIB_DIR}"
)

install(TARGETS basalt_node
  DESTINATION lib/${PROJECT_NAME}
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
