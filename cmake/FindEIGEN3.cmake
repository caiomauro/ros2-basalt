# Adapted from Basalt's ROS wrapper so this package can pin Eigen to the same
# include tree used by the local Basalt build.

if(NOT EIGEN3_FIND_VERSION)
  if(NOT EIGEN3_FIND_VERSION_MAJOR)
    set(EIGEN3_FIND_VERSION_MAJOR 2)
  endif()
  if(NOT EIGEN3_FIND_VERSION_MINOR)
    set(EIGEN3_FIND_VERSION_MINOR 91)
  endif()
  if(NOT EIGEN3_FIND_VERSION_PATCH)
    set(EIGEN3_FIND_VERSION_PATCH 0)
  endif()

  set(EIGEN3_FIND_VERSION
      "${EIGEN3_FIND_VERSION_MAJOR}.${EIGEN3_FIND_VERSION_MINOR}.${EIGEN3_FIND_VERSION_PATCH}")
endif()

macro(_eigen3_check_version)
  file(READ "${EIGEN3_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h"
       _eigen3_version_header)

  string(REGEX MATCH "define[ \t]+EIGEN_WORLD_VERSION[ \t]+([0-9]+)"
         _eigen3_world_version_match "${_eigen3_version_header}")
  set(EIGEN3_WORLD_VERSION "${CMAKE_MATCH_1}")

  string(REGEX MATCH "define[ \t]+EIGEN_MAJOR_VERSION[ \t]+([0-9]+)"
         _eigen3_major_version_match "${_eigen3_version_header}")
  set(EIGEN3_MAJOR_VERSION "${CMAKE_MATCH_1}")

  string(REGEX MATCH "define[ \t]+EIGEN_MINOR_VERSION[ \t]+([0-9]+)"
         _eigen3_minor_version_match "${_eigen3_version_header}")
  set(EIGEN3_MINOR_VERSION "${CMAKE_MATCH_1}")

  set(EIGEN3_VERSION
      "${EIGEN3_WORLD_VERSION}.${EIGEN3_MAJOR_VERSION}.${EIGEN3_MINOR_VERSION}")

  if(EIGEN3_VERSION VERSION_LESS EIGEN3_FIND_VERSION)
    set(EIGEN3_VERSION_OK FALSE)
  else()
    set(EIGEN3_VERSION_OK TRUE)
  endif()
endmacro()

if(EIGEN3_INCLUDE_DIR)
  _eigen3_check_version()
  set(EIGEN3_FOUND ${EIGEN3_VERSION_OK})
else()
  foreach(_eigen_hint ${EIGEN_INCLUDE_DIR_HINTS})
    if(EXISTS "${_eigen_hint}/Eigen/src/Core/util/Macros.h")
      set(EIGEN3_INCLUDE_DIR "${_eigen_hint}")
      break()
    endif()

    if(EXISTS "${_eigen_hint}/eigen3/Eigen/src/Core/util/Macros.h")
      set(EIGEN3_INCLUDE_DIR "${_eigen_hint}/eigen3")
      break()
    endif()
  endforeach()

  if(NOT EIGEN3_INCLUDE_DIR)
    find_path(EIGEN3_INCLUDE_DIR
      NAMES signature_of_eigen3_matrix_library Eigen/src/Core/util/Macros.h
      HINTS ${EIGEN_INCLUDE_DIR_HINTS}
      PATH_SUFFIXES eigen3
      NO_DEFAULT_PATH
    )
  endif()

  if(EIGEN3_INCLUDE_DIR)
    _eigen3_check_version()
  endif()

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(
    EIGEN3 DEFAULT_MSG EIGEN3_INCLUDE_DIR EIGEN3_VERSION_OK)

  mark_as_advanced(EIGEN3_INCLUDE_DIR)
endif()
