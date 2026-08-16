# Copyright 2026 Caio Mauro
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Caio Mauro nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


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
