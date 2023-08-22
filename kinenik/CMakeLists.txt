#     Copyright 2022 Institute of Industrial and Control Engineering (IOC)
#                  Universitat Politecnica de Catalunya
#                  BarcelonaTech
#     All Rights Reserved.
#
#     This program is free software; you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation; either version 2 of the License, or
#     (at your option) any later version.
#
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
#
#     You should have received a copy of the GNU General Public License
#     along with this program; if not, write to the
#     Free Software Foundation, Inc.,
#     59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
#
#    Author:   Leopold Palomo-Avellaneda
#


cmake_minimum_required(VERSION 3.1.3)
project(kinenik VERSION 0.0.1 LANGUAGES CXX)

set(PROJECT_NAME kinenik)
set(PROJECT_BUGREPORT leopold.palomo@upc.edu)
set(
  PROJECT_STRING
  "${PROJECT_NAME} ${PROJECT_VERSION}
  Library to solve inverse and direct kinematic problems of robots"
)
set(PROJECT_TARNAME ${PROJECT_NAME})
set(PROJECT_NAME_VERSION
  ${PROJECT_NAME_MAJOR_VERSION}.${PROJECT_NAME_MINOR_VERSION}.${PROJECT_NAME_PATCH_VERSION})

# some needed stuff
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# some definitions
add_definitions(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# required package
find_package( Eigen3 REQUIRED )

include_directories(include)

if(NOT DEFINED BUILD_SHARED_LIBS)
  option(BUILD_SHARED_LIBS "Build dynamically-linked binaries" ON)
endif()

set(SRC_FILES src/kinenik_ur.cpp)

add_library(${PROJECT_NAME} ${SRC_FILES})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include;${CMAKE_CURRENT_BINARY_DIR}>"
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>
)

add_executable( test_kinenik_ur ${PROJECT_SOURCE_DIR}/src/test_kinenik_ur.cpp)
target_link_libraries(test_kinenik_ur PUBLIC ${PROJECT_NAME})
add_dependencies(test_kinenik_ur ${PROJECT_NAME})

#############
## Install ##
#############
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)



##############################################################################
# File describing how to use library
##############################################################################
# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)

# Create the ${PROJECT_NAME}Config.cmake.
configure_file(
  ${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME}Config.cmake.in
  "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY
)

# Create the ${PROJECT_NAME}ConfigVersion.cmake.
write_basic_package_version_file(
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  COMPATIBILITY AnyNewerVersion
)

install(
  FILES
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    "${CMAKE_CURRENT_SOURCE_DIR}/package.xml"
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}
)

# Export targets.
set(export_targets ${export_targets};${PROJECT_NAME})
export(
  EXPORT export_${PROJECT_NAME}
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake"
  NAMESPACE ${PROJECT_NAME}::
)

install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
)
