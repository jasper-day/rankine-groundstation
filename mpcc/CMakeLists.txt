# SPDX-License-Identifier: MIT-0

cmake_minimum_required(VERSION 3.16)
project(mpcc LANGUAGES CXX)

# N.B. This is a temporary flag. It only really applies to Linux, as Mac
# does not need X11.
option(RUN_X11_TESTS "Run tests that require X11" OFF)

include(CTest)

find_package(drake CONFIG REQUIRED)

if(DEFINED ENV{VIRTUAL_ENV})
  set(Python3_ROOT_DIR $ENV{VIRTUAL_ENV})
  message(STATUS "Using Python from virtual environment: $ENV{VIRTUAL_ENV}")
endif()

find_package(Python3 ${drake_PYTHON_VERSION} EXACT REQUIRED COMPONENTS Interpreter Development)

if(DEFINED ENV{VIRTUAL_ENV})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{VIRTUAL_ENV}")
endif()

get_filename_component(DRAKE_PYTHONPATH
  "${drake_DIR}/../../python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
  REALPATH
)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# third party defines google test gtest
add_subdirectory(third_party)
# main code lives here, including library definitions. Edit library definitions in src/cpp/CMakeLists.txt
add_subdirectory(src/cpp)
# add_subdirectory(tests)

# Install pure-python files
file(GLOB_RECURSE PYTHON_SOURCES "src/${SKBUILD_PROJECT_NAME}/*.py")
file(GLOB_RECURSE PYTHON_STUBS "src/${SKBUILD_PROJECT_NAME}/*.pyi")

install(FILES ${PYTHON_SOURCES} ${PYTHON_STUBS} DESTINATION ${SKBUILD_PROJECT_NAME})