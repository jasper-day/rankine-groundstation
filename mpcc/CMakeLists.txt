# SPDX-License-Identifier: MIT-0

cmake_minimum_required(VERSION 3.16)
project(mpcc LANGUAGES CXX)

# N.B. This is a temporary flag. It only really applies to Linux, as Mac
# does not need X11.
option(RUN_X11_TESTS "Run tests that require X11" OFF)

if(DEFINED ENV{VIRTUAL_ENV})
  set(Python3_ROOT_DIR $ENV{VIRTUAL_ENV})
  message(STATUS "Using Python from virtual environment: $ENV{VIRTUAL_ENV}")
endif()

find_package(Python3 COMPONENTS Interpreter Development)

find_package(Eigen3 REQUIRED)

if(DEFINED ENV{VIRTUAL_ENV})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{VIRTUAL_ENV}")
endif()

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(DUBINS_SOURCES 
    src/cpp/lib/dubins_solver.cc
)

# Add bindings
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(cpp_solve MODULE ${DUBINS_SOURCES})
target_link_libraries(cpp_solve PRIVATE Eigen3::Eigen)
target_include_directories(cpp_solve PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Set the output name to match the Python import
set_target_properties(cpp_solve PROPERTIES
    PREFIX ""
    SUFFIX ".so"
)

# Install entire Python package directory
install(DIRECTORY "src/${SKBUILD_PROJECT_NAME}/" 
        DESTINATION "${SKBUILD_PROJECT_NAME}"
        FILES_MATCHING PATTERN "*.py" PATTERN "*.pyi"
        PATTERN "__pycache__" EXCLUDE)

install(TARGETS cpp_solve 
        LIBRARY DESTINATION "${SKBUILD_PROJECT_NAME}/pydubins")

# ---
# testing

if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  include(CTest)
endif()
