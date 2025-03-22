# SPDX-License-Identifier: MIT-0

cmake_minimum_required(VERSION 3.16)
project(mpcc LANGUAGES CXX)

# N.B. This is a temporary flag. It only really applies to Linux, as Mac
# does not need X11.
option(RUN_X11_TESTS "Run tests that require X11" OFF)


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

set(DUBINS_SOURCES 
    src/cpp/lib/dubins_segment.cc
    src/cpp/lib/dubins_path.cc
    src/cpp/lib/dubins_solver.cc
)

# Create dubins library
add_library(dubins SHARED ${DUBINS_SOURCES})

target_include_directories(dubins
    PUBLIC src/cpp/include
    PUBLIC ${drake_INCLUDE_DIRS}
)

target_link_libraries(dubins PRIVATE drake::drake)
target_link_directories(dubins PRIVATE ${drake_LIBRARY_DIRS})

# Add bindings
find_package(pybind11 CONFIG REQUIRED)

set(PYDUBINS_SOURCES ${DUBINS_SOURCES} src/cpp/bindings/dubins_py.cc)

pybind11_add_module(pydubins MODULE ${PYDUBINS_SOURCES})
target_link_libraries(pydubins PRIVATE drake::drake dubins)
target_link_directories(pydubins PRIVATE ${drake_LIBRARY_DIRS})
target_include_directories(pydubins PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${drake_INCLUDE_DIRS}
)

# Set the output name to match the Python import
set_target_properties(pydubins PROPERTIES
    PREFIX ""
    SUFFIX ".so"
)

install(TARGETS pydubins 
  LIBRARY DESTINATION "${SKBUILD_PROJECT_NAME}")

# main code lives here, including library definitions. Edit library definitions in src/cpp/CMakeLists.txt
# add_subdirectory(tests)

# Install pure-python files
file(GLOB_RECURSE PYTHON_SOURCES "src/${SKBUILD_PROJECT_NAME}/*.py")
file(GLOB_RECURSE PYTHON_STUBS "src/${SKBUILD_PROJECT_NAME}/*.pyi")

install(FILES ${PYTHON_SOURCES} ${PYTHON_STUBS} DESTINATION ${SKBUILD_PROJECT_NAME})

# ---
# testing

if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  include(CTest)
endif()

# third party defines google test gtest
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
  add_subdirectory(third_party)
  add_subdirectory(tests)
endif()