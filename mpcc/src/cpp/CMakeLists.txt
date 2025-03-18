# Create dubins library
add_library(dubins SHARED
    lib/dubins.cc
)

target_include_directories(dubins
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PUBLIC ${drake_INCLUDE_DIRS}
)

target_link_libraries(dubins PRIVATE drake::drake)

# Add bindings
find_package(pybind11 CONFIG REQUIRED)

set(PYDUBINS_SOURCES 
    bindings/dubins_py.cc
    lib/dubins.cc)

pybind11_add_module(pydubins MODULE ${PYDUBINS_SOURCES})
target_link_libraries(pydubins PRIVATE drake::drake dubins)
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