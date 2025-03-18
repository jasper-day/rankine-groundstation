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
add_subdirectory(bindings)