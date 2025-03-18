find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(pydubins MODULE dubins_py.cc)
target_link_libraries(pydubins PUBLIC drake::drake dubins)
target_include_directories(pydubins PRIVATE ${drake_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
set_target_properties(pydubins PROPERTIES CXX_VISIBILITY_PRESET default)
install(TARGETS pydubins DESTINATION mpcc)

# add_test(NAME mpcc_bindings_test COMMAND
#     "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/mpcc_bindings_test.py")
# set_tests_properties(mpcc_bindings_test PROPERTIES
#     ENVIRONMENT "PYTHONPATH=$<TARGET_FILE_DIR:mpcc_bindings>:${DRAKE_PYTHONPATH}")