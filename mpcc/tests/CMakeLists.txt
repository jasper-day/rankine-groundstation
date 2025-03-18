# test dubins path
add_executable(test_dubins test_dubins.cpp)
target_link_libraries(test_dubins PRIVATE drake::drake gtest dubins)
add_test(NAME test_dubins COMMAND test_dubins)
set_tests_properties(test_dubins PROPERTIES LABELS "dubins" TIMEOUT 10)