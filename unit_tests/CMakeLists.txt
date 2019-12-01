
project(cpphash_general_tests)

## Basic tests
add_executable(test_vml all_tests.cpp)
add_test(vml test_vml)

target_link_libraries(test_vml vml)


