
project(cpphash_general_tests)
  
include(ExternalProject)

ExternalProject_Add(benchmark
  GIT_REPOSITORY https://github.com/google/benchmark.git
  SOURCE_DIR "${CMAKE_SOURCE_DIR}/external/benchmark"
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/benchmark
	STEP_TARGETS configure build install 
  CMAKE_ARGS 
  	-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
		-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
		-DCMAKE_CONFIGURATION_TYPES=${CMAKE_CONFIGURATION_TYPES}
		-DBENCHMARK_ENABLE_GTEST_TESTS=OFF
)

ExternalProject_Add(Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  SOURCE_DIR "${CMAKE_SOURCE_DIR}/external/Catch2"
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/Catch2
  STEP_TARGETS configure build install 
  CMAKE_ARGS 
  	-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
		-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
		-DCMAKE_CONFIGURATION_TYPES=${CMAKE_CONFIGURATION_TYPES}
		-DCATCH_BUILD_TESTING=OFF
)


## Basic tests
add_executable(validity-cpp validity/main.cpp)
add_test(vml_validity-cpp validity-cpp)
add_dependencies(validity-cpp benchmark-install)
add_dependencies(validity-cpp Catch2-install)
message("Installed here: ${CMAKE_INSTALL_PREFIX}")
target_include_directories(validity-cpp PRIVATE "${CMAKE_INSTALL_PREFIX}/include")
target_link_libraries(validity-cpp vml)


