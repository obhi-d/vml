
project(cpphash_general_tests)
  
include(ExternalProject)

ExternalProject_Add(benchmark
  GIT_REPOSITORY https://github.com/google/benchmark.git
  SOURCE_DIR "${CMAKE_SOURCE_DIR}/external/benchmark"
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/benchmark
	STEP_TARGETS install 
  CMAKE_ARGS 
  	-DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/out/external/install
		-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
		-DCMAKE_CONFIGURATION_TYPES=${CMAKE_CONFIGURATION_TYPES}
		-DBENCHMARK_ENABLE_GTEST_TESTS=OFF
)

ExternalProject_Add(Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  SOURCE_DIR "${CMAKE_SOURCE_DIR}/external/Catch2"
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/Catch2
  STEP_TARGETS install 
  CMAKE_ARGS 
  	-DCMAKE_INSTALL_PREFIX=${CMAKE_SOURCE_DIR}/out/external/install
		-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
		-DCMAKE_CONFIGURATION_TYPES=${CMAKE_CONFIGURATION_TYPES}
		-DCATCH_BUILD_TESTING=OFF
)


## Basic tests
function(validity_test name definitions compile_flags)
  add_executable(vmltest-validity-${name} 
    validity/aabb.cpp
    validity/main.cpp)
  add_test(vmltest-validity-${name} validity-cpp)
  add_dependencies(vmltest-validity-${name} benchmark-install)
  add_dependencies(vmltest-validity-${name} Catch2-install)
    target_include_directories(vmltest-validity-${name} PRIVATE "${CMAKE_SOURCE_DIR}/out/external/install/include")
  target_link_libraries(vmltest-validity-${name} vml)  
  if (NOT "${definitions}" MATCHES "")
    target_compile_definitions(vmltest-validity-${name} PRIVATE ${definitions})
  endif()
  if (NOT "${compile_flags}" MATCHES "")
    target_compile_options(vmltest-validity-${name} PRIVATE ${compile_flags})
  endif()
endfunction()

validity_test("cpp" "" "")
validity_test("sse" "VML_USE_SSE_AVX=1 -DVML_USE_SSE_LEVEL=2" "")
validity_test("sse3" "-DVML_USE_SSE_AVX=1 -DVML_USE_SSE_LEVEL=3" "")
validity_test("avx" "-DVML_USE_SSE_AVX=1 -DVML_USE_SSE_LEVEL=4" "")
