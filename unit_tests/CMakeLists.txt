
project(cpphash_general_tests)
  
include(ExternalProject)

ExternalProject_Add(benchmark
#  GIT_REPOSITORY https://github.com/google/benchmark.git
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
#  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
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
macro(validity_test test_name definitions compile_flags link_options)
  add_executable(vmltest-validity-${test_name} 
    validity/aabb.cpp
    validity/bounding_volume.cpp
    validity/euler_angles.cpp
    validity/frustum.cpp
    validity/intersect.cpp
    validity/plane.cpp
    validity/axis_angle.cpp
    validity/mat3.cpp
    validity/mat4.cpp
    validity/quad.cpp
    validity/main.cpp)
  add_test(validity-${test_name} vmltest-validity-${test_name})
  add_dependencies(vmltest-validity-${test_name} Catch2-install)
  target_include_directories(vmltest-validity-${test_name} PRIVATE "${CMAKE_SOURCE_DIR}/out/external/install/include")
  target_include_directories(vmltest-validity-${test_name} PRIVATE "${CMAKE_SOURCE_DIR}/include")
  target_compile_definitions(vmltest-validity-${test_name} PRIVATE ${definitions})
  target_compile_options(vmltest-validity-${test_name} PRIVATE ${compile_flags})
  target_compile_features(vmltest-validity-${test_name} PRIVATE cxx_std_17)
  target_link_options(vmltest-validity-${test_name} PRIVATE ${link_options})
endmacro()

set(VML_SSE3_CXX_FLAGS "")
set(VML_AVX_CXX_FLAGS "")
set(VML_COMMON_CXX_FLAGS "")
set(VML_COMMON_CXX_LINK_FLAGS "")

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  set(VML_SSE3_CXX_FLAGS "-msse3")
  set(VML_AVX_CXX_FLAGS "-mavx")
  set(VML_COMMON_CXX_FLAGS "-fsanitize=address;-fno-omit-frame-pointer;-ftest-coverage;-fprofile-instr-generate;-fcoverage-mapping")
  set(VML_COMMON_CXX_LINK_FLAGS "-fsanitize=address;-fprofile-instr-generate")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(VML_SSE3_CXX_FLAGS "-msse3")
  set(VML_AVX_CXX_FLAGS "-mavx")
  set(VML_COMMON_CXX_FLAGS "-fsanitize=address;-fno-omit-frame-pointer")
  set(VML_COMMON_CXX_LINK_FLAGS "-fsanitize=address")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
  set(VML_SSE3_CXX_FLAGS "-msse3")
  set(VML_AVX_CXX_FLAGS "-mavx")
  set(VML_COMMON_CXX_FLAGS "")
  set(VML_COMMON_CXX_LINK_FLAGS "")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  set(VML_SSE3_CXX_FLAGS "")
  set(VML_AVX_CXX_FLAGS "/arch:AVX")
  set(VML_COMMON_CXX_FLAGS "")
  set(VML_COMMON_CXX_LINK_FLAGS "")
endif()

validity_test("cpp" "" "${VML_COMMON_CXX_FLAGS}" "${VML_COMMON_CXX_LINK_FLAGS}")
validity_test("sse" "VML_USE_SSE_AVX=1;-DVML_USE_SSE_LEVEL=2" "${VML_COMMON_CXX_FLAGS};${VML_COMMON_CXX_FLAGS}" "${VML_COMMON_CXX_LINK_FLAGS}")
validity_test("sse3" "-DVML_USE_SSE_AVX=1;-DVML_USE_SSE_LEVEL=3" "${VML_SSE3_CXX_FLAGS};${VML_COMMON_CXX_FLAGS}" "${VML_COMMON_CXX_LINK_FLAGS}")
validity_test("avx" "-DVML_USE_SSE_AVX=1;-DVML_USE_SSE_LEVEL=4" "${VML_SSE3_CXX_FLAGS};${VML_AVX_CXX_FLAGS};${VML_COMMON_CXX_FLAGS}" "${VML_COMMON_CXX_LINK_FLAGS}")
