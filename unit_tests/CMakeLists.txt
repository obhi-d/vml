
project(vml_general_tests)
  
include(FetchContent)


FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG origin/v2.x
)

FetchContent_MakeAvailable(Catch2)

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
    validity/quat.cpp
    validity/transform.cpp
    validity/vec.cpp
    validity/main.cpp
    )
  add_test(validity-${test_name} vmltest-validity-${test_name})
  target_link_libraries(vmltest-validity-${test_name} vml::vml)
  target_link_libraries(vmltest-validity-${test_name} Catch2::Catch2)
  target_compile_definitions(vmltest-validity-${test_name} PRIVATE ${definitions})
  target_compile_options(vmltest-validity-${test_name} PRIVATE ${compile_flags})
  target_compile_features(vmltest-validity-${test_name} PRIVATE cxx_std_20)
  target_link_options(vmltest-validity-${test_name} PRIVATE ${link_options})
endmacro()

set(VML_SSE3_CXX_FLAGS "")
set(VML_AVX_CXX_FLAGS "")
set(VML_COMMON_CXX_FLAGS "")
set(VML_COMMON_CXX_LINK_FLAGS "")

set(CMAKE_CXX_STANDARD 20)
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
