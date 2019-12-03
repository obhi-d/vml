#pragma once

#if defined(_MSC_VER)
#define VML_EXPORT __declspec(dllexport)
#define VML_IMPORT __declspec(dllimport)
#else
#define VML_EXPORT __attribute__((visibility("default")))
#define VML_IMPORT __attribute__((visibility("default")))
#endif

#ifdef VML_DLL_IMPL
#ifdef VML_EXPORT_SYMBOLS
#define VML_API VML_EXPORT
#else
#define VML_API VML_IMPORT
#endif
#else
#define VML_API
#endif

#if VML_USE_SSE_AVX
#include <emmintrin.h>
#include <smmintrin.h>
#include <xmmintrin.h>

#ifdef _MSC_VER
#define vml_cast_i_to_v(v) _mm_castsi128_ps(v)
#define vml_cast_v_to_i(v) _mm_castps_si128(v)
#else
#define vml_cast_i_to_v(v) (__m128)(v)
#define vml_cast_v_to_i(v) (__m128i)(v)
#endif

#define VML_CLEAR_W_VEC                                                        \
	vml_cast_i_to_v(_mm_set_epi32(0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000))
#define VML_XYZ0_W1_VEC vml_cast_i_to_v(_mm_set_epi32(0, 0, 0, 1))
#define VML_CLEAR_XYZ_VEC vml_cast_i_to_v(_mm_set_epi32(0, 0, 0, 0xFFFFFFFF))

#endif

#ifdef _MSC_VER
#include <intrin.h>
#elif defined(__clang__) || defined(__GNUC__)
#endif

#include <array>
#include <cstdint>
#include <limits>
#include <memory>

namespace vml {

void* allocate(std::size_t amount, std::size_t alignment) {
#ifdef _MSC_VER
		return _aligned_malloc(amount, alignment);
#else
		return aligned_alloc(amount, alignment);
#endif
}	

void deallocate(void* mem, std::size_t size) {
#ifdef _MSC_VER
		return _aligned_free(mem);
#else
		return free(mem);
#endif
}
}