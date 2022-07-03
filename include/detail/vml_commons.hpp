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

#define VML_CLEAR_W_VEC   vml_cast_i_to_v(_mm_set_epi32(0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF))
#define VML_XYZ0_W1_VEC   _mm_set_ps(1.0f, 0.0f, 0.0f, 0.0f)
#define VML_CLEAR_XYZ_VEC vml_cast_i_to_v(_mm_set_epi32(0xFFFFFFFF, 0, 0, 0))

#endif

#ifdef _MSC_VER
#include <intrin.h>
#elif defined(__clang__) || defined(__GNUC__)
#endif

#include <array>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <memory>

//! Integer representation of a floating-point value.
#define VML_MK_INT(x) (*(uint32*)(&(x)))
//! Signed integer representation of a floating-point value.
#define VML_MK_SIGNED_INT(x) (*(int32*)(&(x)))
//! Absolute integer representation of a floating-point value
#define VML_MK_ABS_INT(x) (VML_MK_INT(x) & 0x7fffffff)
//! Floating-point representation of an integer value.
#define VML_MK_FLOAT(x) (*(float*)(&(x)))
//! Check the sign of a floating point number.
#define VML_IS_FLOAT_NEGETIVE(x) (VML_MK_INT(x) & 0x80000000)
//! Is a undefined quantity
#define VML_IS_FLOAT_NAN(value) ((VML_MK_INT(value) & 0x7f800000) == 0x7f800000)
//! Is indeterminate
#define VML_IS_FLOAT_IND(value) (VML_MK_INT(value) == 0xffc00000)
//! Is +infinity
#define VML_IS_FLOAT_PINF(value) (VML_MK_INT(value) == 0x7f800000)
//! Is -infinity
#define VML_IS_FLOAT_NINF(value) (VML_MK_INT(value) == 0xff800000)
//! Is float valid
#define VML_IS_VALID_FLOAT(value)                                                                                      \
  (!(VML_IS_FLOAT_NAN(value) || VML_IS_FLOAT_IND(value) || VML_IS_FLOAT_PINF(value) || VML_IS_FLOAT_NINF(value)))

//! More than
#define VML_FLOAT_MORETHAN(a, b)    (VML_MK_ABS_INT(a) > VML_MK_INT(b))
#define VML_ABSFLOAT_MORETHAN(a, b) (VML_MK_INT(a) > VML_MK_INT(b))
// some comparison macros for float that can prove efficient
#define VML_FLOAT_MORETHANEQZERO(a) !VML_IS_FLOAT_NEGETIVE(a)
#define VML_FLOAT_MORETHANZERO(a)   ((a) > 0)
#define VML_FLOAT_LESSTHANEQZERO(a) ((a) <= 0)
#define VML_FLOAT_LESSTHANZERO(a)   VML_IS_FLOAT_NEGETIVE(a)

#define VML_PIXEL_ALIGNED(v) (float)(int32)(v + (v > 0 ? 0.5f : -0.5f))
//! roll degree [0,360]
#define VML_ROLL_360(a)                                                                                                \
  if (VML_IS_FLOAT_NEGETIVE(a))                                                                                        \
    a = 360;                                                                                                           \
  else if (a > 360)                                                                                                    \
    a = 0;
//! roll radians [0,2pi]
#define VML_ROLL_2PI(a)                                                                                                \
  if (VML_IS_FLOAT_NEGETIVE(a))                                                                                        \
    a = vml::k_2pi;                                                                                                    \
  else if (a > vml::k_2pi)                                                                                             \
    a = 0;
//! roll degree [-180,180]
#define VML_ROLL_180(a)                                                                                                \
  if (a < -180.0f)                                                                                                     \
    a = 180.0f;                                                                                                        \
  else if (a > 180.0f)                                                                                                 \
    a = -180.0f;
//! roll pi
#define VML_ROLL_PI(a)                                                                                                 \
  if (a < -(vml::k_pi))                                                                                                \
    a = (vml::k_pi);                                                                                                   \
  else if (a > (vml::k_pi))                                                                                            \
    a = -(vml::k_pi);
//! blocks the range of angle to straight up/down
#define VML_ROLL_PIBY2(a)                                                                                              \
  if (a < -vml::k_pi_by_2 - vml::k_const_epsilon)                                                                      \
    a = -vml::k_pi_by_2 - vml::k_const_epsilon;                                                                        \
  else if (a > vml::k_pi_by_2 - vml::k_const_epsilon)                                                                  \
    a = vml::k_pi_by_2 - vml::k_const_epsilon;
//! grades to radians
#define VML_DEG2RAD(a) (a * 0.0174532925f)
//! radians to grades
#define VML_RAD2DEG(a) (a * 57.295779513f)
//! if floats are equal with round off
#define VML_FLOAT_TOLERANCE_EQUAL(v1, v2, roundoff) ((v2 - roundoff) <= v1 && v1 <= (v2 + roundoff))
//! random number generator
#define VML_FLOAT_RAND(iLow, iHigh)                                                                                    \
  (float)((float)(rand() % iLow) / iHigh) /* further apart iHigh-iLow more near 0 the ans goes */
#define VML_FLOAT_RAND_PREC(nLow, nHigh) (((1.0f / ((rand() % 100) + 1)) * ((nHigh) - (nLow))) + (nLow))
#define VML_INT_RAND(low, high)          (rand() % ((high) - (low) + 1) + (low))
/* swap float */
#define VML_FLOAT_SWAP(fV1, fV2) VML_MK_INT(fV1) ^= VML_MK_INT(fV2) ^= VML_MK_INT(fV1) ^= VML_MK_INT(fV2);

namespace vml
{

template <typename pointer_arg = void>
inline pointer_arg* allocate(std::size_t amount, std::size_t alignment)
{
#ifdef _MSC_VER
  return reinterpret_cast<pointer_arg*>(_aligned_malloc(amount, alignment));
#else
  return reinterpret_cast<pointer_arg*>(aligned_alloc(alignment, amount));
#endif
}

template <typename pointer_arg = void>
inline void deallocate(pointer_arg* mem, std::size_t size)
{
#ifdef _MSC_VER
  return _aligned_free(mem);
#else
  return free(mem);
#endif
}
} // namespace vml