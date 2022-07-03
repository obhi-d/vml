
#pragma once

#include "detail/vml_commons.hpp"
#include <cassert>
#include <cmath>
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

namespace vml
{
static constexpr float k_round_off                = 0.000001f;
static constexpr float k_pi                       = 3.14159265358900f;
static constexpr float k_2pi                      = 2 * k_pi;
static constexpr float k_pi_by_2                  = k_pi / 2;
static constexpr float k_1_by_pi                  = 0.31830988618379067153776752674503f;
static constexpr float k_1_by_2pi                 = 0.15915494309189533576888376337254f;
static constexpr float k_degrees_to_radian_factor = 0.0174532925199f;

static constexpr float k_scalar_max = 3.402823466e+38f;
static constexpr float k_scalar_big = 999999999.0f;

static constexpr float k_const_epsilon      = 1.192092896e-06f;
static constexpr float k_const_epsilon_med  = 0.0009765625f;
static constexpr float k_const_epsilon_big  = 0.0625f;
static constexpr float k_max_relative_error = 0.005f;

/** vml function overrides */
inline float floor(float value)
{
  return std::floor(value);
}
inline float arc_tan(float value)
{
  return std::atan(value);
}
inline float arc_tan2(float y, float x)
{
  return std::atan2(y, x);
}
inline float arc_sin(float value)
{
  return std::asin(value);
}
inline float arc_cos(float value)
{
  return std::acos(value);
}
inline float abs(float value)
{
  return std::fabs(value);
}
inline float sqrt(float val)
{
  return std::sqrt(val);
}
inline float recip_sqrt(float val)
{
  return 1.f / std::sqrt(val);
}
inline float sin(float value)
{
  return std::sin(value);
}
template <typename T>
inline T sign(T x)
{
  return (x > 0) - (x < 0);
}
inline float cos(float value)
{
  return std::cos(value);
}
inline float tan(float value)
{
  return std::tan(value);
}
inline std::pair<float, float> sin_cos(float i_val)
{
  return {vml::sin(i_val), vml::cos(i_val)};
}
//-------------------------------------------------------
// Other utilities
inline std::uint32_t bit_pos(std::uint32_t v)
{
#ifdef _MSC_VER
  unsigned long index;
  _BitScanForward(&index, v);
  return index;
#elif defined(__clang__) || defined(__GNUC__)
  return __builtin_ffs(v) - 1;
#else
#error Undefined
#endif
}

inline std::uint32_t bit_count(std::uint32_t i)
{
#ifdef _MSC_VER
  return __popcnt(i);
#elif defined(__clang__) || defined(__GNUC__)
  return __builtin_popcount(i);
#else
  i = i - ((i >> 1) & 0x55555555);
  i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
  return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
#endif
}
inline std::uint32_t prev_pow2(std::uint32_t v)
{
  v--;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  return (++v) >> 1;
}
inline std::uint32_t next_pow2(std::uint32_t v)
{
  v--;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  return ++v;
}
inline bool is_pow2(std::uint32_t val)
{
  return (val & (val - 1)) == 0;
}
//-------------------------------------------------------
// inlined functions
// wrap angle between -pi & +pi
template <typename IntType>
inline IntType round_up(IntType number, IntType multiple)
{
  IntType remainder = number % multiple;
  if (remainder == 0)
    return number;
  return number + multiple - remainder;
}

inline float wrap_pi(float theta)
{
  theta += vml::k_pi;
  theta -= std::floor(theta * vml::k_1_by_2pi) * vml::k_2pi;
  theta -= vml::k_pi;
  return theta;
}
// float to fixed point conversion, float must be
// between 0-1
inline std::uint32_t float_to_fixed(float f, std::uint32_t n)
{
  // value * maxvalue
  return static_cast<std::uint32_t>(f * ((1 << (n)) - 1));
}
// fixed to float, float returned is between
// zero and one
inline float fixed_to_float(std::uint32_t f, std::uint32_t n)
{
  // value / maxvalue
  return static_cast<float>(f) / (float)((1 << n) - 1);
}
// fixed to fixed, fixed returned is between
// zero and one

inline std::uint32_t fixed_to_fixed(std::uint32_t f, std::uint32_t f_base, std::uint32_t req_base)
{
  // ((max(reqb)/max(fb)) * f)
  // the trick is if reqb < fb we can straightforwardly
  // divide by pow(2,reqb-fb), so
  if (req_base < f_base)
    return (f >> (req_base - f_base));
  return f * ((1 << req_base) - 1 / ((1 << f_base) - 1));
}

//-- half-float conversions.
//-- to speed up we can use tables
// (really small tables with mantissa, exponent, offset)
//-- this might be required elsewhere, for now just do it
// algorithmetically

inline std::uint16_t float_to_half_i(std::uint32_t i)
{
  // can use SSE here, but lets
  // do it naive way.
  int s = (i >> 16) & 0x00008000;
  int e = ((i >> 23) & 0x000000ff) - (127 - 15);
  int m = i & 0x007fffff;
  if (e <= 0)
  {
    if (e < -10)
      return 0;
    m = (m | 0x00800000) >> (1 - e);

    return s | (m >> 13);
  }
  else if (e == 0xff - (127 - 15))
  {
    if (m == 0) // Inf
      return s | 0x7c00;
    else // NAN
    {
      m >>= 13;
      return s | 0x7c00 | m | (m == 0);
    }
  }
  else
  {
    if (e > 30) // Overflow
      return s | 0x7c00;
    return s | (e << 10) | (m >> 13);
  }
}

inline std::uint16_t float_to_half(float f)
{
  return float_to_half_i(*(std::uint32_t*)&f);
}

inline std::uint32_t half_to_float_i(std::uint16_t y)
{
  // can use SSE here, but lets
  // do it naive way.
  int s = (y >> 15) & 0x00000001;
  int e = (y >> 10) & 0x0000001f;
  int m = y & 0x000003ff;

  if (e == 0)
  {
    if (m == 0) // Plus or minus zero
    {
      return s << 31;
    }
    else // Denormalized number -- renormalize it
    {
      while (!(m & 0x00000400))
      {
        m <<= 1;
        e -= 1;
      }

      e += 1;
      m &= ~0x00000400;
    }
  }
  else if (e == 31)
  {
    if (m == 0) // Inf
    {
      return (s << 31) | 0x7f800000;
    }
    else // NaN
    {
      return (s << 31) | 0x7f800000 | (m << 13);
    }
  }

  e = e + (127 - 15);
  m = m << 13;
  return (s << 31) | (e << 23) | m;
}

inline float half_to_float(std::uint16_t y)
{
  union
  {
    float         f;
    std::uint32_t i;
  } o;
  o.i = half_to_float_i(y);
  return o.f;
}

inline float to_radians(float value)
{
  return k_degrees_to_radian_factor * (value);
}

inline float to_degrees(float value)
{
  return (value)*57.295779513f;
}

/* Famous fast reciprocal sqrt */
inline float fast_recip_sqrt(float x)
{
  std::int32_t i;
  float        y, r;
  y = x * 0.5f;
  i = *(std::int32_t*)(&x);
  i = 0x5f3759df - (i >> 1);
  r = *(float*)(&i);
  r = r * (1.5f - r * r * y);
  return r;
}

/* sin of angle in the range of [0, pi/2]*/
inline float sin_of_ang_between_0_to_half_pi(float a)
{
  float s, t;
  s = a * a;
  t = -2.39e-08f;
  t *= s;
  t += 2.7526e-06f;
  t *= s;
  t += -1.98409e-04f;
  t *= s;
  t += 8.3333315e-03f;
  t *= s;
  t += -1.666666664e-01f;
  t *= s;
  t += 1.0f;
  t *= a;
  return t;
}

/* Arc tan when x and y are positives */
inline float arc_tan_positive_xy(float y, float x)
{
  float a, d, s, t;
  if (y > x)
  {
    a = -x / y;
    d = vml::k_pi / 2;
  }
  else
  {
    a = y / x;
    d = 0.0f;
  }
  s = a * a;
  t = 0.0028662257f;
  t *= s;
  t += -0.0161657367f;
  t *= s;
  t += 0.0429096138f;
  t *= s;
  t += -0.0752896400f;
  t *= s;
  t += 0.1065626393f;
  t *= s;
  t += -0.1420889944f;
  t *= s;
  t += 0.1999355085f;
  t *= s;
  t += -0.3333314528f;
  t *= s;
  t += 1.0f;
  t *= a;
  t += d;
  return t;
}

inline bool almost_equals_ulps(float i_a, float i_b, int max_ulps)
{
  assert(sizeof(float) == sizeof(int));
  if (i_a == i_b)
    return true;
  int a_int = *(int*)&i_a;
  // Make a_int lexicographically ordered as a twos-complement int
  if (a_int < 0)
    a_int = 0x80000000 - a_int;
  // Make b_int lexicographically ordered as a twos-complement int
  int b_int = *(int*)&i_b;
  if (b_int < 0)
    b_int = 0x80000000 - b_int;

  int int_diff = std::abs(a_int - b_int);
  if (int_diff <= max_ulps)
    return true;
  return false;
}

inline bool almost_equals_rel_or_abs(float i_a, float i_b, float max_diff, float max_rel_diff)
{
  float diff = fabs(i_a - i_b);
  if (diff < max_diff)
    return true;
  i_a           = fabs(i_a);
  i_b           = fabs(i_b);
  float largest = i_b > i_a ? i_b : i_a;
  if (diff <= largest * max_rel_diff)
    return true;
  return false;
}

template <typename type>
inline void clamp(type& clampwhat, type lowvalue, type hivalue)
{
  clampwhat = std::max(lowvalue, std::min(clampwhat, hivalue));
}
} // namespace vml
