
#include "vml_commons.hpp"

namespace vml {

namespace sse_types {
#if VML_USE_SSE_AVX

template <typename scalar_t>
struct quad_type { using type = std::array<scalar_t, 4>; };
template <> struct quad_type<double> { using type = __m256; };
template <> struct quad_type<float> { using type = __m128; };
template <> struct quad_type<std::int32_t> { using type = __m128i; };
template <> struct quad_type<std::int64_t> { using type = __m256i; };

template <typename scalar_t> using quad_t         = typename quad_type<scalar_t>::type;
template <typename scalar_t> using vec2_t         = std::array<scalar_t, 2>;
template <typename scalar_t> using vec3_t         = std::array<scalar_t, 3>;
template <typename scalar_t> using vec3a_t        = quad_t<scalar_t>;
template <typename scalar_t> using vec4_t         = quad_t<scalar_t>;
template <typename scalar_t> using plane_t        = quad_t<scalar_t>;
template <typename scalar_t> using quat_t         = quad_t<scalar_t>;
template <typename scalar_t> using axis_angle_t   = quad_t<scalar_t>;
template <typename scalar_t> using polar_coord_t  = vec2_t<scalar_t>;
template <typename scalar_t> using eular_angles_t = vec3_t<scalar_t>;
template <typename scalar_t> using rect_t = std::array<vec2_t<scalar_t>, 2>;
template <typename scalar_t> using aabb_t = std::array<vec3a_t<scalar_t>, 2>;

template <typename scalar_t>
struct mat4_t {
  //! @note This is possibly non isoc++, but knowing it will work for all compilers
  //! is why I am putting it here
  union {
    vec4_t<scalar_t> r[4];
    scalar_t m[16];
    scalar_t e[4][4];
  };
};

template <typename scalar_t>
struct mat3_t {
  //! @note This is possibly non isoc++, but knowing it will work for all compilers
  //! is why I am putting it here
  union {
    vec4_t<scalar_t> r[3];
    scalar_t m[12];
    scalar_t e[3][4];
  };
};

bool constexpr is_pref_cref = false;

#endif
} // namespace types

} // namespace vml