#pragma once

#include <array>
#include <cstdint>

namespace vml {

namespace fallback_types {

template <typename scalar_t = float> using quad_t = std::array<scalar_t, 4>;
template <typename scalar_t> using vec2_t         = std::array<scalar_t, 2>;
template <typename scalar_t> using vec3_t         = std::array<scalar_t, 3>;
template <typename scalar_t> using vec3a_t        = quad_t<scalar_t>;
template <typename scalar_t> using vec4_t         = quad_t<scalar_t>;
template <typename scalar_t> using plane_t        = quad_t<scalar_t>;
template <typename scalar_t> using quat_t         = quad_t<scalar_t>;
template <typename scalar_t> using sphere_t       = quad_t<scalar_t>;
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
    std::array<scalar_t, 16> m;
    scalar_t e[4][4];
  };

  mat4_t() {};
  mat4_t(std::initializer_list<float> v) : m(v) {}
};

template <typename scalar_t>
struct mat3_t {
  //! @note This is possibly non isoc++, but knowing it will work for all compilers
  //! is why I am putting it here
  union {
    vec4_t<scalar_t> r[3];
    std::array<scalar_t, 12> m;
    scalar_t e[3][4];
  };
  mat3_t() {};
  mat3_t(std::initializer_list<float> v) : m(v) {}
};

bool constexpr is_pref_cref = false;

} // namespace types
} // namespace vml