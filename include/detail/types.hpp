
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
template <typename scalar_t> using axis_angle_t   = quad_t<scalar_t>;
template <typename scalar_t> using polar_coord_t  = vec2_t<scalar_t>;
template <typename scalar_t> using eular_angles_t = vec3_t<scalar_t>;
template <typename scalar_t> using mat4_t = std::array<vec4_t<scalar_t>, 4>;
template <typename scalar_t> using mat3_t = std::array<vec3a_t<scalar_t>, 3>;
template <typename scalar_t> using rect_t = std::array<vec2_t<scalar_t>, 2>;
template <typename scalar_t> using aabb_t = std::array<vec3a_t<scalar_t>, 2>;

bool constexpr is_pref_cref = false;

} // namespace types
} // namespace vml