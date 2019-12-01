#pragma once

#include "types.hpp"
#include "types_sse.hpp"
#include "vml_fcn.hpp"

namespace vml {
#if VML_USE_SSE_AVX
namespace types = vml::sse_types;
#else
namespace types = vml::fallback_types;
#endif

using real_t         = float;
using quad_t         = types::quad_t<real>;
using vec2_t         = types::vec2_t<real>;
using vec3_t         = types::vec3_t<real>;
using vec3a_t        = types::vec3a_t<real>;
using vec4_t         = types::vec4_t<real>;
using plane_t        = types::plane_t<real>;
using quat_t         = types::quat_t<real>;
using axis_angle_t   = types::axis_angle_t<real>;
using polar_coord_t  = types::polar_coord_t<real>;
using eular_angles_t = types::eular_angles_t<real>;
using mat4_t         = types::mat4_t<real>;
using mat3_t         = types::mat3_t<real>;
using rect_t         = types::rect_t<real>;
using aabb_t         = types::aabb_t<real>;
using ivec2_t        = types::vec2_t<int>;
using ivec3_t        = types::vec3_t<int>;
using ivec4_t        = types::vec4_t<int>;
using irect_t        = types::rect_t<int>;

} // namespace vml