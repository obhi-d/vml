#pragma once

#include "quad.hpp"
#include "vec3.hpp"

namespace vml {
struct bounds_info {
	vec3_t center  = {};
	vec3_t extends = {};
	float radius   = {};

	static inline bounds_info& append(bounds_info& _, bounds_info const& info) {
		if (info.radius > 0) {
			if (_.radius <= 0)
				_ = info;
			else {
				vec3_t a  = vec3::abs(vec3::sub(_.center, info.center));
				vec3 b    = vec3::add(_.center, info.center);
				_.center  = vec3::mul(b, 0.5f);
				_.extends = vec3::add(a, vec3::add(_.extends, info.extends));
				_.radius += (info.radius + vec3::dot(a, a));
			}
		}
		return _;
	}
};
struct bounding_volume {

	inline static void nullify(bounding_volume&);
	inline static void set(bounding_volume& _, const vec3a_t& center,
	                       const vec3a_t& extends);
	inline static void set(bounding_volume& _, const vVec3A& center,
	                       const vVec3A& extends, float radius);
	inline static void update(bounding_volume& _, Mat4::pref m);
	inline static void update(bounding_volume& _, float scale, Quat::pref rot,
	                          const vVec3A& pos);
	inline static void update(bounding_volume& _, const vVec3A* points,
	                          uint32 maxPoints);
	inline static void update(bounding_volume& _, const BoundingVolume&);

	/*! center wrt some coordinate system */
	vec3a_t center;
	/*! half extends dx/2, dy/2, dz/2 and linear radius in w */
	vec3a_t extends;
	/*! center wrt some coordinate system mostly 0,0,0 */
	vec3a_t orig_center;
	/*! half extends dx/2, dy/2, dz/2 and linear extends in w */
	union {
		vec4_t orig_extends_and_radius;
		struct {
			vec3_t orig_extends;
			float radius;
		};
	};
};
} // namespace vml
