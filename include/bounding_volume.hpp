#pragma once

#include "mat4.hpp"
#include "quad.hpp"
#include "vec3a.hpp"

namespace vml {
struct bounds_info_t {
	vec3_t center  = {};
	vec3_t extends = {};
	float radius   = {};
};

struct bounds_info {
	static inline bounds_info& append(bounds_info_t& _,
	                                  bounds_info_t const& info) {
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

struct bounding_volume_t { /*! center wrt some coordinate system */
	vec3a_t center;
	/*! half extends dx/2, dy/2, dz/2 and linear radius in w */
	vec3a_t extends;
	/*! center wrt some coordinate system mostly 0,0,0 */
	vec3a_t orig_center;
	/*! half extends dx/2, dy/2, dz/2 and linear extends in w */
	vec4_t orig_extends_and_radius;
};

struct bounding_volume_t {
	inline static void nullify(bounding_volume_t&);
	inline static void set(bounding_volume_t& _, const vec3a_t& center,
	                       const vec3a_t& extends);
	inline static void set(bounding_volume_t& _, vec3a::pref center,
	                       vec3a::pref extends, float radius);
	inline static void update(bounding_volume_t& _, mat4::pref m);
	inline static void update(bounding_volume_t& _, float scale, quat::pref rot,
	                          vec3a::pref pos);
	inline static void update(bounding_volume_t& _, transform_t const& tf);
	inline static void update(bounding_volume_t& _, vec3a_t const* points,
	                          std::uint32_t count);
	inline static void update(bounding_volume_t& _, bounding_volume_t const&);
};

inline void bounding_volume_t::nullify(bounding_volume_t& _) {
	_.center = _.orig_center = _.extends = vec3a::zero();
	_.orig_extends_and_radius            = vec4::zero();
}

inline void bounding_volume_t::set(bounding_volume_t& _, const vec3a_t& center,
                                   const vec3a_t& extends) {
	_.center = _.orig_center = center;
	_.extends = _.orig_extends_and_radius = extends;

	_.orig_extends_and_radius = vec4::set_w(
	    _.orig_extends_and_radius, vec3a::length(vec3a::mul(extends, 2.0f)));
}

inline void bounding_volume_t::set(bounding_volume_t& _, vec3a::pref center,
                                   vec3a::pref extends, float radius) {
	_.center = _.orig_center = center;
	_.extends = _.orig_extends_and_radius = extends;

	_.orig_extends_and_radius = vec4::set_w(_.orig_extends_and_radius, radius);
}

inline void bounding_volume_t::update(bounding_volume_t& _, mat4::pref m) {
	// TODO test which is tighter avro's bound transform or this one
	_.center  = mat4::transform_assume_ortho(m, _.orig_center);
	_.extends = mat4::transform_bounds_extends(
	    m, vec3a::from_vec4(_.orig_extends_and_radius));
}

inline void bounding_volume_t::update(bounding_volume_t& _, float scale,
                                      quat::pref rot, vec3a::pref pos) {
	_.center  = vec3a::add(quat::transform(rot, _.orig_center), pos);
	_.extends = quat::transform_bounds_extends(
	    rot, vec3a::mul(_.orig_extends_and_radius, scale));
}

inline void bounding_volume_t::update(bounding_volume_t& _,
                                      transform_t const& tf) {
	update(_, transform::scaling(tf), transform::rotation(tf),
	       transform::translation(tf));
}

inline void bounding_volume_t::update(bounding_volume_t& _,
                                      vec3a_t const* points,
                                      std::uint32_t count) {
	aabb_t box = aabb::set(_.center, _.extends);
	for (std::uint32_t i = 0; i < count; i++)
		box = aabb::append(box, points[i]);
	set(_, aabb::center(box), aabb::half_size(box));
}

inline void bounding_volume_t::update(bounding_volume_t& _,
                                      bounding_volume_t const& vol) {
	vec3a_t a = vec3a::abs(vec3a::sub(_.center, vol.center));
	vec3a_t b = vec3a::add(_.center, vol.center);
	_.center = _.orig_center = vec3a::mul(b, 0.5f);
	_.extends                = _.orig_extends_and_radius =
	    vec3a::add(a, vec3a::add(vol.extends, _.extends));
	_.orig_extends_and_radius = vec4::set_w(
	    _.orig_extends_and_radius,
	    vec4::w(_.orig_extends_and_radius) +
	        (vec4::w(vol.orig_extends_and_radius) + vec3a::dot(a, a)));
}
} // namespace vml