#pragma once

#include "vec3.hpp"
#include "mat4.hpp"
#include "quad.hpp"
#include "sphere.hpp"
#include "transform.hpp"

namespace vml {
struct bounds_info_t {
	vec3_t center       = {};
	vec3_t half_extends = {};
	float radius        = {};
};

struct bounds_info {
	static inline bounds_info_t& append(bounds_info_t& _,
	                                  bounds_info_t const& info) {
		if (info.radius > 0) {
			if (_.radius <= 0)
				_ = info;
			else {
				vec3_t a = vec3::abs(vec3::sub(_.center, info.center));
				vec3_t b   = vec3::add(_.center, info.center);
				_.center = vec3::mul(b, 0.5f);
				_.half_extends =
				    vec3::add(a, vec3::add(_.half_extends, info.half_extends));
				_.radius += (info.radius + vec3::dot(a, a));
			}
		}
		return _;
	}
};

struct bounding_volume_t { /*! center wrt some coordinate system */
	sphere_t spherical_vol;
	/*! half half_extends dx/2, dy/2, dz/2 */
	vec3a_t half_extends;
	/*! center wrt some coordinate system mostly 0,0,0 */
	sphere_t orig_spherical_vol;
	/*! half half_extends dx/2, dy/2, dz/2 */
	vec4_t orig_half_extends;
};

struct bounding_volume {

	inline static vec3a_t center(bounding_volume_t const&);
	inline static vec3a_t half_extends(bounding_volume_t const&);
	inline static float radius(bounding_volume_t const&);
	inline static quad_t vradius(bounding_volume_t const&);

	inline static void nullify(bounding_volume_t&);
	inline static void from_box(bounding_volume_t& _, vec3a_t const& center,
	                            vec3a_t const& half_extends);
	inline static void set(bounding_volume_t& _, vec3a_t const& center,
	                       vec3a_t const& half_extends, float radius);
	inline static void set(bounding_volume_t& _, sphere_t const& sphere,
	                       vec3a_t const& half_extends);
	inline static void update(bounding_volume_t& _, mat4::pref m);
	inline static void update(bounding_volume_t& _, float scale, quat::pref rot,
	                          vec3a::pref translation);
	inline static void update(bounding_volume_t& _, transform_t const& tf);
	inline static void update(bounding_volume_t& _, vec3a_t const* points,
	                          std::uint32_t count);
	inline static void update(bounding_volume_t& _, bounding_volume_t const&);
};

inline vec3a_t bounding_volume::center(bounding_volume_t const& _) {
	return sphere::center(_.spherical_vol);
}

inline vec3a_t bounding_volume::half_extends(bounding_volume_t const& _) {
	return _.half_extends;
}

inline float bounding_volume::radius(bounding_volume_t const& _) {
	return sphere::radius(_.spherical_vol);
}

inline quad_t bounding_volume::vradius(bounding_volume_t const& _) {
	return sphere::vradius(_.spherical_vol);
}

inline void bounding_volume::nullify(bounding_volume_t& _) {
	_.spherical_vol = _.orig_spherical_vol = _.half_extends =
	    _.orig_half_extends                = vec4::zero();
}

inline void bounding_volume::from_box(bounding_volume_t& _,
                                      vec3a_t const& center,
                                      vec3a_t const& half_extends) {
	_.spherical_vol = _.orig_spherical_vol = sphere::set(center, sphere::max_radius(vec3a::mul(half_extends, 2.0f));
	_.half_extends = _.orig_half_extends = half_extends;
}

inline void bounding_volume::set(bounding_volume_t& _, vec3a_t const& center,
                                 vec3a_t const& half_extends, float radius) {
	_.spherical_vol = _.orig_spherical_vol = sphere::set(center, radius);
	_.half_extends = _.orig_half_extends = half_extends;
}

inline void bounding_volume::set(bounding_volume_t& _,
                                 sphere_t const& sphere_arg,
                                 vec3a_t const& half_extends) {
	_.spherical_vol = _.orig_spherical_vol = sphere_arg;
	_.half_extends = _.orig_half_extends = half_extends;
}

inline void bounding_volume::update(bounding_volume_t& _, mat4::pref m) {
	// TODO test which is tighter avro's bound transform or this one
	_.spherical_vol = sphere::scale_radius(
	    sphere::set(
	        mat4::transform_assume_ortho(m, sphere::center(_.orig_spherical_vol)),
	        sphere::radius(_.spherical_vol)),
	    mat4::max_scale(_));

	_.half_extends = mat4::transform_bounds_extends(m, _.orig_half_extends);
}

inline void bounding_volume::update(bounding_volume_t& _, float scale,
                                    quat::pref rot, vec3a::pref translation) {

	_.spherical_vol = sphere::scale_radius(
	    sphere::set(
	        vec3a::add(quat::transform(rot, sphere::center(_.orig_spherical_vol)),
	                   translation),
	        sphere::radius(_.spherical_vol)),
	    scale);
	_.half_extends = quat::transform_bounds_extends(
	    rot, vec3a::mul(_.orig_half_extends, scale));
}

inline void bounding_volume::update(bounding_volume_t& _,
                                    transform_t const& tf) {
	update(_, transform::scaling(tf), transform::rotation(tf),
	       transform::translation(tf));
}

inline void bounding_volume::update(bounding_volume_t& _, vec3a_t const* points,
                                    std::uint32_t count) {
	aabb_t box = aabb::set(center(_), half_extends(_));
	for (std::uint32_t i = 0; i < count; i++)
		box = aabb::append(box, points[i]);
	from_box(_, aabb::center(box), aabb::half_size(box));
}

inline void bounding_volume::update(bounding_volume_t& _,
                                    bounding_volume_t const& vol) {
	vec3a_t center_this  = center(_);
	vec3a_t center_other = center(vol);

	vec3a_t a = vec3a::abs(vec3a::sub(center_this, center_other));
	vec3a_t b = vec3a::add(center_this, center_other);

	_.spherical_vol =
	    vec3a::set_w(vec3a::half(b),
	                 vec3a::add_x(vec3a::add_x(vec3a::vradius(vol.spherical_vol),
	                                           vec3a::vradius(_.spherical_vol)),
	                              vec3a::vdot(a, a)));
	_.half_extends = vec3a::add(a, vec3a::add(vol.half_extends, _.half_extends));
	_.orig_spherical_vol = _.spherical_vol;
	_.orig_half_extends  = _.half_extends;
}
} // namespace vml