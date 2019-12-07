#pragma once
#include "vec3a.hpp"

namespace vml {

struct sphere : public quad {
	using quad::set;
	inline static sphere_t set(vec3a::pref _, float radius);
	inline static float radius(quad::pref _);
	inline static vec3a_t center(quad::pref _);
	inline static float max_radius(vec3a_t const&);
	inline static sphere_t scale_radius(sphere::pref, float scale);
};
inline sphere_t sphere::set(vec3a::pref _, float radius) { return quad::set_w(_, radius); }
inline float sphere::radius(quad::pref _) { return quad::w(_); }
inline vec3a_t sphere::center(quad::pref _) { return vec3a::from_vec4(_); }
inline float sphere::max_radius(vec3a_t const& _) {
	return vec3a::length(_);
}
inline static sphere_t sphere::scale_radius(sphere::pref p, float scale) {
	return quad::mul(p, quat::set(1.0f, 1.0f, 1.0f, scale));
}
} // namespace vml