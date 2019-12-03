#pragma once
#include "mat_base.hpp"
#include "vec3a.hpp"

namespace vml {
namespace detail {
struct aabb_traits {
	using type = types::aabb_t<float>;
	using ref  = type&;
	using pref = std::conditional_t<types::is_pref_cref, type const&, type>;
	using cref = type const&;
	using scalar_type = float;
	using row_type    = types::vec4_t<scalar_type>;

	enum { element_count = 8 };
	enum { row_count = 2 };
	enum { column_count = 4 };
};
} // namespace detail

struct aabb : public mat_base<detail::aabb_traits> {
	static inline bool is_valid(pref box);
	static inline vec3a_t center(pref box);
	static inline vec3a_t size(pref box);
	static inline vec3a_t half_size(pref box);
	static inline vec3a_t corner(pref box, unsigned int i);
	static inline type append(pref box, vec3a::pref point);
	static inline type append(pref box, pref other);
	static inline type set(vec3a::pref center, vec3a::pref extends);
};
inline bool vml::aabb::is_valid(pref box) {
#if VML_USE_SSE_AVX
	return _mm_movemask_epi8(_mm_castps_si128(_mm_cmplt_ps(box[1], box[0]))) == 0;
#else
	for (int i = 0; i < 3; ++i)
		if (box[1][i] < box[0][i])
			return false;
	return true;
#endif
}
inline vec3a_t aabb::center(pref box) {
	return vec3a::half(vec3a::add(box[1], box[0]));
}
inline vec3a_t aabb::size(pref box) { return vec3a::sub(box[1], box[0]); }
inline vec3a_t aabb::half_size(pref box) {
	return vec3a::half(vec3a::sub(box[1], box[0]));
}
inline vec3a_t aabb::corner(pref box, unsigned int i) {
	return vec3a::set(vec3a::x(box[((i >> 2) & 1)]),
	                  vec3a::y(box[((i >> 1) & 1)]),
	                  vec3a::z(box[((i >> 0) & 1)]));
}
inline aabb::type aabb::append(pref b, vec3a::pref point) {
#if VML_USE_SSE_AVX
	return {_mm_min_ps(b[0], point), _mm_max_ps(b[1], point)};
#else
	aabb_t box = b;
	for (int i = 0; i < 3; ++i) {
		if (box[0][i] > point[i])
			box[0][i] = point[i];
		if (box[1][i] < point[i])
			box[1][i] = point[i];
	}
	return box;
#endif
}
inline aabb::type aabb::append(pref box, pref other) {
#if VML_USE_SSE_AVX
	return {_mm_min_ps(box[0], other[0]), _mm_max_ps(box[1], other[1])};
#else
	aabb ret;
	for (int i = 0; i < 3; ++i) {
		ret[0][i] = (box[0][i] > other[0][i]) ? other[0][i] : box[0][i];
		ret[1][i] = (box[1][i] < other[1][i]) ? box[1][i] : other[1][i];
	}
	return ret;
#endif
}
inline aabb::type aabb::set(vec3a::pref center, vec3a::pref extends) {
	return {vec3A::sub(center, extends), vec3A::add(center, extends)};
}
} // namespace vml