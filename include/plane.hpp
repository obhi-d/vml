#pragma once

#include "vec3a.hpp"

namespace vml {

struct plane : public quad {
	using quad::cref;
	using quad::pref;
	using quad::ref;
	using quad::row_type;
	using quad::scalar_type;
	using quad::type;

	static inline type normalize(pref p);
	static inline scalar_type dot(pref p, pref vec3a);
	static inline vec3a_t vdot(pref p, pref vec3a);
	static inline scalar_type dot_with_normal(pref p, pref vec3a);
	static inline vec3a_t abs_normal(pref p);
	static inline vec3a_t get_normal(pref p);
};

inline plane::type vml::plane::normalize(pref p) { return vec3a::normalize(p); }

inline vec3a_t plane::vdot(pref p, pref v) {
#if VML_USE_SSE_AVX
#if VML_USE_SSE_LEVEL >= 4
	type q = _mm_and_ps(v, VML_CLEAR_W_VEC);
	q      = _mm_or_ps(q, VML_XYZ0_W1_VEC);
	return _mm_dp_ps(q, p, 0x7F);
#elif VML_USE_SSE_LEVEL >= 3
	__m128 q    = _mm_and_ps(v, VML_CLEAR_W_VEC);
	q           = _mm_or_ps(q, VML_XYZ0_W1_VEC);
	q           = _mm_mul_ps(p, q);
	__m128 shuf = _mm_movehdup_ps(q); // broadcast elements 3,1 to 2,0
	__m128 sums = _mm_add_ps(q, shuf);
	shuf        = _mm_movehl_ps(shuf, sums); // high half -> low half
	sums        = _mm_add_ss(sums, shuf);
	return sums;
#else
	type q      = _mm_and_ps(v, VML_CLEAR_W_VEC);
	q           = _mm_or_ps(q, VML_XYZ0_W1_VEC);
	q           = _mm_mul_ps(v, q);
	q           = _mm_mul_ps(p, q);
	__m128 shuf = _mm_shuffle_ps(q, q, _MM_SHUFFLE(2, 3, 0, 1)); // [ C D | A B ]
	__m128 sums = _mm_add_ps(q, shuf);       // sums = [ D+C C+D | B+A A+B ]
	shuf        = _mm_movehl_ps(shuf, sums); //  [   C   D | D+C C+D ]  // let the
	                                  //  compiler avoid a mov by reusing shuf
	sums = _mm_add_ss(sums, shuf);
	return sums;
#endif
#else
	return set(dot(vec1, vec2), 0, 0, 0);
#endif
}

inline float plane::dot(pref p, pref v) {
#if VML_USE_SSE_AVX
	return quad::x(vdot(p, q));
#else
	return set(p[0] * v[0] + p[1] * v[1] + p[2] * v[2] + p[3], 0, 0, 0);
#endif
}

inline plane::scalar_type plane::dot_with_normal(pref p, pref vec3a) {
	return vec3a::dot(p, vec3a);
}

inline vec3a_t plane::abs_normal(pref p) {
#if VML_USE_SSE_AVX
	return vec3a::abs(_mm_and_ps(p, VML_CLEAR_W_VEC));
#else
	return vec3a::abs(vec3a::set(p[0], p[1], p[2], 0));
#endif
}

inline vec3a_t plane::get_normal(pref p) {
#if VML_USE_SSE_AVX
	return _mm_and_ps(p, VML_CLEAR_W_VEC);
#else
	return vec3a::set(p[0], p[1], p[2], 0);
#endif
}

} // namespace vml
