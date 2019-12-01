
#include "vec3a.hpp"

namespace vml {

struct plane : public quad {
	using quad::scalar_type;
	using quad::cref;
	using quad::pref;
	using quad::ref;
	using quad::row_type;
	using quad::scalar_type;
	using quad::type;

	static inline type normalize(pref p);
	static inline scalar_type dot(pref p, pref vec3a);
	static inline scalar_type dot_with_normal(pref p, pref vec3a);
	static inline vec3a_t abs_normal(pref p);
	static inline vec3a_t get_normal(perf p);
};

inline plane::type vml::plane::normalize(pref p) { return vec3a::normalize(p); }

inline plane::scalar_type plane::dot(pref p, pref v) {
#if VML_USE_SSE_AVX
#if VML_USE_SSE_LEVEL >= 4
	type q = _mm_and_ps(v, VML_CLEAR_W_VEC);
	q      = _mm_or_ps(q, VML_XYZ0_W1_VEC);
	q      = _mm_dp_ps(q, p, 0xFF);
#elif VML_USE_SSE_LEVEL >= 3
	type q = _mm_and_ps(v, VML_CLEAR_W_VEC);
	q      = _mm_or_ps(q, VML_XYZ0_W1_VEC);
	q      = _mm_mul_ps(q, p);
	q      = _mm_hadd_ps(q, q); // latency 7
	q      = _mm_hadd_ps(q, q); // latency 7
#else
	type q    = _mm_and_ps(v, VML_CLEAR_W_VEC);
	q         = _mm_or_ps(q, VML_XYZ0_W1_VEC);
	q         = _mm_mul_ps(q, p);
	type temp = _mm_shuffle_ps(q, q, _MM_SHUFFLE(3, 2, 3, 2));
	q         = _mm_add_ps(q, temp);
	// x+z,x+z,x+z,y+w
	q = _mm_shuffle_ps(q, q, _MM_SHUFFLE(1, 0, 0, 0));
	// y+w, ??, ??, ??
	temp = _mm_shuffle_ps(q, temp, _MM_SHUFFLE(0, 0, 0, 3));
	// x+z+y+w,??, ??, ??
	q = _mm_add_ss(q, temp);
#endif
	return quad::getx(q);
#else
	return p.x * v.x + p.y * v.y + p.z * v.z + p.w;
#endif
}

inline plane::scalar_type plane::dot_with_normal(pref p, pref vec3a) {
	return vec3a::dot(p, vec3a);
}

inline vec3a_t plane::abs_normal(pref p) {
	return vec3a::abs(_mm_and_ps(p, VML_CLEAR_W_VEC));
}

inline vec3a_t plane::get_normal(perf p) {
	return _mm_and_ps(p, VML_CLEAR_W_VEC);
}

} // namespace vml
