#include "quad.hpp"

namespace vml {
struct vec3a : public quad {
	using quad::cref;
	using quad::pref;
	using quad::ref;
	using quad::row_type;
	using quad::scalar_type;
	using quad::type;

	using quad::mul;

	static inline type from_vec4(vec4_t p);
	static inline type normalize(pref p);
	static inline scalar_type dot(pref q1, pref q2);
	static inline type vdot(pref q1, pref q2);
	static inline type cross(pref q1, pref q2);
	static inline bool greater_all(pref q1, pref q2);
	static inline bool greater_any(pref q1, pref q2);
	static inline bool lesser_all(pref q1, pref q2);
	static inline bool lesser_any(pref q1, pref q2);
	static inline type mul(pref q1, mat4_t const& m);
};

inline vec3a::type vec3a::from_vec4(vec4_t p) {
#if VML_USE_SSE_AVX

	return _mm_and_ps(p, VML_CLEAR_W_VEC);
#else
	return quad::set_w(p, 0);
#endif
}

inline vec3a::type vml::vec3a::normalize(pref vec) {
#if VML_USE_SSE_AVX
#if VML_USE_SSE_LEVEL >= 4
	type q = _mm_dp_ps(vec, vec, 0x7F);
	// Get the reciprocal
	q = _mm_sqrt_ps(q);
	return _mm_div_ps(vec, q);
#elif VML_USE_SSE_LEVEL >= 3
	type q = _mm_mul_ps(vec, vec);
	q      = _mm_and_ps(q, VML_CLEAR_W_VEC);
	q      = _mm_hadd_ps(q, q); // latency 7
	q      = _mm_hadd_ps(q, q); // latency 7
	                            // Get the
	                            // reciprocal
	q = _mm_sqrt_ss(q);
	q = _mm_shuffle_ps(q, q, _MM_SHUFFLE(0, 0, 0, 0));
	return _mm_div_ps(vec, q);
#else
	// Perform the dot product
	type q = _mm_mul_ps(vec, vec);
	// x=dot[1], y=dot[2]
	type temp = _mm_shuffle_ps(q, q, _MM_SHUFFLE(2, 1, 2, 1));
	// Result[0] = x+y
	q = _mm_add_ss(q, temp);
	// x=dot[2]
	temp = _mm_shuffle_ps(temp, temp, _MM_SHUFFLE(1, 1, 1, 1));
	// Result[0] = (x+y)+z
	q = _mm_add_ss(q, temp);
	// Splat x
	q = _mm_sqrt_ss(q);
	q = _mm_shuffle_ps(q, q, _MM_SHUFFLE(0, 0, 0, 0));
	return _mm_div_ps(vec, q);
#endif
#else
	float val = vml::sqrt(dot(vec, vec));
	assert(val > vml::k_const_epsilon);
	return mul(vec, 1 / val);
#endif
}

inline vec3a::scalar_type vec3a::dot(pref q1, pref q2) {
#if VML_USE_SSE_AVX
	return x(vdot(q1, q2));
#else
	return (q1[0] * q2[0]) + (q1[1] * q2[1]) + (q1[2] * q2[2]);
#endif
}

inline vec3a::type vec3a::vdot(pref vec1, pref vec2) {
#if VML_USE_SSE_AVX
#if VML_USE_SSE_LEVEL >= 4
	return _mm_dp_ps(vec1, vec2, 0x7F);
#elif VML_USE_SSE_LEVEL >= 3
	__m128 shuf = _mm_movehdup_ps(v); // broadcast elements 3,1 to 2,0
	__m128 sums = _mm_add_ps(v, shuf);
	shuf        = _mm_movehl_ps(shuf, sums); // high half -> low half
	sums        = _mm_add_ss(sums, shuf);
	return sums;
#else
	__m128 shuf = _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 3, 0, 1)); // [ C D | A B ]
	__m128 sums = _mm_add_ps(v, shuf);       // sums = [ D+C C+D | B+A A+B ]
	shuf        = _mm_movehl_ps(shuf, sums); //  [   C   D | D+C C+D ]  // let the
	                                  //  compiler avoid a mov by reusing shuf
	sums = _mm_add_ss(sums, shuf);
	return sums;
#endif
#else
	return set(dot(vec1, vec2), 0, 0);
#endif
}

inline vec3a::type vec3a::cross(pref q1, pref q2) {
#if VML_USE_SSE_AVX
	// y1,z1,x1,w1
	type vTemp1 = _mm_shuffle_ps(vec1, vec1, _MM_SHUFFLE(3, 0, 2, 1));
	// z2,x2,y2,w2
	type vTemp2 = _mm_shuffle_ps(vec2, vec2, _MM_SHUFFLE(3, 1, 0, 2));
	// Perform the left
	// operation
	type vResult = _mm_mul_ps(vTemp1, vTemp2);
	// z1,x1,y1,w1
	vTemp1 = _mm_shuffle_ps(vTemp1, vTemp1, _MM_SHUFFLE(3, 0, 2, 1));
	// y2,z2,x2,w2
	vTemp2 = _mm_shuffle_ps(vTemp2, vTemp2, _MM_SHUFFLE(3, 1, 0, 2));
	// Perform the right
	// operation
	vTemp1 = _mm_mul_ps(vTemp1, vTemp2);
	// Subract the right from
	// left, and return answer
	vResult = _mm_sub_ps(vResult, vTemp1);
	// set w to zero
	return _mm_and_ps(vResult, VML_CLEAR_W_VEC);
#else
	return set(vec1[1] * vec2[2] - vec1[2] * vec2[1],
	           vec1[2] * vec2[0] - vec1[0] * vec2[2],
	           vec1[0] * vec2[1] - vec1[1] * vec2[0]);

#endif
}

inline bool vec3a::greater_all(pref q1, pref q2) {
#if VML_USE_SSE_AVX
	return ((_mm_movemask_ps(_mm_cmpgt_ps(q1, q2)) & 0x7) == 0x7);
#else
	return (q1[0] > q2[0] && q1[1] > q2[1] && q1[2] > q2[2]) != 0;
#endif
}

inline bool vec3a::greater_any(pref q1, pref q2) {
#if VML_USE_SSE_AVX
	return ((_mm_movemask_ps(_mm_cmpgt_ps(q1, q2))) & 0x7) != 0;
#else
	return (q1[0] > q2[0] || q1[1] > q2[1] || q1[2] > q2[2]) != 0;
#endif
}

inline bool vec3a::lesser_all(pref q1, pref q2) { return greater_any(q2, q1); }

inline bool vec3a::lesser_any(pref q1, pref q2) { return greater_all(q2, q1); }

inline vec3a::tye vec3a::mul(pref q1, mat4_t const& m) {
#if VML_USE_SSE_AVX
	type ret;
	ret        = _mm_shuffle_ps(v, v, _MM_SHUFFLE(0, 0, 0, 0));
	ret        = _mm_mul_ps(ret, m[0]);
	type vTemp = _mm_shuffle_ps(v, v, _MM_SHUFFLE(1, 1, 1, 1));
	vTemp      = _mm_mul_ps(vTemp, m[1]);
	ret        = _mm_add_ps(ret, vTemp);
	vTemp      = _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 2, 2, 2));
	vTemp      = _mm_mul_ps(vTemp, m[2]);
	ret        = _mm_add_ps(ret, vTemp);
	ret        = _mm_add_ps(ret, m[3]);
	return ret;
#else
	type r, x, y, z;

	z = splat_z(v);
	y = splat_y(v);
	x = splat_x(v);

	r = madd(z, m[2], m[3]);
	r = madd(y, m[1], r);
	r = madd(x, m[0], r);

	return r;
#endif
}
} // namespace vml