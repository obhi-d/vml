
#pragma once
#include "axis_angle.hpp"
#include "quad.hpp"
#include "vml_fcn.hpp"

namespace vml {

struct quat : public quad {
	using quad::cref;
	using quad::pref;
	using quad::ref;
	using quad::row_type;
	using quad::scalar_type;
	using quad::type;

	using quad::normalize;

	static inline type identity();
	static inline type from_axis_angle(vec3_t const& v, scalar_type ang);
	static inline type from_axis_angle(axis_angle_t const& v);
	static inline type from_mat4(mat4_t const& m);
	static inline type from_mat3(mat3_t const& m);
	static inline type mul(pref q1, pref q2);
	static inline vec3a_t transform(pref q1, vec3a_t const& q2);
	static inline vec3a_t transform_bounds(pref q1, vec3a_t const& extends);
	static inline type slerp(pref q1, pref q2, scalar_type t);
	static inline type lerp(pref q1, pref q2, scalar_type t);
	static inline type inverse(pref q);
};

inline quat::type quat::identity() { return set(0.0f, 0.0f, 0.0f, 1.0f); }
inline quat::type quat::from_axis_angle(vec3_t const& axis, scalar_type ang) {
#ifndef NDEBUG
	float len = vec3::length(axis);
	assert(VML_FLOAT_TOLERANCE_EQUAL(len, 1, vml::k_const_epsilon_med));
#endif
	auto sc = vml::sin_cos(ang * .5f);
	return set(sc.first * axis[0], sc.first * axis[1], sc.first * axis[2],
	           sc.second);
}
inline quat::type quat::from_axis_angle(axis_angle_t const& axis) {
#if VML_USE_SSE_AVX
#ifndef NDEBUG
	float len = 1; //\todo Find length of axis
	assert(VML_FLOAT_TOLERANCE_EQUAL(len, 1, vml::k_const_epsilon_med));
#endif
	quad_t N = axis_angle::get_axis(axis);
	N        = _mm_or_ps(N, _mm_set_ps(0.0f, 0.0f, 0.0f, 1.0f));

	auto sc            = vml::sin_cos(quad::w(axis) * .5f);
	quad_t vSineCosine = quad::set(sc.first, sc.first, sc.first, sc.second);
	return _mm_mul_ps(N, vSineCosine);
#else
	auto sc = vml::sin_cos(quad::w(axis) * .5f);
	return set(sc.first * axis[0], sc.first * axis[1], sc.first * axis[2],
	           sc.second);
#endif
}
inline quat::type quat::from_mat4(mat4_t const& m) {
	return from_mat3(*reinterpret_cast<const mat3_t*>(&m));
}
inline quat::type quat::from_mat3(mat3_t const& m) {
	int maxi;
	float maxdiag, trace;
	trace = m.e[0][0] + m.e[1][1] + m.e[2][2] + 1.0f;
	if (trace > 0.0f) {
		return set((m.e[1][2] - m.e[2][1]) / (2.0f * sqrt(trace)),
		           (m.e[2][0] - m.e[0][2]) / (2.0f * sqrt(trace)),
		           (m.e[0][1] - m.e[1][0]) / (2.0f * sqrt(trace)),
		           sqrt(trace) / 2.0f);
	}
	maxi    = 0;
	maxdiag = m.e[0][0];

	if (m.e[1][1] > maxdiag) {
		maxdiag = m.e[1][1];
		maxi    = 1;
	}

	if (m.e[2][2] > maxdiag) {
		maxdiag = m.e[2][2];
		maxi    = 2;
	}

	float s, invS;
	switch (maxi) {
	case 0:
		s    = 2.0f * sqrt(1.0f + m.e[0][0] - m.e[1][1] - m.e[2][2]);
		invS = 1 / s;
		return set(0.25f * s, (m.e[0][1] + m.e[1][0]) * invS,
		           (m.e[0][2] + m.e[2][0]) * invS, (m.e[1][2] - m.e[2][1]) * invS);

	case 1:
		s    = 2.0f * sqrt(1.0f + m.e[1][1] - m.e[0][0] - m.e[2][2]);
		invS = 1 / s;
		return set((m.e[0][1] + m.e[1][0]) * invS, 0.25f * s,
		           (m.e[1][2] + m.e[2][1]) * invS, (m.e[2][0] - m.e[0][2]) * invS);
	case 2:
	default:
		s    = 2.0f * sqrt(1.0f + m.e[2][2] - m.e[0][0] - m.e[1][1]);
		invS = 1 / s;
		return set((m.e[0][2] + m.e[2][0]) * invS, (m.e[1][2] + m.e[2][1]) * invS,
		           0.25f * s, (m.e[0][1] - m.e[1][0]) * invS);
	}
}
inline quat::type quat::mul(pref q1, pref q2) {
#if VML_USE_SSE_AVX
#if VML_USE_SSE_LEVEL >= 3
#define _mm_pshufd(r, i) vml_cast_i_to_v(_mm_shuffle_epi32(vml_cast_v_to_i(r), i))
	// @link
	// http://momchil-velikov.blogspot.com/2013/10/fast-sse-quternion-multiplication.html
	// Copy to SSE registers and use as few as possible for x86
	__m128 t0 = _mm_pshufd(q1, _MM_SHUFFLE(3, 3, 3, 3)); /* 1, 0.5 */
	__m128 t1 = _mm_pshufd(q2, _MM_SHUFFLE(2, 3, 0, 1)); /* 1, 0.5 */

	__m128 t3 = _mm_pshufd(q1, _MM_SHUFFLE(0, 0, 0, 0)); /* 1, 0.5 */
	__m128 t4 = _mm_pshufd(q2, _MM_SHUFFLE(1, 0, 3, 2)); /* 1, 0.5 */

	__m128 t5 = _mm_pshufd(q1, _MM_SHUFFLE(1, 1, 1, 1)); /* 1, 0.5 */
	__m128 t6 = _mm_pshufd(q2, _MM_SHUFFLE(2, 0, 3, 1)); /* 1, 0.5 */

	/* [d,d,d,d]*[z,w,x,y] = [dz,dw,dx,dy] */
	__m128 m0 = _mm_mul_ps(t0, t1); /* 5/4, 1 */

	/* [a,a,a,a]*[y,x,w,z] = [ay,ax,aw,az]*/
	__m128 m1 = _mm_mul_ps(t3, t4); /* 5/4, 1 */

	/* [b,b,b,b]*[z,x,w,y] = [bz,bx,bw,by]*/
	__m128 m2 = _mm_mul_ps(t5, t6); /* 5/4, 1 */

	/* [c,c,c,c]*[w,z,x,y] = [cw,cz,cx,cy] */
	__m128 t7 = _mm_pshufd(q1, _MM_SHUFFLE(2, 2, 2, 2)); /* 1, 0.5 */
	__m128 t8 = _mm_pshufd(q2, _MM_SHUFFLE(3, 2, 0, 1)); /* 1, 0.5 */

	__m128 m3 = _mm_mul_ps(t7, t8); /* 5/4, 1 */

	/* 1 */
	/* [dz,dw,dx,dy]+-[ay,ax,aw,az] = [dz+ay,dw-ax,dx+aw,dy-az] */
	__m128 e = _mm_addsub_ps(m0, m1); /* 3, 1 */

	/* 2 */
	/* [dx+aw,dz+ay,dy-az,dw-ax] */
	e = _mm_pshufd(e, _MM_SHUFFLE(1, 3, 0, 2)); /* 1, 0.5 */

	/* [dx+aw,dz+ay,dy-az,dw-ax]+-[bz,bx,bw,by] =
	 * [dx+aw+bz,dz+ay-bx,dy-az+bw,dw-ax-by]*/
	e = _mm_addsub_ps(e, m2); /* 3, 1 */

	/* 2 */
	/* [dz+ay-bx,dw-ax-by,dy-az+bw,dx+aw+bz] */
	e = _mm_pshufd(e, _MM_SHUFFLE(2, 0, 1, 3)); /* 1, 0.5 */

	/* [dz+ay-bx,dw-ax-by,dy-az+bw,dx+aw+bz]+-[cw,cz,cx,cy]
	   = [dz+ay-bx+cw,dw-ax-by-cz,dy-az+bw+cx,dx+aw+bz-cy] */
	e = _mm_addsub_ps(e, m3); /* 3, 1 */

	/* 2 */
	/* [dw-ax-by-cz,dz+ay-bx+cw,dy-az+bw+cx,dx+aw+bz-cy] */
	e = _mm_pshufd(e, _MM_SHUFFLE(2, 3, 1, 0)); /* 1, 0.5 */
	return e;
#else
	// Copy to SSE registers and use as few as possible for x86
	type Q2X     = q2;
	type Q2Y     = q2;
	type Q2Z     = q2;
	type vResult = q2;
	// Splat with one instruction
	vResult = _mm_shuffle_ps(vResult, vResult, _MM_SHUFFLE(3, 3, 3, 3));
	Q2X     = _mm_shuffle_ps(Q2X, Q2X, _MM_SHUFFLE(0, 0, 0, 0));
	Q2Y     = _mm_shuffle_ps(Q2Y, Q2Y, _MM_SHUFFLE(1, 1, 1, 1));
	Q2Z     = _mm_shuffle_ps(Q2Z, Q2Z, _MM_SHUFFLE(2, 2, 2, 2));
	// Retire q1 and perform q1*Q2W
	vResult        = _mm_mul_ps(vResult, q1);
	type Q1Shuffle = q1;
	// Shuffle the copies of q1
	Q1Shuffle = _mm_shuffle_ps(Q1Shuffle, Q1Shuffle, _MM_SHUFFLE(0, 1, 2, 3));
	// mul by Q1WZYX
	Q2X       = _mm_mul_ps(Q2X, Q1Shuffle);
	Q1Shuffle = _mm_shuffle_ps(Q1Shuffle, Q1Shuffle, _MM_SHUFFLE(2, 3, 0, 1));
	// Flip the signs on y and z
	Q2X = _mm_xor_ps(Q2X, vml_cast_i_to_v(_mm_set_epi32(0x80000000, 0x00000000,
	                                                    0x80000000, 0x00000000)));
	// mul by Q1ZWXY
	Q2Y       = _mm_mul_ps(Q2Y, Q1Shuffle);
	Q1Shuffle = _mm_shuffle_ps(Q1Shuffle, Q1Shuffle, _MM_SHUFFLE(0, 1, 2, 3));
	// Flip the signs on z and w
	Q2Y = _mm_xor_ps(Q2Y, (vml_cast_i_to_v(_mm_set_epi32(
	                          0x80000000, 0x80000000, 0x00000000, 0x00000000))));
	// mul by Q1YXWZ
	Q2Z     = _mm_mul_ps(Q2Z, Q1Shuffle);
	vResult = _mm_add_ps(vResult, Q2X);
	// Flip the signs on x and w
	Q2Z     = _mm_xor_ps(Q2Z, (vml_cast_i_to_v(_mm_set_epi32(
                            0x80000000, 0x00000000, 0x00000000, 0x80000000))));
	Q2Y     = _mm_add_ps(Q2Y, Q2Z);
	vResult = _mm_add_ps(vResult, Q2Y);
	return vResult;
#endif
#else
	return set(
	    (q2[3] * q1[0]) + (q2[0] * q1[3]) + (q2[1] * q1[2]) - (q2[2] * q1[1]),
	    (q2[3] * q1[1]) - (q2[0] * q1[2]) + (q2[1] * q1[3]) + (q2[2] * q1[0]),
	    (q2[3] * q1[2]) + (q2[0] * q1[1]) - (q2[1] * q1[0]) + (q2[2] * q1[3]),
	    (q2[3] * q1[3]) - (q2[0] * q1[0]) - (q2[1] * q1[1]) - (q2[2] * q1[2]));
#endif
}
inline vec3a_t quat::transform(pref q, vec3a_t const& v) {
	vec3a_t uv  = vec3a::cross(q, v);
	vec3a_t uuv = vec3a::cross(q, uv);
	return vec3a::add(vec3a::add(v, quad::mul(uv, 2 * quad::w(q))),
	                  vec3a::add(uuv, uuv));
}
inline vec3a_t quat::transform_bounds(pref q, vec3a_t const& v) {
	vec3a_t uv  = vec3a::cross(q, v);
	vec3a_t uuv = vec3a::cross(q, uv);
	return vec3a::add(vec3a::add(v, vec3a::abs(quad::mul(uv, 2 * quad::w(q)))),
	                  vec3a::abs(vec3a::add(uuv, uuv)));
}
inline quat::type quat::slerp(pref from, pref to, scalar_type t) {
	float cosom, abs_cosom, sinom, omega, scale0, scale1;
	cosom     = vec4::dot(from, to);
	abs_cosom = vml::abs(cosom);
	if ((1.0f - abs_cosom) > vml::k_const_epsilon) {
		omega  = vml::arc_cos(abs_cosom);
		sinom  = 1.0f / vml::sin(omega);
		scale0 = vml::sin((1.0f - t) * omega) * sinom;
		scale1 = vml::sin(t * omega) * sinom;
	} else {
		scale0 = 1.0f - t;
		scale1 = t;
	}

	scale1 = (cosom >= 0.0f) ? scale1 : -scale1;
	return quad::add(quad::mul(from, scale0), quad::mul(to, scale1));
}
inline quat::type quat::inverse(pref q) {
#if VML_USE_SSE_AVX
	return _mm_xor_ps(q, (vml_cast_i_to_v(_mm_set_epi32(
	                         0x00000000, 0x80000000, 0x80000000, 0x80000000))));
#else
	return set(-q[0], -q[1], -q[2], q[3]);
#endif
}
inline quat::type quat::lerp(pref q1, pref q2, scalar_type t) {
	return quad::normalize(quad::lerp(q1, q2, t));
}
} // namespace vml