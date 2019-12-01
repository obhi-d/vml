
#include "quad.hpp"

namespace vml {

struct quat : public quad {
	using quad::scalar_type;
	using quad::cref;
	using quad::pref;
	using quad::ref;
	using quad::row_type;
	using quad::scalar_type;
	using quad::type;

	static inline type identity();
	static inline type from_axis_angle(vec3_t const& v, scalar_type ang);
	static inline type from_axis_angle(axis_angle_t const& v);
	static inline type from_mat4(mat4_t const& m);
	static inline type from_mat3(mat3_t const& m);
	static inline type mul(pref q1, pref q2);
	static inline vec3a_t transform(pref q1, vec3a_t const& q2);
	static inline vec3a_t transform_bounds(pref q1, vec3a_t const& extends);
	static inline type slerp(pref q1, pref q2, scalar_type t);
	static inline type inverse(pref q);
};

inline quat::type quat::identity() { return set(0, 0, 0, 1); }
inline type quat::from_axis_angle(vec3_t const& v, scalar_type ang) {
#if L_CORE_DEBUG_CHECKS == 1 float len = Vec3Op::length(axis);
	L_ASSERT(L_FLOAT_TOLERANCE_EQUAL(len, 1, Math::kEpsilonMed));
#endif
	float f, c;
	Math::SinCos(ang * .5f, f, c);
	return set(f * axis[0], f * axis[1], f * axis[2], c);
}
inline type quat::from_axis_angle(axis_angle_t const& v) {
#if VML_USE_SSE_AVX
#if L_CORE_DEBUG_CHECKS == 1
	float len = 1; //\todo Find length of axis
	L_ASSERT(L_FLOAT_TOLERANCE_EQUAL(len, 1, Math::kEpsilonMed));
#endif
	V_Quad N = _mm_and_ps(axis, N3D_FFFO.v);
	N        = _mm_or_ps(N, N3D_0001.v);

	float f, c;
	Math::SinCos(QuatOp::GetW(axis) * .5f, f, c);
	V_Quad vSineCosine = QuadOp::set(f, f, f, c);
	return _mm_mul_ps(N, vSineCosine);
#else
	float s, c;
	Math::SinCos(axis[3] * .5f, s, c);
	return set(s * axis[0], s * axis[1], s * axis[2], c);
#endif
}
inline type quat::from_mat4(mat4_t const& m) {
	return from_mat3(*reinterpret_cast<const mat3_t*>(&m));
}
inline type quat::from_mat3(mat3_t const& m) {
	int maxi;
	float maxdiag, trace;
	trace = m.m00 + m.m11 + m.m22 + 1.0f;
	if (trace > 0.0f) {
		return set((m.m12 - m.m21) / (2.0f * sqrt(trace)),
		           (m.m20 - m.m02) / (2.0f * sqrt(trace)),
		           (m.m01 - m.m10) / (2.0f * sqrt(trace)), sqrt(trace) / 2.0f);
	}
	maxi    = 0;
	maxdiag = m.m00;

	if (m.m11 > maxdiag) {
		maxdiag = m.m11;
		maxi    = 1;
	}

	if (m.m22 > maxdiag) {
		maxdiag = m.m22;
		maxi    = 2;
	}

	float s, invS;
	switch (maxi) {
	case 0:
		s    = 2.0f * sqrt(1.0f + m.m00 - m.m11 - m.m22);
		invS = 1 / s;
		return set(0.25f * s, (m.m01 + m.m10) * invS, (m.m02 + m.m20) * invS,
		           (m.m12 - m.m21) * invS);

	case 1:
		s    = 2.0f * sqrt(1.0f + m.m11 - m.m00 - m.m22);
		invS = 1 / s;
		return set((m.m01 + m.m10) * invS, 0.25f * s, (m.m12 + m.m21) * invS,
		           (m.m20 - m.m02) * invS);
	case 2:
	default:
		s    = 2.0f * sqrt(1.0f + m.m22 - m.m00 - m.m11);
		invS = 1 / s;
		return set((m.m02 + m.m20) * invS, (m.m12 + m.m21) * invS, 0.25f * s,
		           (m.m01 - m.m10) * invS);
	}
}
inline type quat::mul(pref q1, pref q2) {
#if VML_USE_SSE_AVX
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
	Quad Q1Shuffle = q1;
	// Shuffle the copies of q1
	Q1Shuffle = _mm_shuffle_ps(Q1Shuffle, Q1Shuffle, _MM_SHUFFLE(0, 1, 2, 3));
	// mul by Q1WZYX
	Q2X       = _mm_mul_ps(Q2X, Q1Shuffle);
	Q1Shuffle = _mm_shuffle_ps(Q1Shuffle, Q1Shuffle, _MM_SHUFFLE(2, 3, 0, 1));
	// Flip the signs on y and z
	Q2X = _mm_xor_ps(Q2X, (N3D_OXOX.v));
	// mul by Q1ZWXY
	Q2Y       = _mm_mul_ps(Q2Y, Q1Shuffle);
	Q1Shuffle = _mm_shuffle_ps(Q1Shuffle, Q1Shuffle, _MM_SHUFFLE(0, 1, 2, 3));
	// Flip the signs on z and w
	Q2Y = _mm_xor_ps(Q2Y, (N3D_OOXX.v));
	// mul by Q1YXWZ
	Q2Z     = _mm_mul_ps(Q2Z, Q1Shuffle);
	vResult = _mm_add_ps(vResult, Q2X);
	// Flip the signs on x and w
	Q2Z     = _mm_xor_ps(Q2Z, (N3D_XOOX.v));
	Q2Y     = _mm_add_ps(Q2Y, Q2Z);
	vResult = _mm_add_ps(vResult, Q2Y);
	return vResult;
#else
	return set((q2[3] * q1[0]) + (q2[0] * q1[3]) + (q2[1] * q1[2]) - (q2[2] * q1[1]),
	           (q2[3] * q1[1]) - (q2[0] * q1[2]) + (q2[1] * q1[3]) + (q2[2] * q1[0]),
	           (q2[3] * q1[2]) + (q2[0] * q1[1]) - (q2[1] * q1[0]) + (q2[2] * q1[3]),
	           (q2[3] * q1[3]) - (q2[0] * q1[0]) - (q2[1] * q1[1]) - (q2[2] * q1[2]));
#endif
}
inline vec3a_t quat::transform(pref q1, vec3a_t const& q2) {
	vec3a_t uv  = vec3a::Cross(q, v);
	vec3a_t uuv = vec3a::Cross(q, uv);
	return vec3a::add(vec3a::add(v, QuadOp::mul(uv, 2 * QuatOp::GetW(q))),
	                    vec3a::add(uuv, uuv));
}
inline vec3a_t quat::transform_bounds(pref q1, vec3a_t const& extends) {
	Vector3A uv  = vec3a::Cross(q, v);
	Vector3A uuv = vec3a::Cross(q, uv);
	return vec3a::add(
	    vec3a::add(v, vec3a::Abs(QuadOp::mul(uv, 2 * QuatOp::GetW(q)))),
	    vec3a::Abs(vec3a::add(uuv, uuv)));
}
inline type quat::slerp(pref q1, pref q2, scalar_type t) {
	float cosom, absCosom, sinom, omega, scale0, scale1;
	cosom    = Vec4Op::dot(from, to);
	absCosom = Math::Abs(cosom);
	if ((1.0f - absCosom) > Math::kEpsilon) {
		omega  = Math::ArcCos(absCosom);
		sinom  = 1.0f / Math::Sin(omega);
		scale0 = Math::Sin((1.0f - t) * omega) * sinom;
		scale1 = Math::Sin(t * omega) * sinom;
	} else {
		scale0 = 1.0f - t;
		scale1 = t;
	}

	scale1 = (cosom >= 0.0f) ? scale1 : -scale1;
	return QuadOp::add(QuadOp::mul(from, scale0), QuadOp::mul(to, scale1));
}
inline type quat::lerp(pref q1, pref q2, scalar_type t) { return type(); }
inline type quat::inverse(pref q) {
#if VML_USE_SSE_AVX 
	return _mm_xor_ps(q, N3D_OXXX.v);
#else
	return set(-q[0], -q[1], -q[2], q[3]);
}
} // namespace vml