#pragma once
#include "multi_dim.hpp"
namespace vml {
template <typename concrete> struct mat_base : public multi_dim<concrete> {
	// @brief Create a matrix from vector mapping that
	// can rotate the vector axis1 to axis2 when post multiplied to axis1.
	static inline type FromVectorMapping(TraitsVec3::pref v1,
	                                     TraitsVec3::pref v2);
	// @brief rotate vector in place
	static inline void Rotate(pref m, TraitsVec3::type* ioStream,
	                          std::uint32_t inStride, std::uint32_t count);
	// @brief rotate vector
	static inline vec3a_t Rotate(pref m, vec3a::pref v);
	// @brief set a rotation for a given matrix
	static inline void SetRot(ref m, TraitsQuat::pref rot);
	// @brief Returns a look at matrix based on a look at vector and up position
	static inline void SetViewAndUp(ref m, vec3a::pref viewDir, vec3a::pref up);
};

template <typename concrete>
inline typename mat_base<concrete>::type mat_base<concrete>::FromVectorMapping(
    TraitsVec3::pref axis1, TraitsVec3::pref axis2) {
	/** \todo sse **/
	type m;
	scalar_type cs, xy1C, xz1C, yz1C;
	cs = Vec3Op::dot(axis1, axis2);
	scalar_type _1_c;
	Vector3 axis = Vec3Op::Cross(axis1, axis2);
	// OPTIMIZE: we can also check the angle to
	// see if its a multiple of Pi.
	if (Math::Abs(axis.x) < Math::kEpsilonMed &&
	    Math::Abs(axis.y) < Math::kEpsilonMed &&
	    Math::Abs(axis.z) < Math::kEpsilonMed) {
		// take a cross for that
		axis = Vec3Op::Cross(axis1, Vector3::kYAxis);
		if (Math::Abs(axis.x) < Math::kEpsilonMed &&
		    Math::Abs(axis.y) < Math::kEpsilonMed &&
		    Math::Abs(axis.z) < Math::kEpsilonMed) {
			axis = Vec3Op::Cross(axis1, Vector3::kXAxis);
		}
	}
	_1_c         = 1.0f - cs;
	Vector3 xyzs = Vec3Op::Mul(axis, -Math::Sqrt(1 - cs * cs));
	Vector3 mstr = Vec3Op::Mul(axis, axis);
	mstr         = mstr * _1_c;
	xy1C         = axis.x * axis.y * _1_c;
	xz1C         = axis.x * axis.z * _1_c;
	yz1C         = axis.y * axis.z * _1_c;

	m.r[0] = row_op::Set(cs + mstr.x, xy1C - xyzs.z, xz1C + xyzs.y, 0);
	m.r[1] = row_op::Vec4ASet(xy1C + xyzs.z, cs + mstr.y, yz1C - xyzs.x, 0);
	m.r[2] = row_op::Set(xz1C - xyzs.y, yz1C + xyzs.x, cs + mstr.z, 0);

	return m;
}

template <typename concrete>
inline void mat_base<concrete>::Rotate(pref m, TraitsVec3::type* iStream,
                                       uint32 inStride, uint32 count) {
	L_ASSERT(iStream);
	const uint8* inpVec = (const uint8*)iStream;
#if L_VECTOR_MATH_TYPE_IS_SSE
	uint8* outVec = (uint8*)iStream;
	union {
		V_Quad v;
		scalar_type s[4];
	} store;

	for (uint32 i = 0; i < count; i++) {
		V_Quad x   = _mm_load_ps1(&reinterpret_cast<const Vector3*>(inpVec)->x);
		V_Quad y   = _mm_load_ps1(&reinterpret_cast<const Vector3*>(inpVec)->y);
		V_Quad res = _mm_load_ps1(&reinterpret_cast<const Vector3*>(inpVec)->z);
		res        = _mm_mul_ps(res, m.r[2]);
		y          = _mm_mul_ps(y, m.r[1]);
		res        = _mm_add_ps(res, y);
		x          = _mm_mul_ps(x, m.r[0]);
		res        = _mm_add_ps(res, x);
		res        = Vec3AOp::Normalize(res);
		_mm_store_ps(store.s, res);
		((scalar_type*)inpVec)[0] = store.s[0];
		((scalar_type*)inpVec)[1] = store.s[1];
		((scalar_type*)inpVec)[2] = store.s[2];

		inpVec += inStride;
	}
#else
	V_Quad x, y, z, r;
	for (uint32 i = 0; i < count; i++) {
		x = QuadOp::Set(((scalar_type*)inpVec)[0]);
		y = QuadOp::Set(((scalar_type*)inpVec)[1]);
		z = QuadOp::Set(((scalar_type*)inpVec)[2]);

		r = Vec3AOp::Mul(z, Row(m, 2));
		r = Vec3AOp::Madd(y, Row(m, 1), r);
		r = Vec3AOp::Normalize(Vec3AOp::Madd(x, Row(m, 0), r));

		((scalar_type*)inpVec)[0] = r.x;
		((scalar_type*)inpVec)[1] = r.y;
		((scalar_type*)inpVec)[2] = r.z;

		inpVec += inStride;
	}
#endif
}

template <typename concrete>
inline TraitsVec3A::type mat_base<concrete>::Rotate(pref m,
                                                    TraitsVec3A::pref v) {
#if L_VECTOR_MATH_TYPE_IS_SSE
	V_Quad vRes  = _mm_shuffle_ps(v, v, _MM_SHUFFLE(0, 0, 0, 0));
	vRes         = _mm_mul_ps(vRes, m.r[0]);
	V_Quad vTemp = _mm_shuffle_ps(v, v, _MM_SHUFFLE(1, 1, 1, 1));
	vTemp        = _mm_mul_ps(vTemp, m.r[1]);
	vRes         = _mm_add_ps(vRes, vTemp);
	vTemp        = _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 2, 2, 2));
	vTemp        = _mm_mul_ps(vTemp, m.r[2]);
	vRes         = _mm_add_ps(vRes, vTemp);
	return vRes;
#else
	V_Quad r = Vec3AOp::Mul(Vec3AOp::SplatZ(v), Row(m, 2));
	r        = Vec3AOp::Madd(Vec3AOp::SplatY(v), Row(m, 1), r);
	r        = Vec3AOp::Madd(Vec3AOp::SplatX(v), Row(m, 0), r);
	return r;
#endif
}

template <typename concrete>
inline void mat_base<concrete>::SetRot(ref ret, TraitsQuat::pref rot) {
#if L_VECTOR_MATH_TYPE_IS_SSE
	V_Quad r0, r1, r2;
	V_Quad q0, q1;
	V_Quad v0, v1, v2;

	q0 = _mm_add_ps(rot, rot);
	q1 = _mm_mul_ps(rot, q0);

	v0 = _mm_shuffle_ps(q1, q1, _MM_SHUFFLE(3, 0, 0, 1));
	v0 = _mm_and_ps(v0, N3D_FFFO.v);
	v1 = _mm_shuffle_ps(q1, q1, _MM_SHUFFLE(3, 1, 2, 2));
	v1 = _mm_and_ps(v1, N3D_FFFO.v);
	r0 = _mm_sub_ps(N3D_1110.v, v0);
	r0 = _mm_sub_ps(r0, v1);

	v0 = _mm_shuffle_ps(rot, rot, _MM_SHUFFLE(3, 1, 0, 0));
	v1 = _mm_shuffle_ps(q0, q0, _MM_SHUFFLE(3, 2, 1, 2));
	v0 = _mm_mul_ps(v0, v1);

	v1 = _mm_shuffle_ps(rot, rot, _MM_SHUFFLE(3, 3, 3, 3));
	v2 = _mm_shuffle_ps(q0, q0, _MM_SHUFFLE(3, 0, 2, 1));
	v1 = _mm_mul_ps(v1, v2);

	r1 = _mm_add_ps(v0, v1);
	r2 = _mm_sub_ps(v0, v1);

	v0 = _mm_shuffle_ps(r1, r2, _MM_SHUFFLE(1, 0, 2, 1));
	v0 = _mm_shuffle_ps(v0, v0, _MM_SHUFFLE(1, 3, 2, 0));
	v1 = _mm_shuffle_ps(r1, r2, _MM_SHUFFLE(2, 2, 0, 0));
	v1 = _mm_shuffle_ps(v1, v1, _MM_SHUFFLE(2, 0, 2, 0));

	q1       = _mm_shuffle_ps(r0, v0, _MM_SHUFFLE(1, 0, 3, 0));
	q1       = _mm_shuffle_ps(q1, q1, _MM_SHUFFLE(1, 3, 2, 0));
	ret.r[0] = q1;
	q1       = _mm_shuffle_ps(r0, v0, _MM_SHUFFLE(3, 2, 3, 1));
	q1       = _mm_shuffle_ps(q1, q1, _MM_SHUFFLE(1, 3, 0, 2));
	ret.r[1] = q1;
	q1       = _mm_shuffle_ps(v1, r0, _MM_SHUFFLE(3, 2, 1, 0));
	ret.r[2] = q1;
#else
	scalar_type xx = rot.x * rot.x;
	scalar_type yy = rot.y * rot.y;
	scalar_type zz = rot.z * rot.z;
	scalar_type xy = rot.x * rot.y;
	scalar_type xz = rot.x * rot.z;
	scalar_type yz = rot.y * rot.z;
	scalar_type wx = rot.w * rot.x;
	scalar_type wy = rot.w * rot.y;
	scalar_type wz = rot.w * rot.z;

	ret.m00 = (1 - 2 * (yy + zz));
	ret.m01 = (2 * (xy + wz));
	ret.m02 = (2 * (xz - wy));
	ret.m03 = 0;

	ret.m10 = (2 * (xy - wz));
	ret.m11 = (1 - 2 * (xx + zz));
	ret.m12 = (2 * (yz + wx));
	ret.m13 = 0;

	ret.m20 = (2 * (xz + wy));
	ret.m21 = (2 * (yz - wx));
	ret.m22 = (1 - 2 * (xx + yy));
	ret.m23 = 0;
#endif
}

template <typename concrete>
inline void mat_base<concrete>::SetViewAndUp(ref ret, TraitsVec3A::pref viewDir,
                                             TraitsVec3A::pref upDir) {
	// TODO needs validation
	Vector3A up;
	ret.r[2] = Vec3AOp::Normalize(viewDir);
	ret.r[0] = Vec3AOp::Normalize(Vec3AOp::Cross(viewDir, upDir));
	ret.r[1] = Vec3AOp::Cross(ret.r[0], ret.r[2]);
}
} // namespace vml