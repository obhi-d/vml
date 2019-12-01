#pragma once
#include "quad.hpp"

namespace vml {

namespace detail {
} // namespace detail
struct eular_angles : public quad {
	using quad::scalar_type;
	using quad::cref;
	using quad::pref;
	using quad::ref;
	using quad::row_type;
	using quad::scalar_type;
	using quad::type;

	static inline type canonize(pref m);
	static inline type from_quat(quat_t const& m);
	static inline type from_inv_quat(quat_t const& m);
	static inline type from_mat4(mat4_t const& m);
	static inline type from_mat3(mat3_t const& m);
};
inline eular_angles::type eular_angles::canonize(pref m) { 	// First, wrap r.x in range -pi ... pi
	type r;
	r.x = Math::WrapPi(m.x);
	r.y = m.y;
	r.z = m.z;
	// Now, check for "the back side" of the matrix r.x outside
	// the canonical range of -pi/2 ... pi/2
	if (r.x < -Math::kPiBy2) {
		r.x = -Math::kPiBy2 - r.x;
		r.y += Math::kPi;
		r.z += Math::kPi;
	} else if (r.x > Math::kPiBy2) {
		r.x = Math::kPi - r.x;
		r.y += Math::kPi;
		r.z += Math::kPi;
	}
	// Now check for the gimbel Lock case (within a slight tolerance)
	if (Math::Abs(r.x) > Math::kPiBy2 - 1e-4) {
		// We are in gimbel Lock. Assign all rotation
		// about the vertical axis to r.y
		r.y += r.z;
		r.z = 0.0f;
	} else {
		// Not in gimbel Lock. Wrap the r.z angle in
		// canonical range
		r.z = Math::WrapPi(r.z);
	}
	// Wrap r.y in canonical range
	r.y = Math::WrapPi(r.y);
	return r;
}

inline eular_angles::type eular_angles::from_quat(quat_t const& m) {
	type r;
	float srcX = QuatOp::GetX(src);
	float srcY = QuatOp::GetY(src);
	float srcZ = QuatOp::GetZ(src);
	float srcW = QuatOp::GetW(src);
	// Extract sin(r.x)
	float sp = -2.0f * (srcY * srcZ - srcW * srcX);
	// Check for Gimbel Lock, giving slight tolerance for numerical imprecision
	if (Math::Abs(sp) > 0.9999f) {
		// Looking straight up or down
		r.x = Math::kPiBy2 * sp;
		// Compute r.y, slam r.z to zero
		r.y = Math::ArcTan2(-srcX * srcZ + srcW * srcY,
		                    0.5f - srcY * srcY - srcZ * srcZ);
		r.z = 0.0f;
	} else {
		// Compute angles. We don't have to use the "safe" asin
		// function because we already checked for range errors when
		// checking for Gimbel Lock
		r.x = Math::ArcSin(sp);
		r.y = Math::ArcTan2(srcX * srcZ + srcW * srcY,
		                    0.5f - srcX * srcX - srcY * srcY);
		r.z = Math::ArcTan2(srcX * srcY + srcW * srcZ,
		                    0.5f - srcX * srcX - srcZ * srcZ);
	}
	return r;
}

inline eular_angles::type eular_angles::from_inv_quat(quat_t const& m) {
	type r;
	float srcX = Quat::GetX(src);
	float srcY = Quat::GetY(src);
	float srcZ = Quat::GetZ(src);
	float srcW = Quat::GetW(src);
	// Extract sin(r.x)
	float sp = -2.0f * (srcY * srcZ + srcW * srcX);
	// Check for Gimbel Lock, giving slight tolerance for numerical imprecision
	if (Math::Abs(sp) > 0.9999f) {
		// Looking straight up or down
		r.x = Math::kPiBy2 * sp;
		// Compute heading, slam bank to zero
		r.y = Math::ArcTan2(-srcX * srcZ - srcW * srcY,
		                    0.5f - srcY * srcY - srcZ * srcZ);
		r.z = 0.0f;
	} else {
		// Compute angles. We don't have to use the "safe" asin
		// function because we already checked for range errors when
		// checking for Gimbel Lock
		r.x = Math::ArcSin(sp);
		r.y = Math::ArcTan2(srcX * srcZ - srcW * srcY,
		                    0.5f - srcX * srcX - srcY * srcY);
		r.z = Math::ArcTan2(srcX * srcY - srcW * srcZ,
		                    0.5f - srcX * srcX - srcZ * srcZ);
	}
	return r;
}

inline eular_angles::type eular_angles::from_mat4(mat4_t const& m) {
	return FromMat3x4(*reinterpret_cast<const TraitsMat3x4::type*>(&m));
}

inline eular_angles::type eular_angles::from_mat3(mat3_t const& m) {
	type r;
	// Extract sin(r.x) from m32.
	float sp = -src.m21;
	// Check for Gimbel Lock
	if (Math::Abs(sp) > 9.99999f) {
		// Looking straight up or down
		r.x = Math::kPiBy2 * sp;
		// Compute r.y, slam r.z to zero
		r.y = Math::ArcTan2(-src.m12, src.m00);
		r.z = 0.0f;
	} else {
		// Compute angles. We don't have to use the "safe" asin
		// function because we already checked for range errors when
		// checking for Gimbel Lock
		r.y = Math::ArcTan2(src.m20, src.m22);
		r.x = Math::ArcSin(sp);
		r.z = Math::ArcTan2(src.m01, src.m11);
	}
	return r;
}

} // namespace vml