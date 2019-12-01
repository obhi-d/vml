#pragma once
#include "quad.hpp"

namespace vml {
namespace detail {} // namespace detail
struct eular_angles : public quad {
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
inline eular_angles::type eular_angles::canonize(
    pref m) { // First, wrap r[0] in range -pi ... pi
	type r;
	r[0] = Math::WrapPi(m[0]);
	r[1] = m[1];
	r[2] = m[2];
	// Now, check for "the back side" of the matrix r[0] outside
	// the canonical range of -pi/2 ... pi/2
	if (r[0] < -Math::kPiBy2) {
		r[0] = -Math::kPiBy2 - r[0];
		r[1] += Math::kPi;
		r[2] += Math::kPi;
	} else if (r[0] > Math::kPiBy2) {
		r[0] = Math::kPi - r[0];
		r[1] += Math::kPi;
		r[2] += Math::kPi;
	}
	// Now check for the gimbel Lock case (within a slight tolerance)
	if (Math::Abs(r[0]) > Math::kPiBy2 - 1e-4) {
		// We are in gimbel Lock. Assign all rotation
		// about the vertical axis to r[1]
		r[1] += r[2];
		r[2] = 0.0f;
	} else {
		// Not in gimbel Lock. Wrap the r[2] angle in
		// canonical range
		r[2] = Math::WrapPi(r[2]);
	}
	// Wrap r[1] in canonical range
	r[1] = Math::WrapPi(r[1]);
	return r;
}

inline eular_angles::type eular_angles::from_quat(quat_t const& m) {
	type r;
	float srcX = QuatOp::x(src);
	float srcY = QuatOp::y(src);
	float srcZ = QuatOp::z(src);
	float srcW = QuatOp::GetW(src);
	// Extract sin(r[0])
	float sp = -2.0f * (srcY * srcZ - srcW * srcX);
	// Check for Gimbel Lock, giving slight tolerance for numerical imprecision
	if (Math::Abs(sp) > 0.9999f) {
		// Looking straight up or down
		r[0] = Math::kPiBy2 * sp;
		// Compute r[1], slam r[2] to zero
		r[1] = Math::ArcTan2(-srcX * srcZ + srcW * srcY,
		                     0.5f - srcY * srcY - srcZ * srcZ);
		r[2] = 0.0f;
	} else {
		// Compute angles. We don't have to use the "safe" asin
		// function because we already checked for range errors when
		// checking for Gimbel Lock
		r[0] = Math::ArcSin(sp);
		r[1] = Math::ArcTan2(srcX * srcZ + srcW * srcY,
		                     0.5f - srcX * srcX - srcY * srcY);
		r[2] = Math::ArcTan2(srcX * srcY + srcW * srcZ,
		                     0.5f - srcX * srcX - srcZ * srcZ);
	}
	return r;
}

inline eular_angles::type eular_angles::from_inv_quat(quat_t const& m) {
	type r;
	float srcX = quat::x(src);
	float srcY = quat::y(src);
	float srcZ = quat::z(src);
	float srcW = quat::GetW(src);
	// Extract sin(r[0])
	float sp = -2.0f * (srcY * srcZ + srcW * srcX);
	// Check for Gimbel Lock, giving slight tolerance for numerical imprecision
	if (Math::Abs(sp) > 0.9999f) {
		// Looking straight up or down
		r[0] = Math::kPiBy2 * sp;
		// Compute heading, slam bank to zero
		r[1] = Math::ArcTan2(-srcX * srcZ - srcW * srcY,
		                     0.5f - srcY * srcY - srcZ * srcZ);
		r[2] = 0.0f;
	} else {
		// Compute angles. We don't have to use the "safe" asin
		// function because we already checked for range errors when
		// checking for Gimbel Lock
		r[0] = Math::ArcSin(sp);
		r[1] = Math::ArcTan2(srcX * srcZ - srcW * srcY,
		                     0.5f - srcX * srcX - srcY * srcY);
		r[2] = Math::ArcTan2(srcX * srcY - srcW * srcZ,
		                     0.5f - srcX * srcX - srcZ * srcZ);
	}
	return r;
}

inline eular_angles::type eular_angles::from_mat4(mat4_t const& m) {
	return FromMat3x4(*reinterpret_cast<const TraitsMat3x4::type*>(&m));
}

inline eular_angles::type eular_angles::from_mat3(mat3_t const& m) {
	type r;
	// Extract sin(r[0]) from m32.
	float sp = -src.m21;
	// Check for Gimbel Lock
	if (Math::Abs(sp) > 9.99999f) {
		// Looking straight up or down
		r[0] = Math::kPiBy2 * sp;
		// Compute r[1], slam r[2] to zero
		r[1] = Math::ArcTan2(-src.m12, src.m00);
		r[2] = 0.0f;
	} else {
		// Compute angles. We don't have to use the "safe" asin
		// function because we already checked for range errors when
		// checking for Gimbel Lock
		r[1] = Math::ArcTan2(src.m20, src.m22);
		r[0] = Math::ArcSin(sp);
		r[2] = Math::ArcTan2(src.m01, src.m11);
	}
	return r;
}
} // namespace vml