#pragma once
#include "mat_base.hpp"
namespace vml {
namespace detail {
struct mat3_traits {
	using type     = types::mat3_t<float>;
	using ref      = type&;
	using pref     = std::conditional_t<types::is_pref_cref, type const&, type>;
	using cref     = type const&;
	using row_type = types::vec3a_t<float>;
	using scalar_type = float;

	enum { element_count = 12 };
	enum { row_count = 3 };
	enum { column_count = 4 };
};
} // namespace detail

struct mat3 : public mat_base<detail::mat3_traits> {
	static inline type Transpose(pref m);
	// @brief Create a rotation matrix from quaternion
	static inline type FromQuat(TraitsQuat::pref rot);
	// @brief Create a rotation matrix from quaternion
	static inline type FromRot(TraitsQuat::pref rot);
};

inline MatOp<V_Matrix3x4>::type MatOp<V_Matrix3x4>::Transpose(pref m) {
#if L_VECTOR_MATH_TYPE_IS_SSE
	Matrix3x4 ret;
	//    std::swap(m.e[0][1], m.m10);
	//    std::swap(m.m02, m.e[2][0]);
	//    std::swap(m.e[1][2], m.e[2][1]);
	ret.r[0] = _mm_move_ss(
	    _mm_shuffle_ps(m.r[1], m.r[2], _MM_SHUFFLE(3, 0, 0, 3)), m.r[0]);
	ret.r[1] =
	    _mm_shuffle_ps(_mm_shuffle_ps(m.r[0], m.r[1], _MM_SHUFFLE(3, 1, 3, 1)),
	                   m.r[2], _MM_SHUFFLE(3, 1, 2, 0));
	ret.r[2] =
	    _mm_shuffle_ps(_mm_shuffle_ps(m.r[0], m.r[1], _MM_SHUFFLE(3, 2, 3, 2)),
	                   m.r[2], _MM_SHUFFLE(3, 2, 2, 0));
	return ret;

#else
	// basic swapping
	//
	//  a   b   c
	//  d   e   f
	//  g   h   i
	// swap(b,d) swap(g,c), swap(h,f)
	Matrix3x4 ret = m;
	std::swap(ret.e[0][1], ret.m10);
	std::swap(ret.e[1][2], ret.e[2][1]);
	std::swap(ret.m02, ret.e[2][0]);
	return ret;

#endif
}

inline MatOp<V_Matrix3x4>::type MatOp<V_Matrix3x4>::FromQuat(
    TraitsQuat::pref rot) {
	type ret;
	SetRot(ret, rot);
	return ret;
}

inline MatOp<V_Matrix3x4>::type MatOp<V_Matrix3x4>::FromRot(
    TraitsQuat::pref rot) {
	return FromQuat(rot);
}
} // namespace vml