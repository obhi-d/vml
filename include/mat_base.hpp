#pragma once
#include "multi_dim.hpp"
#include "vec3.hpp"
#include "vec3a.hpp"
#include "vec4.hpp"
#include "quat.hpp"

namespace vml {
template <typename concrete> struct mat_base : public multi_dim<concrete> {
	using typename multi_dim<concrete>::type;
	using typename multi_dim<concrete>::ref;
	using typename multi_dim<concrete>::pref;
	using typename multi_dim<concrete>::scalar_type;
	using typename multi_dim<concrete>::row_tag;
	using typename multi_dim<concrete>::row_type;

	//! @brief Create a matrix from vector mapping that
	//! can rotate the vector axis1 to axis2 when post multiplied to axis1.
	static inline type from_vector_mapping(vec3::pref v1,
	                                     vec3::pref v2);
	//! @brief rotate vector in place
	static inline void rotate(pref m, vec3::type* io_stream,
	                          std::uint32_t i_stride, std::uint32_t i_count);
	//! @brief rotate vector
	static inline vec3a_t rotate(pref m, vec3a::pref v);
	//! @brief set a rotation for a given matrix
	static inline void set_rotation(ref _, quat::pref rot);
	//! @brief Returns a look at matrix based on a look at vector and up direction
	static inline void set_as_view(ref m, vec3a::pref view_dir, vec3a::pref up_dir);
};

template <typename concrete>
inline typename mat_base<concrete>::type mat_base<concrete>::from_vector_mapping(
    vec3::pref axis1, vec3::pref axis2) {
	/** \todo sse **/
	type m;
	scalar_type cs, xy_1_c, xz_1_c, yz_1_c;
	cs = vec3::dot(axis1, axis2);
	scalar_type _1_c;
	vec3_t axis = vec3::cross(axis1, axis2);
	// OPTIMIZE: we can also check the angle to
	// see if its a multiple of Pi.
	if (vml::abs(axis[0]) < vml::k_const_epsilon_med &&
	    vml::abs(axis[1]) < vml::k_const_epsilon_med &&
	    vml::abs(axis[2]) < vml::k_const_epsilon_med) {
		// take a cross for that
		axis = vec3::cross(axis1, vec3::set(0.0f, 1.0f, 0.0f));
		if (vml::abs(axis[0]) < vml::k_const_epsilon_med &&
		    vml::abs(axis[1]) < vml::k_const_epsilon_med &&
		    vml::abs(axis[2]) < vml::k_const_epsilon_med) {
			axis = vec3::cross(axis1, vec3::set(1.0f, 0.0f, 0.0f));
		}
	}
	_1_c         = 1.0f - cs;
	vec3_t xyzs = vec3::mul(axis, -vml::sqrt(1 - cs * cs));
	vec3_t mstr = vec3::mul(axis, axis);
	mstr         = mstr * _1_c;
	xy_1_c         = axis[0] * axis[1] * _1_c;
	xz_1_c         = axis[0] * axis[2] * _1_c;
	yz_1_c         = axis[1] * axis[2] * _1_c;

	m.r[0] = row_tag::set(cs + mstr[0], xy_1_c - xyzs[2], xz_1_c + xyzs[1], 0);
	m.r[1] = row_tag::set(xy_1_c + xyzs[2], cs + mstr[1], yz_1_c - xyzs[0], 0);
	m.r[2] = row_tag::set(xz_1_c - xyzs[1], yz_1_c + xyzs[0], cs + mstr[2], 0);

	return m;
}

template <typename concrete>
inline void mat_base<concrete>::rotate(pref m, vec3::type* io_stream,
                                       std::uint32_t i_stride, std::uint32_t i_count) {
	assert(io_stream);
	const std::uint8_t* inout_vec = (const std::uint8_t*)io_stream;
#if VML_USE_SSE_AVX
	std::uint8_t* out_vec = (std::uint8_t*)io_stream;
	union {
		quad_t v;
		scalar_type s[4];
	} store;

	for (std::uint32_t i = 0; i < i_count; i++) {
		quad_t x   = _mm_load_ps1(reinterpret_cast<const scalar_type*>(inout_vec));
		quad_t y   = _mm_load_ps1(reinterpret_cast<const scalar_type*>(inout_vec + 4));
		quad_t res = _mm_load_ps1(reinterpret_cast<const scalar_type*>(inout_vec + 8));
		res        = _mm_mul_ps(res, m.r[2]);
		y          = _mm_mul_ps(y, m.r[1]);
		res        = _mm_add_ps(res, y);
		x          = _mm_mul_ps(x, m.r[0]);
		res        = _mm_add_ps(res, x);
		res        = vec3a::normalize(res);
		_mm_store_ps(store.s, res);
		((scalar_type*)inout_vec)[0] = store.s[0];
		((scalar_type*)inout_vec)[1] = store.s[1];
		((scalar_type*)inout_vec)[2] = store.s[2];

		inout_vec += i_stride;
	}
#else
	quad_t x, y, z, r;
	for (std::uint32_t i = 0; i < i_count; i++) {
		x = quad::set(((scalar_type*)inout_vec)[0]);
		y = quad::set(((scalar_type*)inout_vec)[1]);
		z = quad::set(((scalar_type*)inout_vec)[2]);

		r = vec3a::mul(z, row(m, 2));
		r = vec3a::madd(y, row(m, 1), r);
		r = vec3a::normalize(vec3a::madd(x, row(m, 0), r));

		((scalar_type*)inout_vec)[0] = r[0];
		((scalar_type*)inout_vec)[1] = r[1];
		((scalar_type*)inout_vec)[2] = r[2];

		inout_vec += i_stride;
	}
#endif
}

template <typename concrete>
inline vec3a_t mat_base<concrete>::rotate(pref m, vec3a::pref v) {
#if VML_USE_SSE_AVX
	quad_t v_res  = _mm_shuffle_ps(v, v, _MM_SHUFFLE(0, 0, 0, 0));
	v_res         = _mm_mul_ps(v_res, m.r[0]);
	quad_t v_temp = _mm_shuffle_ps(v, v, _MM_SHUFFLE(1, 1, 1, 1));
	v_temp        = _mm_mul_ps(v_temp, m.r[1]);
	v_res         = _mm_add_ps(v_res, v_temp);
	v_temp        = _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 2, 2, 2));
	v_temp        = _mm_mul_ps(v_temp, m.r[2]);
	v_res         = _mm_add_ps(v_res, v_temp);
	return v_res;
#else
	quad_t r = vec3a::mul(vec3a::splat_z(v), row(m, 2));
	r        = vec3a::madd(vec3a::splat_y(v), row(m, 1), r);
	r        = vec3a::madd(vec3a::splat_x(v), row(m, 0), r);
	return r;
#endif
}

template <typename concrete>
inline void mat_base<concrete>::set_rotation(ref ret, quat::pref rot) {
#if VML_USE_SSE_AVX
	quad_t r0, r1, r2;
	quad_t q0, q1;
	quad_t v0, v1, v2;

	q0 = _mm_add_ps(rot, rot);
	q1 = _mm_mul_ps(rot, q0);

	v0 = _mm_shuffle_ps(q1, q1, _MM_SHUFFLE(3, 0, 0, 1));
	v0 = _mm_and_ps(v0, VML_CLEAR_W_VEC);
	v1 = _mm_shuffle_ps(q1, q1, _MM_SHUFFLE(3, 1, 2, 2));
	v1 = _mm_and_ps(v1, VML_CLEAR_W_VEC);
	r0 = _mm_sub_ps(_mm_set_ps(0.0f, 1.0f, 1.0f, 1.0f), v0);
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
	scalar_type xx = rot[0] * rot[0];
	scalar_type yy = rot[1] * rot[1];
	scalar_type zz = rot[2] * rot[2];
	scalar_type xy = rot[0] * rot[1];
	scalar_type xz = rot[0] * rot[2];
	scalar_type yz = rot[1] * rot[2];
	scalar_type wx = rot[3] * rot[0];
	scalar_type wy = rot[3] * rot[1];
	scalar_type wz = rot[3] * rot[2];

	ret.e[0][0] = (1 - 2 * (yy + zz));
	ret.e[0][1] = (2 * (xy + wz));
	ret.e[0][2] = (2 * (xz - wy));
	ret.e[0][3] = 0;

	ret.e[1][0] = (2 * (xy - wz));
	ret.e[1][1] = (1 - 2 * (xx + zz));
	ret.e[1][2] = (2 * (yz + wx));
	ret.e[1][3] = 0;

	ret.e[2][0] = (2 * (xz + wy));
	ret.e[2][1] = (2 * (yz - wx));
	ret.e[2][2] = (1 - 2 * (xx + yy));
	ret.e[2][3] = 0;
#endif
}

template <typename concrete>
inline void mat_base<concrete>::set_as_view(ref ret, vec3a::pref view_dir,
                                             vec3a::pref up_dir) {
	// TODO needs validation
	ret.r[2] = vec3a::normalize(view_dir);
	ret.r[0] = vec3a::normalize(vec3a::cross(view_dir, up_dir));
	ret.r[1] = vec3a::cross(ret.r[0], ret.r[2]);
}
} // namespace vml