
#pragma once

#include "detail/deduced_types.hpp"
#include "real.hpp"

namespace vml {

template <typename concrete>
struct vec_base : public concrete {

	using type      = typename concrete::type;
	using ref       = typename concrete::ref;
	using pref      = typename concrete::pref;
	using cref      = typename concrete::cref;
	using scalar_type = typename concrete::scalar_type;
	using row_type  = typename concrete::row_type;
	using scalar_type = typename concrete::scalar_type;
	enum { element_count = concrete::element_count };

	static inline bool equals(pref v1, pref v2);
	static inline bool isnan(pref v);
	static inline bool isinf(pref v);
	static inline type isnanv(pref v);
	static inline type isinfv(pref v);
	static inline type set(scalar_type v);
	static inline type set(scalar_type x, scalar_type y);
	static inline type set(scalar_type x, scalar_type y, scalar_type z);
	static inline type set(scalar_type x, scalar_type y, scalar_type z, scalar_type w);
	static inline type set(scalar_type const* v);
	static inline type set_unaligned(scalar_type const* v);
	static inline type x(pref v, scalar_type x);
	static inline type y(pref v, scalar_type y);
	static inline type z(pref v, scalar_type z);
	static inline type w(pref v, scalar_type w);
	static inline scalar_type x(pref v);
	static inline scalar_type y(pref v);
	static inline scalar_type z(pref v);
	static inline scalar_type w(pref v);
	static inline type zero();
	static inline type splat_x(pref v);
	static inline type splat_y(pref v);
	static inline type splat_z(pref v);
	static inline type splat_w(pref v);
	static inline type select(pref v1, pref v2, pref control);
	static inline scalar_type get(pref v, std::uint32_t i);
	static inline type abs(pref v);
	static inline type negate(pref v);
	static inline type add(pref a, pref b);
	static inline type sub(pref a, pref b);
	static inline type mul(pref a, pref b);
	static inline type mul(pref a, scalar_type b);
	static inline type mul(scalar_type b, pref a);
	static inline type half(pref a);
	static inline type div(pref a, pref b);
	static inline type madd(pref v, pref m, pref a);
	static inline scalar_type hadd(pref q1);
	static inline bool greater_all(pref q1, pref q2);
	static inline bool greater_any(pref q1, pref q2);
	static inline bool lesser_all(pref q1, pref q2);
	static inline bool lesser_any(pref q1, pref q2);
	static inline type vdot(pref q1, pref q2);
	static inline scalar_type dot(pref q1, pref q2);
	static inline scalar_type sqlength(pref c1);
	static inline scalar_type length(pref c1);
	static inline scalar_type distance(pref vec1, pref vec2);
	static inline scalar_type sqdistance(pref vec1, pref vec2);
	static inline type normalize(pref v);
	static inline type lerp(pref src, pref dest, scalar_type t);
	static inline type recip_sqrt(pref qpf);
};

template <typename concrete>
inline bool vec_base<concrete>::equals(pref v1, pref v2) {
	for (std::uint32_t i = 0; i < element_count; ++i)
		if (!real::equals(v1[i], v2[i]))
			return false;
	return true;
}
template <typename concrete> inline bool vec_base<concrete>::isnan(pref v) {
	for (std::uint32_t i = 0; i < element_count; ++i)
		if (!real::isnan(v[i]))
			return true;
	return false;
}
template <typename concrete> inline bool vec_base<concrete>::isinf(pref v) {
	for (std::uint32_t i = 0; i < element_count; ++i)
		if (!real::isinf(v[i]))
			return true;
	return false;
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::isnanv(pref v) {
	type ret;
	for (std::uint32_t i = 0; i < element_count; ++i)
		ret[i] = (v[i] != v[i]);
	return ret;
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::isinfv(pref v) {
	type ret;
	for (std::uint32_t i = 0; i < element_count; ++i)
		ret[i] = (scalar_type)real::isinf(v[i]);
	return ret;
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::set(scalar_type v) {
	type ret;
	for (std::uint32_t i = 0; i < element_count; ++i)
		ret[i] = v;
	return ret;
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::set(scalar_type x,
                                                      scalar_type y) {
	return {x, y};
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::set(scalar_type x, scalar_type y,
                                                      scalar_type z) {
	return {x, y, z};
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::set(scalar_type x, scalar_type y,
                                                      scalar_type z,
                                   scalar_type w) {
	return {x, y, z, w};
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::set(scalar_type const* v) {
	type ret;
	for (std::uint32_t i = 0; i < element_count; ++i)
		ret[i] = v[i];
	return ret;
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::set_unaligned(
    scalar_type const* v) {
	type ret;
	for (std::uint32_t i = 0; i < element_count; ++i)
		ret[i] = v[i];
	return ret;
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::x(pref v, scalar_type x) {
	if constexpr (element_count == 1)
		return {x};
	else if constexpr (element_count == 2)
		return {x, v[1]};
	else if constexpr (element_count == 3)
		return {x, v[1], v[2]};
	else if constexpr (element_count == 4)
		return {x, v[1], v[2], v[3]};
	else {
		type ret = v;
		ret[0]   = x;
		return ret;
	}
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::y(pref v, scalar_type y) {
	if constexpr (element_count == 1)
		return v;
	else if constexpr (element_count == 2)
		return {v[0], y};
	else if constexpr (element_count == 3)
		return {v[0], y, v[2]};
	else if constexpr (element_count == 4)
		return {v[0], y, v[2], v[3]};
	else {
		type ret = v;
		ret[1]   = y;
		return ret;
	}
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::z(pref v, scalar_type z) {
	if constexpr (element_count == 1)
		return v;
	else if constexpr (element_count == 2)
		return v;
	else if constexpr (element_count == 3)
		return {v[0], v[1], z};
	else if constexpr (element_count == 4)
		return {v[0], v[1], z, v[3]};
	else {
		type ret = v;
		ret[2]   = z;
		return ret;
	}
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::w(pref v, scalar_type w) {
	if constexpr (element_count == 1)
		return v;
	else if constexpr (element_count == 2)
		return v;
	else if constexpr (element_count == 3)
		return v;
	else if constexpr (element_count == 4)
		return {v[0], v[1], v[2], w};
	else {
		type ret = v;
		ret[3]   = w;
		return ret;
	}
}
template <typename concrete>
inline vec_base<concrete>::scalar_type vec_base<concrete>::x(pref v) {
	return v[0];
}
template <typename concrete>
inline vec_base<concrete>::scalar_type vec_base<concrete>::y(pref v) {
	return v[1];
}
template <typename concrete>
inline vec_base<concrete>::scalar_type vec_base<concrete>::z(pref v) {
	return v[2];
}
template <typename concrete>
inline vec_base<concrete>::scalar_type vec_base<concrete>::w(pref v) {
	return v[3];
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::zero() {
	return {};
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::splat_x(pref v) {
	constexpr std::uint32_t k_index = 0;
	if constexpr (element_count == 1)
		return {v[k_index]};
	else if constexpr (element_count == 2)
		return {v[k_index], v[k_index]};
	else if constexpr (element_count == 3)
		return {v[k_index], v[k_index], v[k_index]};
	else if constexpr (element_count == 4)
		return {v[k_index], v[k_index], v[k_index], v[k_index]};
	else {
		type ret;
		for (std::uint32_t i = 0; i < element_count; ++i)
			ret[i] = v[k_index];
		return ret;
	}
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::splat_y(pref v) {
	constexpr std::uint32_t k_index = 1;
	if constexpr (element_count == 2)
		return {v[k_index], v[k_index]};
	else if constexpr (element_count == 3)
		return {v[k_index], v[k_index], v[k_index]};
	else if constexpr (element_count == 4)
		return {v[k_index], v[k_index], v[k_index], v[k_index]};
	else {
		type ret;
		for (std::uint32_t i = 0; i < element_count; ++i)
			ret[i] = v[k_index];
		return ret;
	}
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::splat_z(pref v) {
	constexpr std::uint32_t k_index = 2;
	if constexpr (element_count == 3)
		return {v[k_index], v[k_index], v[k_index]};
	else if constexpr (element_count == 4)
		return {v[k_index], v[k_index], v[k_index], v[k_index]};
	else {
		type ret;
		for (std::uint32_t i = 0; i < element_count; ++i)
			ret[i] = v[k_index];
		return ret;
	}
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::splat_w(pref v) {
	constexpr std::uint32_t k_index = 3;
	if constexpr (element_count == 4)
		return {v[k_index], v[k_index], v[k_index], v[k_index]};
	else {
		type ret;
		for (std::uint32_t i = 0; i < element_count; ++i)
			ret[i] = v[k_index];
		return ret;
	}
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::select(pref v1, pref v2,
                                                         pref control) {
	type ret;
	std::uint32_t* iret = reinterpret_cast<std::uint32_t*>(&ret);
	std::uint32_t* iv1  = reinterpret_cast<std::uint32_t*>(&v1);
	std::uint32_t* iv2  = reinterpret_cast<std::uint32_t*>(&v2);
	std::uint32_t* ic   = reinterpret_cast<std::uint32_t*>(&control);
	for (std::uint32_t i = 0; i < element_count; ++i)
		iret[i] = (~ic[i] & iv1[i]) | (ic[i] & iv2[i]);
	return ret;
}
template <typename concrete>
inline vec_base<concrete>::scalar_type vec_base<concrete>::get(pref v,
                                                           std::uint32_t i) {
	return v[i];
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::abs(pref v) {
	type ret;
	for (std::uint32_t i = 0; i < element_count; ++i)
		ret[i] = std::abs(v[i]);
	return ret;
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::negate(pref v) {
	type ret;
	for (std::uint32_t i = 0; i < element_count; ++i)
		ret[i] = -(v[i]);
	return ret;
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::add(pref a, pref b) {
	type ret;
	for (std::uint32_t i = 0; i < element_count; ++i)
		ret[i] = a[i] + b[i];
	return ret;
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::sub(pref a, pref b) {
	type ret;
	for (std::uint32_t i = 0; i < element_count; ++i)
		ret[i] = a[i] - b[i];
	return ret;
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::mul(pref a, pref b) {
	type ret;
	for (std::uint32_t i = 0; i < element_count; ++i)
		ret[i] = a[i] * b[i];
	return ret;
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::mul(pref a, scalar_type b) {
	type ret;
	for (std::uint32_t i = 0; i < element_count; ++i)
		ret[i] = a[i] * b;
	return ret;
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::mul(scalar_type b, pref a) {
	type ret;
	for (std::uint32_t i = 0; i < element_count; ++i)
		ret[i] = a[i] * b;
	return ret;
}
template <typename concrete> inline type vec_base<concrete>::half(pref a) {
	type ret;
	for (std::uint32_t i = 0; i < element_count; ++i)
		ret[i] = a[i] / 2;
	return ret;
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::div(pref a, pref b) {
	type ret;
	for (std::uint32_t i = 0; i < element_count; ++i)
		ret[i] = a[i] / b[i];
	return ret;
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::madd(pref v, pref m, pref a) {
	type ret;
	for (std::uint32_t i = 0; i < element_count; ++i)
		ret[i] = (v[i] * m[i]) + a[i];
	return ret;
}
template <typename concrete>
inline vec_base<concrete>::scalar_type vec_base<concrete>::hadd(pref q1) {
	if constexpr (element_count == 1)
		return {q[0]};
	else if constexpr (element_count == 2)
		return q[0] + q[1];
	else if constexpr (element_count == 3)
		return q[0] + q[1] + q[2];
	else if constexpr (element_count == 4)
		return q[0] + q[1] + q[2] + q[3];
	else {
		scalar_type ret = v[0];
		for (std::uint32_t i = 1; i < element_count; ++i)
			ret[i] += v[i];
		return ret;
	}
}
template <typename concrete>
inline bool vec_base<concrete>::greater_all(pref q1, pref q2) {
	for (std::uint32_t i = 0; i < element_count; ++i)
		if (q1[i] <= q2[i])
			return false;
	return true;
}
template <typename concrete>
inline bool vec_base<concrete>::greater_any(pref q1, pref q2) {
	for (std::uint32_t i = 0; i < element_count; ++i)
		if (q1[i] > q2[i])
			return true;
	return false;
}
template <typename concrete>
inline bool vec_base<concrete>::lesser_all(pref q1, pref q2) {
	for (std::uint32_t i = 0; i < element_count; ++i)
		if (q1[i] >= q2[i])
			return false;
	return true;
}
template <typename concrete>
inline bool vec_base<concrete>::lesser_any(pref q1, pref q2) {
	for (std::uint32_t i = 0; i < element_count; ++i)
		if (q1[i] < q2[i])
			return true;
	return false;
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::vdot(pref q1, pref q2) {
	return {dot(q1, q2)};
}
template <typename concrete>
inline vec_base<concrete>::scalar_type vec_base<concrete>::dot(pref q1, pref q2) {
	return hadd(mul(q1, q2));
}
template <typename concrete>
inline vec_base<concrete>::scalar_type vec_base<concrete>::sqlength(pref c1) {
	return (dot(c1, c1));
}
template <typename concrete>
inline vec_base<concrete>::scalar_type vec_base<concrete>::length(pref c1) {
	return std::sqrt(sqlength(c1));
}
template <typename concrete>
inline vec_base<concrete>::scalar_type vec_base<concrete>::distance(pref vec1,
                                                                  pref vec2) {
	return length(sub(vec2, vec1));
}
template <typename concrete>
inline vec_base<concrete>::scalar_type vec_base<concrete>::sqdistance(pref vec1,
                                                                    pref vec2) {
	return sqlength(sub(vec2, vec1));
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::normalize(pref v) {
	return mul(v, vml::recip_sqrt(sqlength(v)));
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::lerp(pref src, pref dest,
                                                       scalar_type t) {
	return madd(set(t), sub(dst, src), src);
}
template <typename concrete>
inline vec_base<concrete>::type vec_base<concrete>::recip_sqrt(pref qpf) {
	type ret;
	for (std::uint32_t i = 0; i < element_count; ++i)
		ret[i] = vml::recip_sqrt(qpf[i]);
	return ret;
}
} // namespace vml
