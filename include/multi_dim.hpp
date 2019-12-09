#pragma once
#include "detail/deduced_types.hpp"
#include "quad.hpp"
namespace vml {
template <typename concrete> struct multi_dim {
	using type = typename concrete::type;
	using ref  = type&;
	using pref = type const&;
	using cref = type const&;
	using scalar_type = typename concrete::scalar_type;
	using row_type    = typename concrete::row_type;
	using row_tag     = typename concrete::row_tag;

	enum { element_count = concrete::element_count };
	enum { row_count = concrete::row_count };
	enum { column_count = concrete::column_count };

	static inline row_type row(pref m, std::uint32_t i);
	static inline scalar_type get(pref m, std::uint32_t i, std::uint32_t j);
	static inline type add(pref m1, pref m2);
	static inline type sub(pref m1, pref m2);
	static inline void set_row(ref m, std::uint32_t i, row_type const& p);
};

template <typename concrete>
inline typename multi_dim<concrete>::row_type multi_dim<concrete>::row(
    pref m, std::uint32_t i) {
	return m.r[i];
}

template <typename concrete>
inline typename multi_dim<concrete>::scalar_type multi_dim<concrete>::get(
    pref m, std::uint32_t i, std::uint32_t j) {
	return m.e[i][j];
}

template <typename concrete>
inline typename multi_dim<concrete>::type multi_dim<concrete>::add(pref m1,
                                                                   pref m2) {
	type r;
	for (unsigned int i = 0; i < concrete::row_count; ++i)
		r.r[i] = row_tag::add(m1.r[i], m2.r[i]);
	return r;
}

template <typename concrete>
inline typename multi_dim<concrete>::type multi_dim<concrete>::sub(pref m1,
                                                                   pref m2) {
	type r;
	for (unsigned int i = 0; i < concrete::row_count; ++i)
		r.r[i] = row_tag::sub(m1.r[i], m2.r[i]);
	return r;
}
template <typename concrete>
inline void multi_dim<concrete>::set_row(ref m, std::uint32_t i,
                                         row_type const& r) {
	m.r[i] = r;
}
} // namespace vml