#pragma once
#include "detail/deduced_types.hpp"
#include "quad.hpp"
namespace vml {
template <typename concrete> struct multi_dim {
	using type = typename concrete::type;
	using ref  = type&;
	using pref = std::conditional_t<types::is_pref_cref, type const&, type>;
	using cref = type const&;
	using scalar_type = typename concrete::scalar_type;
	using row_type    = typename concrete::type;

	enum { element_count = concrete::element_count };
	enum { row_count = concrete::row_count };
	enum { column_count = concrete::column_count };

	static inline row_type row(pref m, std::uint32_t i);
	static inline scalar_type Get(pref m, std::uint32_t i, std::uint32_t j);
	static inline type add(pref m1, pref m2);
	static inline type sub(pref m1, pref m2);
	static inline void SetRow(ref m, std::uint32_t i, row_type_pref p);
};

template <typename concrete>
inline typename multi_dim<concrete>::row_type multi_dim<concrete>::row(
    pref m, uint32 i) {
	return m.r[i];
}

template <typename concrete>
inline typename multi_dim<concrete>::base_type multi_dim<concrete>::Get(
    pref m, uint32 i, uint32 j) {
	return VecOp<row_type>::Get(m.r[i], j);
}

template <typename concrete>
inline typename multi_dim<concrete>::type multi_dim<concrete>::add(pref m1,
                                                                   pref m2) {
	type r;
	for (unsigned int i = 0; i < _rows; ++i)
		r.r[i] = row_op::add(m1.r[i], m2.r[i]);
	return r;
}

template <typename concrete>
inline typename multi_dim<concrete>::type multi_dim<concrete>::sub(pref m1,
                                                                   pref m2) {
	type r;
	for (unsigned int i = 0; i < _rows; ++i)
		r.r[i] = row_op::sub(m1.r[i], m2.r[i]);
	return r;
}
template <typename concrete>
inline void multi_dim<concrete>::SetRow(ref m, uint32 i, row_type_pref r) {
	m.r[i] = r;
}
} // namespace vml