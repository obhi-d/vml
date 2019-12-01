#pragma once
#include "detail/deduced_types.hpp"
#include "quad.hpp"
namespace vml {
template <typename concrete> struct multi_dim {

	using type      = typename concrete::type;
	using ref       = type&;
	using pref      = std::conditional_t<types::is_pref_cref, type const&, type>;
	using cref      = type const&;
	using scalar_type = typename concrete::scalar_type;
	using row_type  = typename concrete::type;

	enum { element_count = concrete::element_count };
	enum { row_count = concrete::row_count };
	enum { column_count = concrete::column_count };

	static inline row_type Row(pref m, uint32 i);
	static inline scalar_type Get(pref m, uint32 i, uint32 j);
	static inline type Add(pref m1, pref m2);
	static inline type Sub(pref m1, pref m2);
	static inline void SetRow(ref m, uint32 i, row_type_pref p);
};
} // namespace vml
