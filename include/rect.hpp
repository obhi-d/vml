#pragma once
#include "multi_dim.hpp"
#include "vec2.hpp"

namespace vml {

namespace detail {
struct rect_traits {

	using type     = types::rect_t<float>;
	using ref      = type&;
	using pref     = std::conditional_t<types::is_pref_cref, type const&, type>;
	using cref     = type const&;
	using row_type = types::vec2_t<float>;
	using row_tag  = vec2;
	using scalar_type = float;

	enum { element_count = 8 };
	enum { row_count = 2 };
	enum { column_count = 2 };
};

template <typename traits> struct rect_base : public multi_dim<traits> {
	static inline type set(scalar_type left, scalar_type top, scalar_type right,
	                       scalar_type bottom) {
		return {{left, top}, {right, bottom}};
	}
	static inline row_type half_size(pref box) {
		return row_tag::half(row_tag::sub(box[1], box[0]));
	}
	static inline row_type size(pref box) { return row_tag::sub(box[1], box[0]); }
	static inline row_type center(pref box) {
		return row_tag::half(row_tag::add(box[1], box[0]));
	}
};

} // namespace detail

struct rect : public detail::rect_base<detail::rect_traits> {};

} // namespace vml
