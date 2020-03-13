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
	using typename multi_dim<traits>::type;
	using typename multi_dim<traits>::scalar_type;
	using typename multi_dim<traits>::row_type;
	using typename multi_dim<traits>::row_tag;
	using typename multi_dim<traits>::pref;

	static inline type set(scalar_type left, scalar_type top, scalar_type right,
	                       scalar_type bottom) {
		return {row_type{left, top}, row_type{right, bottom}};
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

struct rect : public detail::rect_base<detail::rect_traits> {
	template <typename... Args>
	rect(Args&&... args)
	    : detail::rect_base<detail::rect_traits>(std::forward<Args>(args)...) {}
};
} // namespace vml