#pragma once
#include "vec_base.hpp"

namespace vml {

namespace detail {
struct ivec2_traits {

	using scalar_type = int;
	using row_type    = scalar_type;
	using type = types::vec2_t<scalar_type>;
	using ref  = type&;
	using pref = std::conditional_t<types::is_pref_cref, type const&, type>;
	using cref = type const&;

	enum { element_count = 2 };
};
} // namespace detail
struct ivec2 : public vec_base<detail::ivec2_traits> {};
} // namespace vml