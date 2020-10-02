#pragma once
#include "vec_base.hpp"

namespace vml {

namespace detail {
struct ivec3_traits {
	using scalar_type = int;
	using row_type    = scalar_type;
	using type        = types::vec3_t<scalar_type>;
	using ref         = type&;
	using pref = std::conditional_t<types::is_pref_cref, type const&, type>;
	using cref = type const&;

	enum : unsigned int { element_count = 3 };
};
} // namespace detail
struct ivec3 : public vec_base<detail::ivec3_traits> {};
} // namespace vml