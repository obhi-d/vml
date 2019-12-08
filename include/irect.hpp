#pragma once
#include "ivec2.hpp"
#include "rect.hpp"

namespace vml {
namespace detail {
struct irect_traits {

	using type     = types::rect_t<int>;
	using ref      = type&;
	using pref     = std::conditional_t<types::is_pref_cref, type const&, type>;
	using cref     = type const&;
	using row_type = types::vec2_t<int>;
	using row_tag  = ivec2;
	using scalar_type = int;

	enum { element_count = 8 };
	enum { row_count = 2 };
	enum { column_count = 2 };
};
} // namespace detail
struct irect : public detail::rect_base<detail::irect_traits> {};
}
