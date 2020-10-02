#pragma once
#include "vec_base.hpp"

namespace vml {

namespace detail {
struct vec3_traits {

	using type = types::vec3_t<float>;
	using ref  = type&;
	using pref = std::conditional_t<types::is_pref_cref, type const&, type>;
	using cref = type const&;
	using scalar_type = float;
	using row_type    = float;

	enum : unsigned int { element_count = 3 };
};
} // namespace detail
struct vec3 : public vec_base<detail::vec3_traits> {
	static inline type cross(pref q1, pref q2);
};

inline vec3::type vec3::cross(pref v1, pref v2) {
	return set(v1[1] * v2[2] - v1[2] * v2[1], v1[2] * v2[0] - v1[0] * v2[2],
	           v1[0] * v2[1] - v1[1] * v2[0]);
}
} // namespace vml