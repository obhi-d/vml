#pragma once
#include "vec_base.hpp"

namespace vml {

namespace detail {
struct vec3_traits {

	using type      = types::vec3_t<float>;
	using ref       = type&;
	using pref      = std::conditional_t<types::is_pref_cref, type const&, type>;
	using cref      = type const&;
	using scalar_type = float;
	using row_type  = float;
	using scalar_type = float;

	enum { element_count = 3 };
};
} // namespace detail
struct vec3 : public vec_base<detail::vec3_traits> {
	static inline type cross(pref q1, pref q2);
};

inline vec3::type vec3::cross(pref v1, pref v2) {
	return Set(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z,
	           v1.x * v2.y - v1.y * v2.x);
}
} // namespace vml