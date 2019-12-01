#pragma once
#include "mat_base.hpp"
namespace vml {

namespace detail {
struct mat3_traits {

	using type      = types::mat3_t<float>;
	using ref       = type&;
	using pref      = std::conditional_t<types::is_pref_cref, type const&, type>;
	using cref      = type const&;
	using row_type  = types::vec3a_t<float>;
	using scalar_type = float;

	enum { element_count = 12 };
	enum { row_count = 3 };
	enum { column_count = 4 };
};
} // namespace detail

struct mat3 : public mat_base<detail::mat3_traits> {
	static inline type Transpose(pref m);
	// @brief Create a rotation matrix from quaternion
	static inline type FromQuat(TraitsQuat::pref rot);
	// @brief Create a rotation matrix from quaternion
	static inline type FromRot(TraitsQuat::pref rot);
};
} // namespace vml
