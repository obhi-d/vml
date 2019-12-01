#pragma once
#include "mat_base.hpp"
namespace vml {

namespace detail {
struct aabb_traits {

	using type      = types::aabb_t<float>;
	using ref       = type&;
	using pref      = std::conditional_t<types::is_pref_cref, type const&, type>;
	using cref      = type const&;
	using scalar_type = float;
	using row_type  = types::vec4_t<scalar_t>;
	
	enum { element_count = 8 };
	enum { row_count = 2 };
	enum { column_count = 4 };
};
} // namespace detail

struct aabb : public mat_base<detail::aabb_traits> {
	static inline bool IsValid(pref box);
	static inline TraitsVec3A::type GetCenter(pref box);
	static inline TraitsVec3A::type GetSize(pref box);
	static inline TraitsVec3A::type GetHalfSize(pref box);
	static inline TraitsVec3A::type GetPoint(pref box, unsigned int i);
	static inline type Union(pref box, TraitsVec3A::pref point);
	static inline type Union(pref box, pref other);
	static inline type FromCenterExtends(TraitsVec3A::pref center,
	                                     TraitsVec3A::pref extends);
};
} // namespace vml
