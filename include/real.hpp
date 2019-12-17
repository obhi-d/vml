#pragma once

#include "detail/deduced_types.hpp"

namespace vml {

struct real {

	using type        = float;
	using ref         = type&;
	using pref        = type;
	using cref        = type;
	using scalar_type   = float;
	using row_type    = float;
	enum { element_count = 1 };

	inline static bool equals(pref v1, pref v2) {
		if (vml::almost_equals_ulps(v1, v2, 4))
			return true;
		return vml::almost_equals_rel_or_abs(v1, v2,
		                                         vml::k_max_relative_error,
		                                         vml::k_const_epsilon);

	}

	inline static bool isnan(pref v) { return std::isnan(v); }
	inline static bool isinf(pref v) { return std::isinf(v); }

};

} // namespace cppamth