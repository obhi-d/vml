#pragma once

#include "mat4.hpp"
#include "plane.hpp"
#include "vec4.hpp"
#include <tuple>

namespace vml {

struct frustum_t {
	struct coherency {
		std::uint32_t mask_hierarchy;
		std::uint32_t plane;
	};

	// common frustums will have these planes
	enum plane_type {
		k_near,
		k_far,
		k_left,
		k_right,
		k_top,
		k_bottom,
	};

	enum { k_fixed_plane_count = 6 };

	frustum_t() : plane_count(0) {}
	frustum_t(frustum_t const& i_other) : plane_count(i_other.plane_count) {
		if (plane_count < k_fixed_plane_count && plane_count > 0) {
			for (std::uint32_t i = 0; i < plane_count; ++i)
				planes[i] = i_other.planes[i];
		} else {
			pplanes = vml::allocate<vml::plane_t>(sizeof(vml::plane_t)*plane_count, alignof(vml::plane_t));
			for (std::uint32_t i = 0; i < plane_count; ++i)
				pplanes[i] = i_other.pplanes[i];
		}
	}
	frustum_t(frustum_t&& i_other) : plane_count(i_other.plane_count) {
		if (plane_count < k_fixed_plane_count && plane_count > 0) {
			for (std::uint32_t i = 0; i < plane_count; ++i)
				planes[i] = i_other.planes[i];
		} else {
			pplanes        = i_other.pplanes;
			i_other.pplanes = nullptr;
		}
	}
	frustum_t(plane_t const* i_planes, std::uint32_t i_size) : plane_count(i_size) {
		if (plane_count < k_fixed_plane_count && plane_count > 0) {
			for (std::uint32_t i = 0; i < plane_count; ++i)
				planes[i] = i_planes[i];
		} else {
			pplanes = vml::allocate<vml::plane_t>(sizeof(vml::plane_t) * plane_count,
			                                      alignof(vml::plane_t));
			for (std::uint32_t i = 0; i < plane_count; ++i)
				pplanes[i] = i_planes[i];
		}
	}
	~frustum_t();

	inline frustum_t& operator=(frustum_t const& i_other) {
		if (plane_count > k_fixed_plane_count && pplanes)
			vml::deallocate(pplanes, sizeof(vml::plane_t) * plane_count);

		plane_count = i_other.plane_count;
		if (plane_count < k_fixed_plane_count && plane_count > 0) {
			for (std::uint32_t i = 0; i < plane_count; ++i)
				planes[i] = i_other.planes[i];
		} else {
			pplanes = vml::allocate<vml::plane_t>(sizeof(vml::plane_t) * plane_count,
			                                      alignof(vml::plane_t));
			for (std::uint32_t i = 0; i < plane_count; ++i)
				pplanes[i] = i_other.pplanes[i];
		}
		return *this;
	}

	inline frustum_t& operator=(frustum_t&& i_other) {
		if (plane_count > k_fixed_plane_count && pplanes)
			vml::deallocate(pplanes, sizeof(vml::plane_t) * plane_count);

		plane_count = i_other.plane_count;
		if (plane_count < k_fixed_plane_count && plane_count > 0) {
			for (std::uint32_t i = 0; i < plane_count; ++i)
				planes[i] = i_other.planes[i];
		} else {
			pplanes        = i_other.pplanes;
			i_other.pplanes = nullptr;
		}
		return *this;
	}

	inline std::uint32_t count() const { return plane_count; }

	inline plane_t operator[](std::uint32_t i) const {
		assert(i < plane_count);
		return (plane_count > k_fixed_plane_count) ? pplanes[i] : planes[i];
	}

	inline void modify(std::uint32_t i, plane_t const& p) {
		assert(i < plane_count);
		plane_t* dest = (plane_count > k_fixed_plane_count) ? &pplanes[i] : &planes[i];
		*dest        = p;
	}

	/**
	 * @remarks Construct from a transpose(view*projection) matrix
	 * @param mat transpose(View*Projection) or transpose(Proj)*transpose(View)
	 * matrix
	 */
	void build(mat4_t const& combo) {
		// Near clipping planeT
		planes[k_near] = plane::normalize(vec4::negate(mat4::row(combo, 2)));
		// Far clipping planeT
		planes[k_far] =
		    plane::normalize(vec4::sub(mat4::row(combo, 2), mat4::row(combo, 3)));
		// Left clipping planeT
		planes[k_left] = plane::normalize(
		    vec4::negate(vec4::add(mat4::row(combo, 0), mat4::row(combo, 3))));
		// Right clipping planeT
		planes[k_right] =
		    plane::normalize(vec4::sub(mat4::row(combo, 0), mat4::row(combo, 3)));
		// Top clipping planeT
		planes[k_top] =
		    plane::normalize(vec4::sub(mat4::row(combo, 1), mat4::row(combo, 3)));
		// Bottom clipping planeT
		planes[k_bottom] = plane::normalize(
		    vec4::negate(vec4::add(mat4::row(combo, 1), mat4::row(combo, 3))));
	}

	plane_t const* get_all() const {
		return (plane_count > k_fixed_plane_count) ? pplanes : planes;
	}
	plane_t* get_all() {
		return (plane_count > k_fixed_plane_count) ? pplanes : planes;
	}

	union {
		plane_t planes[k_fixed_plane_count];
		plane_t* pplanes;	
	};
	// if plane_count <= 6, we use planes, otherwise we use planes
	std::uint32_t plane_count;
};

struct frustum {
	static inline void set(frustum_t& _, mat4::pref m) {
		_.build(m);
	}
	static inline std::pair<plane_t const*, std::uint32_t> get_planes(frustum_t const& _) {
		return std::make_pair(_.get_all(), _.count());
	}
	static inline std::pair<plane_t*, std::uint32_t> get_planes(frustum_t& _) {
		return std::make_pair(_.get_all(), _.count());
	}
	static inline void set_plane(frustum_t& _,std::uint32_t i, plane_t const& p) {
		_.modify(i, p);
	}
};

} // namespace vml
