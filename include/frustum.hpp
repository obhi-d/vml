#pragma once

#include "mat4.hpp"
#include "plane.hpp"
#include "vec4.hpp"

namespace vml {

struct frustum_t {
	struct coherency {
		std::uint32_t mask;
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

	frustum_t() : size(0) {}
	frustum_t(frustum_t const& i_other) : size(i_other.size) {
		if (size < k_fixed_plane_count && size > 0) {
			for (std::uint32_t i = 0; i < size; ++i)
				planes[i] = i_other.planes[i];
		} else {
			pplanes = vml::allocate<vml::plane_t>(sizeof(vml::plane_t)*size, alignof(vml::plane_t));
			for (std::uint32_t i = 0; i < size; ++i)
				pplanes[i] = i_other.pplanes[i];
		}
	}
	frustum_t(frustum_t&& i_other) : size(i_other.size) {
		if (size < k_fixed_plane_count && size > 0) {
			for (std::uint32_t i = 0; i < size; ++i)
				planes[i] = i_other.planes[i];
		} else {
			pplanes        = i_other.pplanes;
			i_other.pplanes = nullptr;
		}
	}
	frustum_t(const plane_t* i_planes, std::uint32_t iSize) : size(iSize) {
		if (size < k_fixed_plane_count && size > 0) {
			for (std::uint32_t i = 0; i < size; ++i)
				planes[i] = i_planes[i];
		} else {
			pplanes = vml::allocate<vml::plane_t>(sizeof(vml::plane_t) * size,
			                                      alignof(vml::plane_t));
			for (std::uint32_t i = 0; i < size; ++i)
				pplanes[i] = i_planes[i];
		}
	}
	~frustum_t();

	inline frustum_t& operator=(frustum_t const& i_other) {
		if (size > k_fixed_plane_count && pplanes)
			vml::deallocate(pplanes, sizeof(vml::plane_t) * size);

		size = i_other.size;
		if (size < k_fixed_plane_count && size > 0) {
			for (std::uint32_t i = 0; i < size; ++i)
				planes[i] = i_other.planes[i];
		} else {
			pplanes = vml::allocate<vml::plane_t>(sizeof(vml::plane_t) * size,
			                                      alignof(vml::plane_t));
			for (std::uint32_t i = 0; i < size; ++i)
				pplanes[i] = i_planes[i];
		}
		return *this;
	}

	inline frustum_t& operator=(frustum_t&& i_other) {
		if (size > k_fixed_plane_count && pplanes)
			vml::deallocate(pplanes, sizeof(vml::plane_t) * size);

		size = i_other.size;
		if (size < k_fixed_plane_count && size > 0) {
			for (std::uint32_t i = 0; i < size; ++i)
				planes[i] = i_other.planes[i];
		} else {
			pplanes        = i_other.pplanes;
			i_other.planes = nullptr;
		}
		return *this;
	}

	inline std::uint32_t size() const { return size; }

	inline plane_t operator[](std::uint32_t i) const {
		assert(i < size);
		return (size > k_fixed_plane_count) ? pplanes[i] : planes[i];
	}

	inline void modify(std::uint32_t i, plane_t& p) {
		assert(i < size);
		plane_t* dest = (size > k_fixed_plane_count) ? &pplanes[i] : &planes[i];
		*dest        = p;
	}

	/**
	 * @remarks Construct from a Transpose(view*projection) matrix
	 * @param mat Transpose(View*Projection) or Transpose(Proj)*Transpose(View)
	 * matrix
	 */
	void build(mat4_t const& mat) {
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
		return (size > k_fixed_plane_count) ? pplanes : planes;
	}
	plane_t* get_all() {
		return (size > k_fixed_plane_count) ? pplanes : planes;
	}

	union {
		plane_t planes[k_fixed_plane_count];
		plane_t* pplanes;	
	};
	// if size <= 6, we use planes, otherwise we use planes
	std::uint32_t size;
};
} // namespace vml
