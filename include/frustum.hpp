#pragma once

#include "mat4.hpp"
#include "plane.hpp"

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
			planes = vml::allocate(sizeof(vml::plane_t)*size, alignof(vml::plane_t));
			for (std::uint32_t i = 0; i < size; ++i)
				planes[i] = i_other.planes[i];
		}
	}
	frustum_t(frustum_t&& i_other) : size(i_other.size) {
		if (size < k_fixed_plane_count && size > 0) {
			for (std::uint32_t i = 0; i < size; ++i)
				frustum[i] = i_other.frustum[i];
		} else {
			planes        = i_other.planes;
			i_other.planes = nullptr;
		}
	}
	frustum_t(const plane_t* i_planes, std::uint32_t iSize) : size(iSize) {
		if (size < k_fixed_plane_count && size > 0) {
			for (std::uint32_t i = 0; i < size; ++i)
				frustum[i] = i_planes[i];
		} else {
			planes = new plane_t[size];
			for (std::uint32_t i = 0; i < size; ++i)
				planes[i] = i_planes[i];
		}
	}
	~frustum_t();

	inline frustum_t& operator=(frustum_tconst& i_other) {
		if (size > k_fixed_plane_count && planes)
			delete[] planes;

		size = i_other.size;
		if (size < k_fixed_plane_count && size > 0) {
			for (std::uint32_t i = 0; i < size; ++i)
				frustum[i] = i_other.frustum[i];
		} else {
			planes = new plane_t[size];
			for (std::uint32_t i = 0; i < size; ++i)
				planes[i] = i_other.planes[i];
		}
		return *this;
	}

	inline frustum_t& operator=(frustum_t&& i_other) {
		if (size > k_fixed_plane_count && planes)
			delete[] planes;
		size = i_other.size;
		if (size < k_fixed_plane_count && size > 0) {
			for (std::uint32_t i = 0; i < size; ++i)
				frustum[i] = i_other.frustum[i];
		} else {
			planes        = i_other.planes;
			i_other.planes = nullptr;
		}
		return *this;
	}

	inline std::uint32_t Size() const { return size; }

	inline plane_t GetPlane(size_t i) const {
		L_ASSERT(i < size);
		return (size > k_fixed_plane_count) ? frustum[i] : planes[i];
	}

	inline void SetPlane(const plane_t& iPlane, size_t i) {
		L_ASSERT(i < size);
		plane_t* dest = (size > k_fixed_plane_count) ? &frustum[i] : &planes[i];
		*dest        = iPlane;
	}

	/**
	 * @remarks Construct from a Transpose(view*projection) matrix
	 * @param mat Transpose(View*Projection) or Transpose(Proj)*Transpose(View)
	 * matrix
	 */
	void ConstructFrom(const vMat4& mat);

	const plane_t* GetPlanes() const {
		return (size > k_fixed_plane_count) ? frustum_t : planes;
	}
	plane_t* GetPlanes() { return (size > k_fixed_plane_count) ? frustum_t : planes; }

	union {
		plane_t planes[k_fixed_plane_count];
		plane_t* ex_planes;	
	};
	// if size <= 6, we use frustum, otherwise we use planes
	std::uint32_t size;
};
} // namespace vml
