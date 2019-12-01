#pragma once

#include "mat4.hpp"
#include "plane.hpp"

namespace vml {
struct frustum {
	// common frustums will have these planes
	enum plane_type {
		k_near,
		k_far,
		k_left,
		k_right,
		k_top,
		k_bottom,
	};

	enum {
		k_fixed_plane_count = 6
	};

	Frustum() : size(0) {}
	Frustum(const Frustum& iOther) : size(iOther.size) {
		if (size < kFixedPlaneCount && size > 0) {
			for (uint32 i = 0; i < size; ++i)
				frustum[i] = iOther.frustum[i];
		} else {
			planes = new vPlane[size];
			for (uint32 i = 0; i < size; ++i)
				planes[i] = iOther.planes[i];
		}
	}
	Frustum(Frustum&& iOther) : size(iOther.size) {
		if (size < kFixedPlaneCount && size > 0) {
			for (uint32 i = 0; i < size; ++i)
				frustum[i] = iOther.frustum[i];
		} else {
			planes        = iOther.planes;
			iOther.planes = nullptr;
		}
	}
	Frustum(const vPlane* iPlanes, uint32 iSize) : size(iSize) {
		if (size < kFixedPlaneCount && size > 0) {
			for (uint32 i = 0; i < size; ++i)
				frustum[i] = iPlanes[i];
		} else {
			planes = new vPlane[size];
			for (uint32 i = 0; i < size; ++i)
				planes[i] = iPlanes[i];
		}
	}
	~Frustum();

	inline Frustum& operator=(const Frustum& iOther) {
		if (size > kFixedPlaneCount && planes)
			delete[] planes;

		size = iOther.size;
		if (size < kFixedPlaneCount && size > 0) {
			for (uint32 i = 0; i < size; ++i)
				frustum[i] = iOther.frustum[i];
		} else {
			planes = new vPlane[size];
			for (uint32 i = 0; i < size; ++i)
				planes[i] = iOther.planes[i];
		}
		return *this;
	}

	inline Frustum& operator=(Frustum&& iOther) {
		if (size > kFixedPlaneCount && planes)
			delete[] planes;
		size = iOther.size;
		if (size < kFixedPlaneCount && size > 0) {
			for (uint32 i = 0; i < size; ++i)
				frustum[i] = iOther.frustum[i];
		} else {
			planes        = iOther.planes;
			iOther.planes = nullptr;
		}
		return *this;
	}

	inline uint32 Size() const { return size; }

	inline vPlane GetPlane(size_t i) const {
		L_ASSERT(i < size);
		return (size > kFixedPlaneCount) ? frustum[i] : planes[i];
	}

	inline void SetPlane(const vPlane& iPlane, size_t i) {
		L_ASSERT(i < size);
		vPlane* dest = (size > kFixedPlaneCount) ? &frustum[i] : &planes[i];
		*dest        = iPlane;
	}

	/**
	 * @remarks Construct from a Transpose(view*projection) matrix
	 * @param mat Transpose(View*Projection) or Transpose(Proj)*Transpose(View)
	 * matrix
	 */
	void ConstructFrom(const vMat4& mat);

	const vPlane* GetPlanes() const {
		return (size > kFixedPlaneCount) ? frustum : planes;
	}
	vPlane* GetPlanes() { return (size > kFixedPlaneCount) ? frustum : planes; }

	transform const& local);
	union {
		plane_t planes[k_fixed_plane_count];
		plane_t* ex_planes;
	};
	// if size <= 6, we use frustum, otherwise we use planes
	uint32 size;
};
} // namespace vml
