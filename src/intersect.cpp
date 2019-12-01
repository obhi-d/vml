#include <intersect.hpp>

namespace vml {
namespace intersect {
VML_API result_t BoundingVolumes(const BoundingVolume& vol1,
                                 const BoundingVolume& vol2) {
	Vec3A::type d = Vec3A::Sub(vol1.GetCenter(), vol2.GetCenter());
	float dist    = Vec3A::Dot(d, d);
	float rad     = vol1.GetRadius() + vol2.GetRadius();
	if (dist > rad * rad)
		return result_t::k_outside;
	else
		return (Vec3A::GreaterAny(Vec3A::Abs(d),
		                          Vec3A::Add(vol1.GetExtends(), vol2.GetExtends())))
		           ? result_t::k_outside
		           : result_t::k_outside;
}

VML_API result_t BoundingVolumeFrustumCoherent(const BoundingVolume& vol,
                                               const Frustum& frustum,
                                               uint32 inMask, uint32& outMask,
                                               uint32& lastPlane) {
	uint32 i, k = 1 << lastPlane;
	result_t result_t         = result_t::k_inside;
	const Plane::type* planes = frustum.GetPlanes();
	outMask                   = 0;
	if (k & inMask) {
		float m             = Plane::Dot(planes[lastPlane], vol.GetExtends());
		Vec3A::type absNorm = Plane::AbsNormal(planes[lastPlane]);
		float n             = Vec3A::Dot(absNorm, vol.GetExtends());

		if (m + n < 0)
			return result_t::k_outside;
		if (m - n < 0) {
			outMask |= k;
			result_t = result_t::k_intersecting;
		}
	}

	uint32 numPlanes = (uint32)frustum.Size();
	for (i = 0, k = 1; i < numPlanes; i++, k += k) {
		if ((i != lastPlane) && (k & inMask)) {
			float m             = Plane::Dot(planes[i], vol.GetExtends());
			Vec3A::type absNorm = Plane::AbsNormal(planes[i]);
			float n             = Vec3A::Dot(absNorm, vol.GetExtends());

			if (m + n < 0) {
				lastPlane = i;
				return result_t::k_outside;
			}
			if (m - n < 0) {
				outMask |= k;
				result_t = result_t::k_intersecting;
			}
		}
	}
	return result_t;
}

VML_API result_t BoundingVolumeFrustum(const BoundingVolume& vol,
                                       const Frustum& frustum) {
	result_t result_t         = result_t::k_inside;
	size_t numPlanes          = frustum.Size();
	const Plane::type* planes = frustum.GetPlanes();
	for (size_t i = 0; i < numPlanes; ++i) {
		float m             = Plane::Dot(planes[i], vol.GetExtends());
		Vec3A::type absNorm = Plane::AbsNormal(planes[i]);
		float n             = Vec3A::Dot(absNorm, vol.GetExtends());

		if (m + n < 0)
			return result_t::k_outside;
		if (m - n < 0)
			result_t = result_t::k_intersecting;
	}
	return result_t;
}

VML_API result_t BoundingSphereFrustum(Vec3A::pref center, float radius,
                                       const Frustum& frustum) {
	size_t numPlanes          = frustum.Size();
	const Plane::type* planes = frustum.GetPlanes();
	for (size_t i = 0; i < numPlanes; ++i) {
		float c = Plane::Dot(planes[i], center);
		if (c > radius)
			return result_t::k_outside;
		if (c > -radius)
			return result_t::k_intersecting;
	}
	return result_t::k_inside;
}
} // namespace intersect
} // namespace vml