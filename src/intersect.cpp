#include <intersect.hpp>
#include <vec3a.hpp>

namespace vml {
namespace intersect {
VML_API result_t bounding_volumes(bounding_volume_t const& vol1,
                                  bounding_volume_t const& vol2) {
	vec3a::type d = vec3a::sub(vol1.center(), vol2.center());
	float dist    = vec3a::dot(d, d);
	float rad     = vol1.GetRadius() + vol2.GetRadius();
	if (dist > rad * rad)
		return result_t::k_outside;
	else
		return (vec3a::GreaterAny(vec3a::abs(d),
		                          vec3a::add(vol1.GetExtends(), vol2.GetExtends())))
		           ? result_t::k_outside
		           : result_t::k_outside;
}

VML_API result_t bounding_volume_frustum_coherent(bounding_volume_t const& vol,
                                                  frustum const& frustum,
                                                  std::uint32_t i_mask,
                                                  std::uint32_t& outMask,
                                                  std::uint32_t& o_last_plane) {
	std::uint32_t i, k = 1 << o_last_plane;
	result_t result_t         = result_t::k_inside;
	const plane::type* planes = frustum.GetPlanes();
	outMask                   = 0;
	if (k & i_mask) {
		float m             = plane::dot(planes[o_last_plane], vol.GetExtends());
		vec3a::type absNorm = plane::absNormal(planes[o_last_plane]);
		float n             = vec3a::dot(absNorm, vol.GetExtends());

		if (m + n < 0)
			return result_t::k_outside;
		if (m - n < 0) {
			outMask |= k;
			result_t = result_t::k_intersecting;
		}
	}

	std::uint32_t numPlanes = (std::uint32_t)frustum.Size();
	for (i = 0, k = 1; i < numPlanes; i++, k += k) {
		if ((i != o_last_plane) && (k & i_mask)) {
			float m             = plane::dot(planes[i], vol.GetExtends());
			vec3a::type absNorm = plane::absNormal(planes[i]);
			float n             = vec3a::dot(absNorm, vol.GetExtends());

			if (m + n < 0) {
				o_last_plane = i;
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

VML_API result_t BoundingVolumeFrustum(bounding_volume_t const& vol,
                                       frustum const& frustum) {
	result_t result_t         = result_t::k_inside;
	size_t numPlanes          = frustum.Size();
	const plane::type* planes = frustum.GetPlanes();
	for (size_t i = 0; i < numPlanes; ++i) {
		float m             = plane::dot(planes[i], vol.GetExtends());
		vec3a::type absNorm = plane::absNormal(planes[i]);
		float n             = vec3a::dot(absNorm, vol.GetExtends());

		if (m + n < 0)
			return result_t::k_outside;
		if (m - n < 0)
			result_t = result_t::k_intersecting;
	}
	return result_t;
}

VML_API result_t bounding_sphere_frustum(vec3a::pref center, float radius,
                                         frustum const& frustum) {
	size_t numPlanes          = frustum.Size();
	const plane::type* planes = frustum.GetPlanes();
	for (size_t i = 0; i < numPlanes; ++i) {
		float c = plane::dot(planes[i], center);
		if (c > radius)
			return result_t::k_outside;
		if (c > -radius)
			return result_t::k_intersecting;
	}
	return result_t::k_inside;
}
} // namespace intersect
} // namespace vml