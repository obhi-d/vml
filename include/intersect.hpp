#pragma once
#include "bounding_volume.hpp"
#include "frustum.hpp"
#include "sphere.hpp"

namespace vml {
namespace intersect {
enum class result_t : std::uint32_t { k_outside = 0, k_inside, k_intersecting };

/** @remarks Test bounding volume bounding volume */
inline result_t bounding_volumes(bounding_volume_t const& i_vol1,
                                 bounding_volume_t const& i_vol2);

/**
 * @remarks Test bounding volume frustum_t intersection using coherency
 *          and masking.
 */
VML_API result_t bounding_volume_frustum_coherent(
    bounding_volume_t const& i_vol, frustum_t const& i_frustum,
    frustum_t::coherency& io_coherency);

/** @remarks Test bounding volume frustum_t intersection */
VML_API result_t bounding_volume_frustum(bounding_volume_t const& i_vol,
                                        frustum_t const& i_frustum);

/** @remarks Intersect sphere with frustum_t */
VML_API result_t bounding_sphere_frustum(sphere::pref i_sphere,
                                        frustum_t const& i_frustum);

inline result_t bounding_volumes(bounding_volume_t const& vol1,
                                 bounding_volume_t const& vol2) {

	vec3a::type d =
	    vec4::sub(vol1.spherical_vol, quad::negate_w(vol2.spherical_vol));
	if (quad::hadd(quad::negate_w(vec4::mul(d, d))) > 0.0f)
		return result_t::k_outside;
	else
		return (vec3a::greater_any(
		           vec3a::abs(vec3a::from_vec4(d)),
		           vec3a::add(vol1.half_extends, vol2.half_extends)))
		           ? result_t::k_outside
		           : result_t::k_intersecting;
}

} // namespace intersect
} // namespace vml

