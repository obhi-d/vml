#pragma once
#include "bounding_volume.hpp"
#include "frustum.hpp"


namespace vml {
namespace intersect {
enum result_t : std::uint32_t { k_outside = 0, k_inside, k_intersecting };

/**
 * @remarks Test bounding volume frustum_t intersection using coherency
 *          and masking.
 */
VML_API result_t bounding_volume_frustum_coherent(bounding_volume_t const& vol,
                                               frustum_t const& frustum,
                                               frustum_t::coherency& io_coherency);

/** @remarks Test bounding volume frustum_t intersection */
VML_API result_t bounding_volume_frustum(bounding_volume_t const& vol,
                                       frustum_t const& frustum);

/** @remarks Test bounding volume bounding volume */
VML_API result_t bounding_volumes(bounding_volume_t const& vol1,
                                 bounding_volume_t const& vol2);

/** @remarks Intersect sphere with frustum_t */
VML_API result_t bounding_sphere_frustum(vec3a::pref center, float radius,
                                       frustum_t const& frustum);
} // namespace intersect
} // namespace vml

#endif /* LUMIERE_INTERSECTIONS_H */
