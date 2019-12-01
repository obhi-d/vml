#pragma once
#include <bounding_volume.h>

namespace vml {
namespace intersect {
enum result_t : uint32 { k_outside = 0, k_inside, k_intersecting };

/**
 * @remarks Test bounding volume frustum intersection using coherency
 *          and masking.
 */
VML_API result_t BoundingVolumeFrustumCoherent(const BoundingVolume& vol,
                                               const Frustum& frustum,
                                               uint32 inMask, uint32& newMask,
                                               uint32& lastPlane);

/** @remarks Test bounding volume frustum intersection */
VML_API result_t BoundingVolumeFrustum(const BoundingVolume& vol,
                                       const Frustum& frustum);

/** @remarks Test bounding volume bounding volume */
VML_API result_t BoundingVolumes(const BoundingVolume& vol1,
                                 const BoundingVolume& vol2);

/** @remarks Intersect sphere with frustum */
VML_API result_t BoundingSphereFrustum(Vec3A::pref center, float radius,
                                       const Frustum& frustum);
} // namespace intersect
} // namespace vml

#endif /* LUMIERE_INTERSECTIONS_H */
