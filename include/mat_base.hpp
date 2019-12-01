#pragma once
#include "multi_dim.hpp"
namespace vml {
template <typename concrete> struct mat_base : public multi_dim<concrete> {
	// @brief Create a matrix from vector mapping that
	// can rotate the vector axis1 to axis2 when post multiplied to axis1.
	static inline type FromVectorMapping(TraitsVec3::pref v1,
	                                     TraitsVec3::pref v2);
	// @brief rotate vector in place
	static inline void Rotate(pref m, TraitsVec3::type* ioStream, uint32 inStride,
	                          uint32 count);
	// @brief rotate vector
	static inline TraitsVec3A::type Rotate(pref m, TraitsVec3A::pref v);
	// @brief Set a rotation for a given matrix
	static inline void SetRot(ref m, TraitsQuat::pref rot);
	// @brief Returns a look at matrix based on a look at vector and up position
	static inline void SetViewAndUp(ref m, TraitsVec3A::pref viewDir,
	                                TraitsVec3A::pref up);
};
} // namespace vml
