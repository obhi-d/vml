#pragma once
#include "mat_base.hpp"
namespace vml {

namespace detail {
struct mat4_traits {

	using type      = types::mat4_t<float>;
	using ref       = type&;
	using pref      = std::conditional_t<types::is_pref_cref, type const&, type>;
	using cref      = type const&;
	using row_type  = types::vec4_t<float>;
	using scalar_type = float;

	enum { element_count = 16 };
	enum { row_count = 4 };
	enum { column_count = 4 };
};
} // namespace detail

struct mat4 : public mat_base<detail::mat4_traits>{
	// @brief Full matrix multiplication
	static inline type Mul(pref m1, pref m2);
	// @brief Transform vertices assuming orthogonal matrix
	static inline void TransformOrtho(pref m, const TraitsVec3::type* iStream,
	                                  uint32 inStride, uint32 count,
	                                  TraitsVec4::type* oStream,
	                                  uint32 outStride);
	// @brief Transform vertices assuming orthogonal matrix
	static inline void TransformOrtho(pref m, const TraitsVec3::type* iStream,
	                                  uint32 inStride, uint32 count,
	                                  TraitsVec3::type* oStream,
	                                  uint32 outStride);
	// @brief Transform vertices in place, assuming orthogonal matrix
	static inline void TransformOrtho(pref m, TraitsVec3::type* ioStream,
	                                  uint32 inStride, uint32 count);
	// @brief Transform vertices and project the w coord as 1.0.
	static inline void Transform(pref m, const TraitsVec3::type* iStream,
	                             uint32 inStride, uint32 count,
	                             TraitsVec3::type* oStream, uint32 outStride);
	// @brief Transform vertices assuming orthogonal matrix
	static inline TraitsVec3A::type TransformOrtho(pref m, TraitsVec3A::pref v);
	// @brief Transform vertices and project the w coord as 1.0.
	static inline TraitsVec3A::type Transform(pref m, TraitsVec3A::pref v);
	// @brief Special transform for AABB bound extends.
	static inline TraitsVec3A::type TransformBounds(pref m,
	                                                TraitsVec3A::pref extends);
	// @brief Special transform for AABB min and max
	static inline TraitsAABox::type TransformAABox(pref m, TraitsAABox::pref v);

	static inline type FromScaleRotPos(float_type scale, TraitsQuat::pref rot,
	                                   TraitsVec3A::pref pos);
	static inline type FromScale(TraitsVec3A::pref scale);
	static inline type FromPos(TraitsVec3A::pref pos);
	// @brief Create a rotation matrix from quaternion
	static inline type FromQuat(TraitsQuat::pref rot);
	// @brief Create a rotation matrix from quaternion
	static inline type FromRot(TraitsQuat::pref rot);
	// @brief Create a view matrix from camera world matrix
	static inline type FromWorldToView(pref m);
	// @brief Mat4::type Creates a camera look at matrix
	static inline type FromLookAt(TraitsVec3A::pref eye, TraitsVec3A::pref lookAt,
	                              TraitsVec3A::pref up);
	// @brief Orthogonal Projection matrix
	static inline type FromOrthoProjection(float_type width, float_type height,
	                                       float_type nearPlane,
	                                       float_type farPlane);
	// @brief Perspective Projection matrix
	static inline type FromPerspectiveProjection(float_type fieldOfView,
	                                             float_type aspectRatio,
	                                             float_type nearPlane,
	                                             float_type farPlane);

	static inline type Mul(pref m, float_type amount);
	static inline type Mul(float_type amount, pref m);
	static inline TraitsVec4::type Mul(TraitsVec4::pref v, pref m);

	static inline type Transpose(pref m);
	static inline type Inverse(pref m);
	// @brief Inverse for orthogonal matrix
	static inline type InverseOrtho(pref m);
};
} // namespace vml
