#pragma once

#include "quad.hpp"
#include "vec4.hpp"
#include "quat.hpp"
#include "mat4.hpp"

namespace vml {
struct transform {

	static inline void identity(transform& _);
	static inline transform identity();
	static inline void matrix(transform const& _, mat4& out);
	static inline vec3a_t translation(transform const& _);
	static inline quat_t rotation(transform const& _);
	static inline float scale(transform const& _);
	static inline void set_translation(transform& _, vec3a_t const&);
	static inline void set_rotation(transform& _, quat_t const&);
	static inline void set_scale(transform const& _, float);
	static inline transform world(transform const& parent_world,
	                              transform const& local);
	//! Rotation
	quat_t rotation;
	//! Translation and scale (w = scale)
	union {
		vec4_t translation_and_scale;
		struct {
			vec3_t translation;
			float scale;
		};
	};
};

static_assert(sizeof(transform) == sizeof(vec4_t) * 2, "Fix size");
} // namespace vml
