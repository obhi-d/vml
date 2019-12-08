#pragma once
#include "quad.hpp"
#include "vec3a.hpp"

namespace vml {
struct axis_angle : public quad {
    static inline vec3a_t get_axis(quad::pref q) {
        return vec3a::from_vec4(q);
    }
    static inline float get_angle(quad::pref q) {
        return quad::w(q);
    }

};
} // namespace vml
