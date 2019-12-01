
#include "quad.hpp"

namespace vml {

struct vec4 : public quad {
	using quad::scalar_type;
	using quad::cref;
	using quad::pref;
	using quad::ref;
	using quad::row_type;
	using quad::scalar_type;
	using quad::type;

	using quad::mul;

	static inline type mul(pref q1, mat4_t const& m);
};


inline vec4::type vec4::mul(
    pref v, mat4_t const& m) {
#if L_VECTOR_MATH_TYPE_IS_SSE
	V_Quad ret, vTemp;
	ret   = _mm_shuffle_ps(v, v, _MM_SHUFFLE(0, 0, 0, 0));
	ret   = _mm_mul_ps(ret, m.r[0]);
	vTemp = _mm_shuffle_ps(v, v, _MM_SHUFFLE(1, 1, 1, 1));
	vTemp = _mm_mul_ps(vTemp, m.r[1]);
	ret   = _mm_add_ps(ret, vTemp);
	vTemp = _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 2, 2, 2));
	vTemp = _mm_mul_ps(vTemp, m.r[2]);
	ret   = _mm_add_ps(ret, vTemp);
	vTemp = _mm_shuffle_ps(v, v, _MM_SHUFFLE(3, 3, 3, 3));
	vTemp = _mm_mul_ps(vTemp, m.r[3]);
	ret   = _mm_add_ps(ret, vTemp);
	return ret;
#else
	V_Quad r, x, y, z, w;

	z = SplatZ(v);
	y = SplatY(v);
	x = SplatX(v);
	w = SplatW(v);

	r = Mul(w, m.r[3]);
	r = Madd(z, m.r[2], r);
	r = Madd(y, m.r[1], r);
	r = Madd(x, m.r[0], r);

	return r;
#endif
}
}