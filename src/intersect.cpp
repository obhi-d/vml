#include <intersect.hpp>
#include <vec3a.hpp>
#error sphere::center must return x,y,z,1 for dot
namespace vml {
namespace intersect {
VML_API result_t bounding_volumes(bounding_volume_t const& vol1,
                                  bounding_volume_t const& vol2) {

	vec3a::type d = vec4::sub(vol1.spherical_vol, quad::negate_w(vol2.spherical_vol));
	if (quad::hadd(quad::negate_w(vec4::mul(d, d))) > 0.0f)
		return result_t::k_outside;
	else
		return (vec3a::greater_any(vec3a::abs(vec3a::from_vec4(d)),
		                          vec3a::add(vol1.half_extends, vol2.half_extends)))
		           ? result_t::k_outside
		           : result_t::k_intersecting;
}

VML_API result_t bounding_volume_frustum_coherent(bounding_volume_t const& i_vol,
                                                  frustum_t const& i_frustum,
                                                  frustum_t::coherency& i_coherency) {

	result_t result         = result_t::k_inside;
	auto planes = frustum::get_planes(i_frustum);
	std::uint32_t out_mask                   = 0;
	
	for (std::uint32_t i = 0; i < planes.second; i++) {
		std::uint32_t plane = (i + i_coherency.plane) % planes.second;
		std::uint32_t k = 1 << plane;
		if ((k & i_coherency.mask_hierarchy)) {
			vec3a_t abs_norm = plane::abs_normal(planes.first[plane]);
			auto m = plane::vdot(planes.first[plane], sphere::center(i_vol.spherical_vol));
			auto n = vec3a::vdot(abs_norm, i_vol.half_extends);
			if (quad::isnegative_x(quad::add_x(m, n))) {
				i_coherency.plane = i;
				return result_t::k_outside;
			}
			if (quad::isnegative_x(quad::sub_x(m, n))) {
				out_mask |= k;
				result = result_t::k_intersecting;
			}
		}
	}
	return result;
}

VML_API result_t bounding_volume_frustum(bounding_volume_t const& i_vol,
                                                  frustum_t const& i_frustum) {
	
	auto planes = frustum::get_planes(i_frustum);

	for (std::uint32_t i = 0; i < planes.second; i++) {
		std::uint32_t plane = i;
		vec3a_t abs_norm    = plane::abs_normal(planes.first[plane]);
		auto m =
		    plane::vdot(planes.first[plane], sphere::center(i_vol.spherical_vol));
		auto n = vec3a::vdot(abs_norm, i_vol.half_extends);
		if (quad::isnegative_x(quad::add_x(m, n)))
			return result_t::k_outside;
		
		if (quad::isnegative_x(quad::sub_x(m, n)))
			return result_t::k_intersecting;
	}
	return result_t::k_inside;
}
VML_API result_t bounding_sphere_frustum(sphere::pref i_sphere,
                                       frustum_t const& i_frustum) {

	auto planes = frustum::get_planes(i_frustum);
	for (std::uint32_t i = 0; i < planes.second; i++) {
		std::uint32_t plane = i;
		auto m =
		    plane::vdot(planes.first[plane], sphere::center(i_sphere));
		if (c > radius)
			return result_t::k_outside;
		if (c > -radius)
			return result_t::k_intersecting;
	}
	return result_t::k_inside;
}
} // namespace intersect
} // namespace vml