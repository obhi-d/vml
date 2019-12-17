#include <catch2/catch.hpp>
#include <vml.hpp>
#include <cstring>

TEST_CASE("Validate intersect::bounding_volume_frustum_coherent",
          "[intersect::bounding_volume_frustum_coherent]") {
	// define a custom prism
	vml::mat4_t m = vml::mat4::from_orthographic_projection(-50.0f, 50.0f, -45.0f,
	                                                        45.0f, 1.0f, 1000.0f);
	vml::frustum_t frustum_orig =
	    vml::frustum::from_mat4_transpose(vml::mat4::transpose(m));
	auto planes = vml::frustum::get_planes(frustum_orig);
	std::array<vml::plane_t, 8> custom_planes;
	std::memcpy(custom_planes.data(), planes.first,
	            planes.second * sizeof(vml::plane_t));
	custom_planes[6] = vml::plane::set(
	    vml::frustum::get_plane(frustum_orig, vml::frustum::plane_type::k_far),
	    900.0f);
	
	custom_planes[7] = vml::plane::set(
	    vml::frustum::get_plane(frustum_orig, vml::frustum::plane_type::k_near),
	    -10.0f);

	vml::frustum_t custom = vml::frustum::from_planes(
	    nullptr, static_cast<std::uint32_t>(custom_planes.size()));
	vml::frustum_t  unused = vml::frustum::from_planes(
	    custom_planes.data(), 6);

	vml::frustum_t unused_copy(unused);
	CHECK(vml::frustum::count(unused_copy) == 6);

	vml::mat4_t tm = vml::mat4::transpose(m);
	vml::mat4::transpose_in_place(m);
	CHECK(vml::mat4::equals(m, tm));
	
	vml::frustum::set(unused, m);
	std::uint32_t idx = 0;
	for(auto& p : custom_planes)
		vml::frustum::set_plane(custom, idx++, p);

	vml::frustum_t copy(custom);
	copy = custom;
	vml::frustum_t inter(std::move(copy));
	custom = std::move(inter);

	vml::frustum::coherency state = vml::frustum::default_coherency(6);

	vml::bounding_volume_t vol = vml::bounding_volume::from_box(
	    vml::vec3a::set(5, 5, 5), vml::vec3a::set(2, 2, 2));
	vml::bounding_volume_t vol2 = vml::bounding_volume::from_box(
	    vml::vec3a::set(25, 1225, 25), vml::vec3a::set(2, 2, 2));

	CHECK(vml::intersect::bounding_volume_frustum_coherent(
	          vol, frustum_orig, state) == vml::intersect::result_t::k_inside);
	
	state = vml::frustum::default_coherency(8);

	CHECK(vml::intersect::bounding_volume_frustum_coherent(vol2, custom, state) ==
	      vml::intersect::result_t::k_outside);
#ifndef NDEBUG
	CHECK(vml::intersect::bounding_volume_frustum_coherent(vol2, custom, state) ==
	      vml::intersect::result_t::k_outside);
	CHECK(state.iterations == 0);
#endif

	vol = vml::bounding_volume::from_box(vml::vec3a::set(5, 5, 5),
	                                     vml::vec3a::set(20, 20, 20));

	CHECK(vml::intersect::bounding_volume_frustum_coherent(vol, custom, state) ==
	      vml::intersect::result_t::k_intersecting);
}

TEST_CASE("Validate intersect::bounding_volume_frustum",
          "[intersect::bounding_volume_frustum]") {
	// define a custom prism
	vml::mat4_t m = vml::mat4::from_orthographic_projection(-50.0f, 50.0f, -45.0f,
	                                                        45.0f, 1.0f, 1000.0f);
	vml::frustum_t frustum_orig =
	    vml::frustum::from_mat4_transpose(vml::mat4::transpose(m));
	auto planes = vml::frustum::get_planes(frustum_orig);
	std::array<vml::plane_t, 8> custom_planes;
	std::memcpy(custom_planes.data(), planes.first,
	            planes.second * sizeof(vml::plane_t));
	custom_planes[6] = vml::plane::set(
	    vml::frustum::get_plane(frustum_orig, vml::frustum::plane_type::k_far),
	    900.0f);
	custom_planes[7] = vml::plane::set(
	    vml::frustum::get_plane(frustum_orig, vml::frustum::plane_type::k_near),
	    -10.0f);

	vml::frustum_t custom = vml::frustum::from_planes(
	    custom_planes.data(), static_cast<std::uint32_t>(custom_planes.size()));

	vml::bounding_volume_t vol = vml::bounding_volume::from_box(
	    vml::vec3a::set(5, 5, 5), vml::vec3a::set(2, 2, 2));

	CHECK(vml::intersect::bounding_volume_frustum(vol, frustum_orig) ==
	      vml::intersect::result_t::k_inside);
	CHECK(vml::intersect::bounding_volume_frustum(vol, custom) ==
	      vml::intersect::result_t::k_outside);

	vol = vml::bounding_volume::from_box(vml::vec3a::set(5, 5, 5),
	                                     vml::vec3a::set(20, 20, 20));

	CHECK(vml::intersect::bounding_volume_frustum(vol, custom) ==
	      vml::intersect::result_t::k_intersecting);
}

TEST_CASE("Validate intersect::bounding_volumes",
          "[intersect::bounding_volumes]") {
	// define a custom prism
	vml::bounding_volume_t vol1 = vml::bounding_volume::from_box(
	    vml::vec3a::set(5, 5, 5), vml::vec3a::set(12, 12, 12));
	vml::bounding_volume_t vol2 = vml::bounding_volume::from_box(
	    vml::vec3a::set(15, 15, 15), vml::vec3a::set(12, 12, 12));
	vml::bounding_volume_t vol3 = vml::bounding_volume::from_box(
	    vml::vec3a::set(19, 19, 19), vml::vec3a::set(1, 1, 1));

	CHECK(vml::intersect::bounding_volumes(vol1, vol2) ==
	      vml::intersect::result_t::k_intersecting);
	CHECK(vml::intersect::bounding_volumes(vol1, vol3) ==
	      vml::intersect::result_t::k_outside);
	CHECK(vml::intersect::bounding_volumes(vol2, vol3) ==
	      vml::intersect::result_t::k_intersecting);
}

TEST_CASE("Validate intersect::bounding_sphere_frustum",
          "[intersect::bounding_sphere_frustum]") {
	// define a custom prism
	vml::mat4_t m = vml::mat4::from_orthographic_projection(-50.0f, 50.0f, -45.0f,
	                                                        45.0f, 1.0f, 1000.0f);
	vml::frustum_t frustum_orig =
	    vml::frustum::from_mat4_transpose(vml::mat4::transpose(m));
	auto planes = vml::frustum::get_planes(frustum_orig);
	std::array<vml::plane_t, 8> custom_planes;
	std::memcpy(custom_planes.data(), planes.first,
	            planes.second * sizeof(vml::plane_t));
	custom_planes[6] = vml::plane::set(
	    vml::frustum::get_plane(frustum_orig, vml::frustum::plane_type::k_far),
	    900.0f);
	custom_planes[7] = vml::plane::set(
	    vml::frustum::get_plane(frustum_orig, vml::frustum::plane_type::k_near),
	    -10.0f);

	vml::frustum_t custom = vml::frustum::from_planes(
	    custom_planes.data(), static_cast<std::uint32_t>(custom_planes.size()));

	vml::sphere_t vol = vml::sphere::set(vml::vec3a::set(5, 5, 5), 2);

	CHECK(vml::intersect::bounding_sphere_frustum(vol, frustum_orig) ==
	      vml::intersect::result_t::k_inside);
	CHECK(vml::intersect::bounding_sphere_frustum(vol, custom) ==
	      vml::intersect::result_t::k_outside);

	vol = vml::sphere::set(vml::vec3a::set(5, 5, 5), 20);

	CHECK(vml::intersect::bounding_sphere_frustum(vol, custom) ==
	      vml::intersect::result_t::k_intersecting);
}
