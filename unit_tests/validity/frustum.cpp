#include <catch2/catch.hpp>
#include <vml.hpp>

TEST_CASE("Validate frustum::set", "[frustum::set]") {
    vml::mat4_t m = vml::mat4::from_orthographic_projection(100.0f, 90.0f, 1.0f, 1000.0f);
    vml::frustum_t frustum = vml::frustum::from_mat4_transpose(vml::mat4::transpose(m));
    
    CHECK(vml::plane::dot(vml::frustum::get_plane(frustum, vml::frustum::plane_type::k_near), vml::vec3a::set(0.0f, 0.0f, 0.0f)) == Approx(1.0f));
    CHECK(vml::plane::dot(vml::frustum::get_plane(frustum, vml::frustum::plane_type::k_far), vml::vec3a::set(0.0f, 0.0f, 0.0f)) == Approx(-1000.0f));
    CHECK(vml::plane::dot(vml::frustum::get_plane(frustum, vml::frustum::plane_type::k_left), vml::vec3a::set(0.0f, 0.0f, 0.0f)) == Approx(-50.0f));
    CHECK(vml::plane::dot(vml::frustum::get_plane(frustum, vml::frustum::plane_type::k_right), vml::vec3a::set(0.0f, 0.0f, 0.0f)) == Approx(-50.0f));
    CHECK(vml::plane::dot(vml::frustum::get_plane(frustum, vml::frustum::plane_type::k_top), vml::vec3a::set(0.0f, 0.0f, 0.0f)) == Approx(-45.0f));
    CHECK(vml::plane::dot(vml::frustum::get_plane(frustum, vml::frustum::plane_type::k_bottom), vml::vec3a::set(0.0f, 0.0f, 0.0f)) == Approx(-45.0f));
}
