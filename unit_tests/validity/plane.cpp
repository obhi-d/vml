#include <catch2/catch.hpp>
#include <vml.hpp>

TEST_CASE("Validate plane::abs_normal", "[plane::abs_normal]") {
	vml::plane_t p = vml::plane::set(-1.0f, 1.0f, -1.0f, 10.0f);
	vml::vec3a_t n = vml::plane::abs_normal(p);
	CHECK(vml::plane::x(vml::plane::vdot(p, vml::vec3a::set(1.0f, 1.0f, 1.0f))) ==
	      Approx(9.0));
	CHECK(vml::plane::x(n) == Approx(1.0));
	CHECK(vml::plane::y(n) == Approx(1.0));
	CHECK(vml::plane::z(n) == Approx(1.0));
}
