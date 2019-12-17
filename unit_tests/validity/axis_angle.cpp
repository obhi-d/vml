#include <catch2/catch.hpp>
#include <vml.hpp>

TEST_CASE("Validate axis_angle::set", "[axis_angle::set]") {
	vml::axis_angle_t aa = vml::axis_angle::set(vml::vec3a::set(1.0f, 1.0f, 1.0f), vml::to_radians(10.0f));

	CHECK(vml::axis_angle::x(vml::axis_angle::axis(aa)) ==  Approx(0.57735f));
    CHECK(vml::axis_angle::y(vml::axis_angle::axis(aa)) ==  Approx(0.57735f));
    CHECK(vml::axis_angle::z(vml::axis_angle::axis(aa)) ==  Approx(0.57735f));
    CHECK(vml::real::equals(vml::axis_angle::angle(aa), 0.17453f));
    CHECK(vml::real::equals(vml::axis_angle::x(vml::axis_angle::vangle(aa)), 0.17453f));
}
