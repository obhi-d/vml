#include <catch2/catch.hpp>
#include <vml.hpp>

TEST_CASE("Validate euler_angles::canonize", "[euler_angles::canonize]") {
	vml::euler_angles_t angles = vml::euler_angles::set(
	    vml::to_radians(100), vml::to_radians(720), vml::to_radians(410));
	angles = vml::euler_angles::canonize(angles);
	REQUIRE(vml::to_degrees(vml::vec3::x(angles)) == Approx(80.0f));
	REQUIRE(vml::to_degrees(vml::vec3::y(angles)) == Approx(-180.0f));
	REQUIRE(vml::to_degrees(vml::vec3::z(angles)) == Approx(-130.0f));
}

TEST_CASE("Validate euler_angles::from_quat", "[euler_angles::from_quat]") {
	vml::euler_angles_t angles =
	    vml::euler_angles::from_quat(vml::quat::from_axis_angle(
	        vml::vec3::set(0, 1, 0), vml::to_radians(65.0f)));
	angles = vml::euler_angles::canonize(angles);
	REQUIRE(vml::to_degrees(vml::vec3::x(angles)) == Approx(0.0f));
	REQUIRE(vml::to_degrees(vml::vec3::y(angles)) == Approx(65.0f));
	REQUIRE(vml::to_degrees(vml::vec3::z(angles)) == Approx(0.0f));
}

TEST_CASE("Validate euler_angles::from_quat_conjugate",
          "[euler_angles::from_quat_conjugate]") {
	vml::euler_angles_t angles = vml::euler_angles::from_quat_conjugate(
	    vml::quat::conjugate(vml::quat::from_axis_angle(vml::vec3::set(0, 1, 0),
	                                                    vml::to_radians(65.0f))));
	angles = vml::euler_angles::canonize(angles);
	REQUIRE(vml::to_degrees(vml::vec3::x(angles)) == Approx(0.0f));
	REQUIRE(vml::to_degrees(vml::vec3::y(angles)) == Approx(65.0f));
	REQUIRE(vml::to_degrees(vml::vec3::z(angles)) == Approx(0.0f));
}

TEST_CASE("Validate euler_angles::from_mat4",
          "[euler_angles::from_mat4]") {
    vml::mat4_t mat = vml::mat4::from_rotation(vml::quat::from_axis_angle(vml::vec3::set(0, 1, 0),
	                                                    vml::to_radians(65.0f)));
	vml::euler_angles_t angles = vml::euler_angles::from_mat4(mat);
	angles = vml::euler_angles::canonize(angles);
	REQUIRE(vml::to_degrees(vml::vec3::x(angles)) == Approx(0.0f));
	REQUIRE(vml::to_degrees(vml::vec3::y(angles)) == Approx(65.0f));
	REQUIRE(vml::to_degrees(vml::vec3::z(angles)) == Approx(0.0f));
}

TEST_CASE("Validate euler_angles::from_mat3",
          "[euler_angles::from_mat3]") {
    vml::mat3_t mat = vml::mat3::from_rotation(vml::quat::from_axis_angle(vml::vec3::set(0, 1, 0),
	                                                    vml::to_radians(65.0f)));
	vml::euler_angles_t angles = vml::euler_angles::from_mat3(mat);
	angles = vml::euler_angles::canonize(angles);
	REQUIRE(vml::to_degrees(vml::vec3::x(angles)) == Approx(0.0f));
	REQUIRE(vml::to_degrees(vml::vec3::y(angles)) == Approx(65.0f));
	REQUIRE(vml::to_degrees(vml::vec3::z(angles)) == Approx(0.0f));
}
