#include <catch2/catch.hpp>
#include <vml.hpp>

TEST_CASE("Validate mat3::transpose", "[mat3::transpose]") {
	vml::mat3_t m =
	    vml::mat3::from_quat(vml::quat::from_axis_angle(vml::vec3::set(0, 1.0f, 0.0f), vml::to_radians(10.0f)));
    vml::mat3_t t = vml::mat3::transpose(m);
    for(std::uint32_t row = 0; row < 3; ++row)
        for(std::uint32_t col = 0; col < 3; ++col)
            CHECK(vml::mat3::get(m, row, col) == vml::mat3::get(t, col, row));
}

TEST_CASE("Validate mat3::mul", "[mat3::mul]") {
	vml::mat4_t m2 = {
	    3.0f, 10.0f, 12.0f, 18.0f,
        12.0f, 1.0f,  4.0f, 9.0f,
	    9.0f, 10.0f, 12.0f, 2.0f,
        0, 0, 0, 0
	};   

    vml::mat4_t m2_times_3 = {
	    9.0f, 30.0f, 36.0f, 54.0f,
        36.0f, 3.0f,  12.0f, 27.0f,
	    27.0f, 30.0f, 36.0f, 6.0f,
        0, 0, 0, 0
	};

    vml::mat3_t& expected = vml::mat4::as_mat3(m2_times_3);

	CHECK(vml::mat3::equals(vml::mat3::mul(3.0f, vml::mat4::as_mat3(m2)), expected));
    CHECK(vml::mat3::equals(vml::mat3::mul(vml::mat4::as_mat3(m2), 3.0f), expected));
}
