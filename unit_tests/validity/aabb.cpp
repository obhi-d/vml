#include <catch2/catch.hpp>
#include <vml.hpp>

TEST_CASE("Validate aabb::is_valid", "[aabb::is_valid]") {

	REQUIRE(vml::aabb::is_valid(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f))) == true);
	REQUIRE(vml::aabb::is_valid(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(-5.0f, 15.0f, 13.0f))) == false);
	REQUIRE(vml::aabb::is_valid(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, -13.0f))) == false);
	REQUIRE(vml::aabb::is_valid(vml::aabb::set_min_max(
	            vml::vec3a::set(1.0f, 2.0f, 3.0f),
	            vml::vec3a::set(0.1f, 5.0f, 13.0f))) == false);
	REQUIRE(vml::aabb::is_valid(vml::aabb::set_min_max(
	            vml::vec3a::set(1.0f, 2.0f, 3.0f),
	            vml::vec3a::set(-1.0f, 5.0f, 13.0f))) == false);
	REQUIRE(vml::aabb::is_valid(vml::aabb::set_min_max(
	            vml::vec3a::set(1.0f, 2.0f, 3.0f),
	            vml::vec3a::set(1.0f, -5.0f, 13.0f))) == false);
	REQUIRE(vml::aabb::is_valid(vml::aabb::set_min_max(
	            vml::vec3a::set(1.0f, 2.0f, 3.0f),
	            vml::vec3a::set(1.0f, 1.0f, 13.0f))) == false);
	REQUIRE(vml::aabb::is_valid(vml::aabb::set_min_max(
	            vml::vec3a::set(1.0f, 2.0f, 3.0f),
	            vml::vec3a::set(1.0f, 2.0f, 1.0f))) == false);
	REQUIRE(vml::aabb::is_valid(vml::aabb::set_min_max(
	            vml::vec3a::set(1.0f, 2.0f, 3.0f),
	            vml::vec3a::set(1.0f, 2.0f, -8.0f))) == false);
}

TEST_CASE("Validate aabb::center", "[aabb::center]") {

	REQUIRE(vml::vec3a::x(vml::aabb::center(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)))) == 1.0f);
	REQUIRE(vml::vec3a::y(vml::aabb::center(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)))) == 2.0f);
	REQUIRE(vml::vec3a::z(vml::aabb::center(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)))) == 3.0f);
	REQUIRE(vml::vec3a::w(vml::aabb::center(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)))) == 0.0f);
}

TEST_CASE("Validate aabb::size", "[aabb::size]") {

	REQUIRE(vml::vec3a::x(vml::aabb::size(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)))) == 10.0f);
	REQUIRE(vml::vec3a::y(vml::aabb::size(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)))) == 30.0f);
	REQUIRE(vml::vec3a::z(vml::aabb::size(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)))) == 26.0f);
	REQUIRE(vml::vec3a::w(vml::aabb::size(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)))) == 0.0f);
}

TEST_CASE("Validate aabb::half_size", "[aabb::half_size]") {

	REQUIRE(vml::vec3a::x(vml::aabb::half_size(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)))) == 5.0f);
	REQUIRE(vml::vec3a::y(vml::aabb::half_size(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)))) == 15.0f);
	REQUIRE(vml::vec3a::z(vml::aabb::half_size(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)))) == 13.0f);
	REQUIRE(vml::vec3a::w(vml::aabb::half_size(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)))) == 0.0f);
}

TEST_CASE("Validate aabb::corner", "[aabb::corner]") {

	REQUIRE(vml::vec3a::x(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            0)) == -4.0f);
	REQUIRE(vml::vec3a::y(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            0)) == -13.0f);
	REQUIRE(vml::vec3a::z(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            0)) == -10.0f);
	REQUIRE(vml::vec3a::w(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            0)) == 0.0f);

	REQUIRE(vml::vec3a::x(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            1)) == -4.0f);
	REQUIRE(vml::vec3a::y(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            1)) == -13.0f);
	REQUIRE(vml::vec3a::z(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            1)) == 16.0f);
	REQUIRE(vml::vec3a::w(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            1)) == 0.0f);

	REQUIRE(vml::vec3a::x(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            2)) == -4.0f);
	REQUIRE(vml::vec3a::y(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            2)) == 17.0f);
	REQUIRE(vml::vec3a::z(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            2)) == -10.0f);
	REQUIRE(vml::vec3a::w(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            2)) == 0.0f);

	REQUIRE(vml::vec3a::x(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            3)) == -4.0f);
	REQUIRE(vml::vec3a::y(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            3)) == 17.0f);
	REQUIRE(vml::vec3a::z(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            3)) == 16.0f);
	REQUIRE(vml::vec3a::w(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            3)) == 0.0f);

	REQUIRE(vml::vec3a::x(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            4)) == 6.0f);
	REQUIRE(vml::vec3a::y(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            4)) == -13.0f);
	REQUIRE(vml::vec3a::z(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            4)) == -10.0f);
	REQUIRE(vml::vec3a::w(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            4)) == 0.0f);

	REQUIRE(vml::vec3a::x(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            5)) == 6.0f);
	REQUIRE(vml::vec3a::y(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            5)) == -13.0f);
	REQUIRE(vml::vec3a::z(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            5)) == 16.0f);
	REQUIRE(vml::vec3a::w(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            5)) == 0.0f);

	REQUIRE(vml::vec3a::x(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            6)) == 6.0f);
	REQUIRE(vml::vec3a::y(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            6)) == 17.0f);
	REQUIRE(vml::vec3a::z(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            6)) == -10.0f);
	REQUIRE(vml::vec3a::w(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            6)) == 0.0f);

	REQUIRE(vml::vec3a::x(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            7)) == 6.0f);
	REQUIRE(vml::vec3a::y(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            7)) == 17.0f);
	REQUIRE(vml::vec3a::z(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            7)) == 16.0f);
	REQUIRE(vml::vec3a::w(vml::aabb::corner(
	            vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                           vml::vec3a::set(5.0f, 15.0f, 13.0f)),
	            7)) == 0.0f);
}

TEST_CASE("Validate aabb::append", "[aabb::append]") {

	vml::aabb_t aabb1 = vml::aabb::set(vml::vec3a::set(1.0f, 2.0f, 3.0f),
	                                   vml::vec3a::set(5.0f, 15.0f, 13.0f));
	vml::aabb_t aabb2 = vml::aabb::set(vml::vec3a::set(0.0f, 0.0f, -9.0f),
	                                   vml::vec3a::set(1.0f, 5.0f, 10.0f));
	vml::vec3a_t p    = vml::vec3a::set(2.0f, 11.0f, -32.0f);

	REQUIRE(vml::vec3a::x(vml::aabb::center(vml::aabb::append(aabb1, p))) ==
	        1.0f);
	REQUIRE(vml::vec3a::z(vml::aabb::center(vml::aabb::append(aabb1, p))) ==
	        -8.0f);
	REQUIRE(vml::vec3a::x(vml::aabb::size(vml::aabb::append(aabb1, p))) == 10.0f);
	REQUIRE(vml::vec3a::z(vml::aabb::size(vml::aabb::append(aabb1, p))) == 48.0f);

	REQUIRE(vml::vec3a::x(vml::aabb::center(vml::aabb::append(aabb1, aabb2))) ==
	        1.0f);
	REQUIRE(vml::vec3a::y(vml::aabb::center(vml::aabb::append(aabb1, aabb2))) ==
	        2.0f);
	REQUIRE(vml::vec3a::z(vml::aabb::center(vml::aabb::append(aabb1, aabb2))) ==
	        -1.5f);
	REQUIRE(vml::vec3a::x(vml::aabb::size(vml::aabb::append(aabb1, aabb2))) ==
	        10.0f);
	REQUIRE(vml::vec3a::y(vml::aabb::size(vml::aabb::append(aabb1, aabb2))) ==
	        30.0f);
	REQUIRE(vml::vec3a::z(vml::aabb::size(vml::aabb::append(aabb1, aabb2))) ==
	        35.0f);
}
