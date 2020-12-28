#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <vml-impl.hpp>
#include <vml.hpp>

TEST_CASE("Validate rect", "[rect]") {
	vml::rect_t r = vml::rect::set(100.0, 0, 200.0, 400.0);
	auto m2        = vml::rect::center(r);
	CHECK(vml::vec2::equals(vml::vec2_t{150, 200}, m2));
	m2 = vml::rect::half_size(r);
	CHECK(vml::vec2::equals(vml::vec2_t{50, 200}, m2));
	m2 = vml::rect::size(r);
	CHECK(vml::vec2::equals(vml::vec2_t{100, 400}, m2));
}
