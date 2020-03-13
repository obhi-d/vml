#include <catch2/catch.hpp>
#include <vml.hpp>
#include <cmath>
#include <limits>

TEST_CASE("Validate vec3::cross", "[vec3::cross]") {
    vml::vec3_t v1 = vml::vec3::set(4, 6, 1);
    vml::vec3_t v2 = vml::vec3::set(3, 8, 5);
    vml::vec3_t expected = vml::vec3::set(22, -17, 14);
    CHECK(vml::vec3::equals(expected, vml::vec3::cross(v1, v2)));
}

TEST_CASE("Validate vec3a::compare", "[vec3a::compare]") {
    vml::vec3a_t p = vml::vec3a::set(-441.3f, 23.0f, -1.0f);
    vml::vec3a_t q = vml::vec3a::set(441.3f, 5.0f, 51.0f);
    vml::vec3a_t r = vml::vec3a::set(445.3f, 15.0f, 151.0f);
    CHECK(vml::vec3a::greater_any(p, q));
    CHECK(vml::vec3a::greater_all(p, q) == false);
    CHECK(vml::vec3a::lesser_any(p, q));
    CHECK(vml::vec3a::lesser_all(p, q) == false);
    CHECK(vml::vec3a::greater_any(q, r) == false);
    CHECK(vml::vec3a::greater_all(q, r) == false);
    CHECK(vml::vec3a::lesser_any(q, r));
    CHECK(vml::vec3a::lesser_all(q, r));
    CHECK(vml::vec3a::greater_any(r, q));
    CHECK(vml::vec3a::greater_all(r, q));
    CHECK(vml::vec3a::lesser_any(r, q) == false);
    CHECK(vml::vec3a::lesser_all(r, q) == false);
}

TEST_CASE("Validate vec4::mul", "[vec4::mul]") {
  vml::mat4_t m = {
	    0.0f,   0.80f, 0.60f,  0.0f, -0.80f, -0.36f, 0.48f, 0.0f,
	    -0.60f, 0.48f, -0.64f, 0.0f, 0.0f,   0.0f,   0.0f,  1.0f,
	};
    vml::vec4_t v = vml::vec4::set(3.000f, 10.000f, 12.000f, 1.0f);
    vml::vec4_t expected = vml::vec4::set(-15.2f, 4.56f, -1.08f, 1.0f);
    CHECK(vml::vec4::equals(expected, vml::vec4::mul(v, m)));
}

TEST_CASE("Validate vec3a::mul", "[vec3a::mul]") {
    vml::mat4_t m = {
	    0.0f,   0.80f, 0.60f,  0.0f, -0.80f, -0.36f, 0.48f, 0.0f,
	    -0.60f, 0.48f, -0.64f, 0.0f, 0.0f,   0.0f,   0.0f,  1.0f,
	};
    vml::vec3a_t v = vml::vec3a::set(3.000f, 10.000f, 12.000f);
    vml::vec3a_t expected = vml::vec3a::set(-15.2f, 4.56f, -1.08f);
    CHECK(vml::vec3a::equals(expected, vml::vec3a::mul(v, m)));
}
