#include <catch2/catch.hpp>
#include <vml.hpp>
#include <cmath>
#include <limits>

TEST_CASE("Validate quad::isnan", "[quad::isnan]") {
    float val[4] = {std::numeric_limits<float>::quiet_NaN(), 1.0f, -1.0f, 10.0f };
	vml::quad_t p = vml::quad::set_unaligned(val);
    CHECK(vml::quad::isnan(p) == true);
    auto res = vml::quad::isnanv(p);
    CHECK(vml::quad::x(res) != 0.0f);
}

TEST_CASE("Validate quad::isinf", "[quad::isinf]") {
	vml::quad_t p = vml::quad::set(std::numeric_limits<float>::infinity(), 1.0f, -1.0f, 10.0f);
    CHECK(vml::quad::isinf(p) == true);
    CHECK(vml::quad::x(vml::quad::isinfv(p)) != 0.0f);
	p = vml::quad::set(1.0f, -std::numeric_limits<float>::infinity(), -1.0f, 10.0f);
    CHECK(vml::quad::isinf(p) == true);
    CHECK(vml::quad::x(vml::quad::isinfv(p)) == 0.0f);
}

TEST_CASE("Validate quad::isgreater_x", "[quad::isgreater_x]") {
	vml::quad_t p = vml::quad::set(-441.3f, 1.0f, -1.0f, 10.0f);
    vml::quad_t q = vml::quad::set(reinterpret_cast<float const*>(&p));
	CHECK(vml::quad::isgreater_x(p, q) == false);
    q = vml::quad::set(441.3f, 1.0f, -1.0f, 10.0f);
    CHECK(vml::quad::isgreater_x(p, q) == false);
    q = vml::quad::set(-1441.3f, 1.0f, -1.0f, 10.0f);
    CHECK(vml::quad::isgreater_x(p, q) == true);
}

TEST_CASE("Validate quad::set", "[quad::set]") {
    vml::quad_t p = vml::quad::set_x(41.3f);
    CHECK(vml::quad::get(p, 0) == Approx(41.3f));
    CHECK(vml::quad::get(p, 1) == Approx(0.0f));
    p = vml::quad::set_x(p, 141.3f);
    CHECK(vml::quad::get(p, 0) == Approx(141.3f));
    CHECK(vml::quad::get(p, 1) == Approx(0.0f));
    p = vml::quad::set_y(p, 41.3f);
    CHECK(vml::quad::get(p, 0) == Approx(141.3f));
    CHECK(vml::quad::get(p, 1) == Approx(41.3f));
    p = vml::quad::set_z(p, 41.3f);
    CHECK(vml::quad::get(p, 0) == Approx(141.3f));
    CHECK(vml::quad::get(p, 1) == Approx(41.3f));
    CHECK(vml::quad::get(p, 2) == Approx(41.3f));
    p = vml::quad::set_w(p, 41.3f);
    CHECK(vml::quad::get(p, 0) == Approx(141.3f));
    CHECK(vml::quad::get(p, 1) == Approx(41.3f));
    CHECK(vml::quad::get(p, 2) == Approx(41.3f));
    CHECK(vml::quad::get(p, 3) == Approx(41.3f));
    vml::quad_t q = vml::quad::set_x(31.3f);
    vml::quad_t r = vml::quad::set_x(p, q);
    CHECK(vml::quad::get(r, 0) == Approx(31.3f));
    r = vml::quad::set_y(r, q);
    CHECK(vml::quad::get(r, 0) == Approx(31.3f));
    CHECK(vml::quad::get(r, 1) == Approx(31.3f));
    r = vml::quad::set_z(r, q);
    CHECK(vml::quad::get(r, 0) == Approx(31.3f));
    CHECK(vml::quad::get(r, 1) == Approx(31.3f));
    CHECK(vml::quad::get(r, 2) == Approx(31.3f));
    r = vml::quad::set_w(r, q);
    CHECK(vml::quad::get(r, 0) == Approx(31.3f));
    CHECK(vml::quad::get(r, 1) == Approx(31.3f));
    CHECK(vml::quad::get(r, 2) == Approx(31.3f));
    CHECK(vml::quad::get(r, 3) == Approx(31.3f));
}

TEST_CASE("Validate quad::compare", "[quad::compare]") {
    vml::quad_t p = vml::quad::set(-441.3f, 23.0f, -1.0f, 10.0f);
    vml::quad_t q = vml::quad::set(441.3f, 5.0f, 51.0f, 10.0f);
    vml::quad_t r = vml::quad::set(445.3f, 15.0f, 151.0f, 110.0f);
    CHECK(vml::quad::greater_any(p, q));
    CHECK(vml::quad::greater_all(p, q) == false);
    CHECK(vml::quad::lesser_any(p, q));
    CHECK(vml::quad::lesser_all(p, q) == false);
    CHECK(vml::quad::greater_any(q, r) == false);
    CHECK(vml::quad::greater_all(q, r) == false);
    CHECK(vml::quad::lesser_any(q, r));
    CHECK(vml::quad::lesser_all(q, r));
    CHECK(vml::quad::greater_any(r, q));
    CHECK(vml::quad::greater_all(r, q));
    CHECK(vml::quad::lesser_any(r, q) == false);
    CHECK(vml::quad::lesser_all(r, q) == false);
}

TEST_CASE("Validate quad::arithmetic", "[quad::arithmetic]") {
    vml::quad_t p = vml::quad::set(10.0f, 23.0f, -1.0f, 10.0f);
    vml::quad_t q = vml::quad::set(441.3f, 5.0f, 51.0f, 10.0f);
    vml::quad_t r = vml::quad::mul_x(p, q);
    CHECK(vml::quad::x(r) == Approx(4413.0f));
    r = vml::quad::recip_sqrt_x(r);
    CHECK(vml::real::equals(vml::quad::x(r), (1 / vml::sqrt(4413.0f))));
    p = vml::quad::set(110.0f, 223.0f, 11.0f, 10.0f);
    q = vml::quad::set(10.0f);
    r = vml::quad::div(p, q);
    CHECK(vml::quad::x(r) == Approx(11.0f));
    CHECK(vml::quad::y(r) == Approx(22.3f));
    CHECK(vml::quad::z(r) == Approx(1.1f));
    CHECK(vml::quad::w(r) == Approx(1.0f));
    r = vml::quad::madd(p, q, q);
    CHECK(vml::quad::x(r) == Approx(1110.0f));
    CHECK(vml::quad::y(r) == Approx(2240.0f));
    CHECK(vml::quad::z(r) == Approx(120.0f));
    CHECK(vml::quad::w(r) == Approx(110.0f));
    r = vml::quad::vhadd(p);
    CHECK(vml::real::equals(vml::quad::x(r), vml::quad::hadd(p)));
    q = vml::quad::set(441.3f, 5.0f, 51.0f, 10.0f);
    r = vml::quad::recip_sqrt(q);
    CHECK(vml::real::equals(vml::quad::x(r), (1 / vml::sqrt(441.3f))));
    CHECK(vml::real::equals(vml::quad::y(r), (1 / vml::sqrt(5.0f))));
    CHECK(vml::real::equals(vml::quad::z(r), (1 / vml::sqrt(51.0f))));
    CHECK(vml::real::equals(vml::quad::w(r), (1 / vml::sqrt(10.0f))));
    std::uint32_t select_mask[4] = { 0xffffffff, 0, 0xffffffff, 0 };
    q = vml::quad::set(441.3f, 5.0f, 51.0f, 10.0f);
    p = vml::quad::set(10.0f, 23.0f, -1.0f, 20.0f);
    r = vml::quad::select(p, q, vml::quad::set_unaligned(reinterpret_cast<float const*>(select_mask)));
    CHECK(vml::quad::x(r) == Approx(441.3f));
    CHECK(vml::quad::y(r) == Approx(23.0f));
    CHECK(vml::quad::z(r) == Approx(51.0f));
    CHECK(vml::quad::w(r) == Approx(20.0f));
    q = vml::quad::set(1.3f, 1.2f, 1.6f, 1.8f);
    r = vml::quad::normalize(q);
    CHECK(vml::quad::x(r) == Approx(0.43503f));
    CHECK(vml::quad::y(r) == Approx(0.40156f));
    CHECK(vml::quad::z(r) == Approx(0.53542f));
    CHECK(vml::quad::w(r) == Approx(0.60234f));
    q = vml::quad::set(10.0f, 12.0f, 5.0f, 8.0f);
    p = vml::quad::set(20.0f, 20.0f, 15.0f, 20.0f);
    r = vml::quad::lerp(p, q, 0.5f);
    CHECK(vml::quad::x(r) == Approx(15.0f));
    CHECK(vml::quad::y(r) == Approx(16.0f));
    CHECK(vml::quad::z(r) == Approx(10.f));
    CHECK(vml::quad::w(r) == Approx(14.f));
    CHECK(vml::quad::distance(p, q) == Approx(20.19901));
    CHECK(vml::quad::sqdistance(p, q) == Approx(408.0f));
    q = vml::quad::set(10.0f, 12.0f, 5.0f, 8.0f);
    r = vml::quad::set_000w(q, 0);
    CHECK(vml::quad::x(r) == Approx(0.0f));
    CHECK(vml::quad::y(r) == Approx(0.0f));
    CHECK(vml::quad::z(r) == Approx(0.0f));
    CHECK(vml::quad::w(r) == Approx(10.0f));
    r = vml::quad::set_000w(q, 1);
    CHECK(vml::quad::x(r) == Approx(0.0f));
    CHECK(vml::quad::y(r) == Approx(0.0f));
    CHECK(vml::quad::z(r) == Approx(0.0f));
    CHECK(vml::quad::w(r) == Approx(12.0f));
    r = vml::quad::set_000w(q, 2);
    CHECK(vml::quad::x(r) == Approx(0.0f));
    CHECK(vml::quad::y(r) == Approx(0.0f));
    CHECK(vml::quad::z(r) == Approx(0.0f));
    CHECK(vml::quad::w(r) == Approx(5.0f));
    r = vml::quad::set_000w(q, 3);
    CHECK(vml::quad::x(r) == Approx(0.0f));
    CHECK(vml::quad::y(r) == Approx(0.0f));
    CHECK(vml::quad::z(r) == Approx(0.0f));
    CHECK(vml::quad::w(r) == Approx(8.0f));
    q = vml::quad::set(10.0f, 12.0f, 5.0f, 8.0f);
    r = vml::quad::set_111w(q, 0);
    CHECK(vml::quad::x(r) == Approx(1.0f));
    CHECK(vml::quad::y(r) == Approx(1.0f));
    CHECK(vml::quad::z(r) == Approx(1.0f));
    CHECK(vml::quad::w(r) == Approx(10.0f));
    r = vml::quad::set_111w(q, 1);
    CHECK(vml::quad::x(r) == Approx(1.0f));
    CHECK(vml::quad::y(r) == Approx(1.0f));
    CHECK(vml::quad::z(r) == Approx(1.0f));
    CHECK(vml::quad::w(r) == Approx(12.0f));
    r = vml::quad::set_111w(q, 2);
    CHECK(vml::quad::x(r) == Approx(1.0f));
    CHECK(vml::quad::y(r) == Approx(1.0f));
    CHECK(vml::quad::z(r) == Approx(1.0f));
    CHECK(vml::quad::w(r) == Approx(5.0f));
    r = vml::quad::set_111w(q, 3);
    CHECK(vml::quad::x(r) == Approx(1.0f));
    CHECK(vml::quad::y(r) == Approx(1.0f));
    CHECK(vml::quad::z(r) == Approx(1.0f));
    CHECK(vml::quad::w(r) == Approx(8.0f));
}
