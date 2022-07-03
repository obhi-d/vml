#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <vml-impl.hpp>
#include <vml.hpp>

TEST_CASE("Validate rect", "[rect]")
{
  vml::rect_t r  = vml::rect::set(100.0, 0, 200.0, 400.0);
  auto        m2 = vml::rect::center(r);
  CHECK(vml::vec2::equals(vml::vec2_t{150, 200}, m2));
  m2 = vml::rect::half_size(r);
  CHECK(vml::vec2::equals(vml::vec2_t{50, 200}, m2));
  m2 = vml::rect::size(r);
  CHECK(vml::vec2::equals(vml::vec2_t{100, 400}, m2));
}

TEST_CASE("Validate irect", "[irect]")
{
  vml::irect_t r  = vml::irect::set(100, 0, 200, 400);
  auto         m2 = vml::irect::center(r);
  CHECK(vml::ivec2::equals(vml::ivec2_t{150, 200}, m2));
  m2 = vml::irect::half_size(r);
  CHECK(vml::ivec2::equals(vml::ivec2_t{50, 200}, m2));
  m2 = vml::irect::size(r);
  CHECK(vml::ivec2::equals(vml::ivec2_t{100, 400}, m2));
  CHECK(vml::irect::width(r) == 100);
  CHECK(vml::irect::height(r) == 400);
  CHECK(vml::irect::left(r) == 100);
  CHECK(vml::irect::top(r) == 0);
  CHECK(vml::irect::right(r) == 200);
  CHECK(vml::irect::bottom(r) == 400);
}
