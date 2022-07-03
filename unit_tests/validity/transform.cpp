#include <catch2/catch.hpp>
#include <cmath>
#include <limits>
#include <vml.hpp>

TEST_CASE("Validate transform::combine", "[transform::combine]")
{
  vml::transform_t t   = vml::transform::identity();
  vml::quat_t      rot = vml::quat::from_axis_angle(vml::vec3::set(0, 1.0f, 0), vml::to_radians(20.0f));

  vml::transform::set_translation(t, vml::vec3a::set(10.0f));
  vml::transform::set_scale(t, 2.5f);
  vml::transform::set_rotation(t, rot);

  vml::mat4_t scale     = vml::mat4::from_scale(vml::vec3a::set(2.5f));
  vml::mat4_t rotate    = vml::mat4::from_rotation(rot);
  vml::mat4_t translate = vml::mat4::from_translation(vml::vec3a::set(10.0f));
  vml::mat4_t expected  = vml::mat4::mul(scale, vml::mat4::mul(rotate, translate));

  vml::mat4_t result;

  vml::transform::matrix(t, result);

  CHECK(vml::mat4::equals(result, expected));

  vml::transform_t id;
  vml::transform::identity(id);

  t = vml::transform::combine(id, t);

  vml::transform::matrix(t, result);

  CHECK(vml::mat4::equals(result, expected));

  t = vml::transform::combine(t, id);

  CHECK(vml::mat4::equals(result, expected));

  vml::transform_t t2 = vml::transform::identity();

  vml::quat_t rot2 = vml::quat::from_axis_angle(vml::vec3::set(1.0f, 0.0f, 0), vml::to_radians(120.0f));

  vml::transform::set_translation(t2, vml::vec3a::set(10.0f));
  vml::transform::set_scale(t2, 0.5f);
  vml::transform::set_rotation(t2, rot2);

  vml::transform_t combined = vml::transform::combine(t2, t);

  vml::mat4_t tm, t2m, exp, res;
  vml::transform::matrix(t2, t2m);
  vml::transform::matrix(t, tm);
  exp = vml::mat4::mul(tm, t2m);

  vml::transform::matrix(combined, res);

  CHECK(vml::mat4::equals(res, exp));

  vml::vec3a_t point = vml::vec3a::set(15.0f, 442.04f, 23.0f);
  CHECK(vml::vec3a::equals(vml::vec3a::mul(point, res), vml::transform::mul(point, combined)));
}