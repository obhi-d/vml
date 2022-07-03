#include <catch2/catch.hpp>
#include <cmath>
#include <limits>
#include <vml.hpp>

TEST_CASE("Validate quat::mul", "[quat::mul]")
{

  {
    vml::quat_t p = vml::quat::from_axis_angle(vml::vec3::set(0, 1.0f, 0), vml::to_radians(20.0f));
    vml::mat4_t m = vml::mat4::from_quat(p);
    vml::mat4_t t = vml::mat4::from_translation(vml::vec3a::set(10.1f, 42.f, 0));
    m             = vml::mat4::mul(m, t);

    CHECK(vml::quat::equals(p, vml::quat::from_mat4(m)));
    CHECK(vml::quat::equals(p, vml::quat::from_mat3(vml::mat4::as_mat3(m))));
    CHECK(vml::quat::equals(p, vml::quat::mul(p, vml::quat::identity())));
    CHECK(vml::quat::equals(p, vml::quat::mul(vml::quat::identity(), p)));
    CHECK(vml::quat::equals(vml::quat::identity(), vml::quat::mul(vml::quat::conjugate(p), p)));
    CHECK(vml::quat::equals(vml::quat::identity(), vml::quat::mul(p, vml::quat::conjugate(p))));
    CHECK(vml::quat::equals(vml::quat::identity(), vml::quat::mul(vml::quat::inverse(p), p)));
    CHECK(vml::quat::equals(vml::quat::identity(), vml::quat::mul(p, vml::quat::inverse(p))));
  }

  {
    vml::quat_t p = vml::quat::from_axis_angle(vml::vec3::set(0, 1.0f, 0), vml::to_radians(180.0f));
    vml::mat4_t m = vml::mat4::from_quat(p);
    vml::mat4_t t = vml::mat4::from_translation(vml::vec3a::set(10.1f, 42.f, 0));
    m             = vml::mat4::mul(m, t);

    CHECK(vml::quat::equals(p, vml::quat::from_mat4(m)));
    CHECK(vml::quat::equals(p, vml::quat::from_mat3(vml::mat4::as_mat3(m))));
    CHECK(vml::quat::equals(p, vml::quat::mul(p, vml::quat::identity())));
    CHECK(vml::quat::equals(p, vml::quat::mul(vml::quat::identity(), p)));
    CHECK(vml::quat::equals(vml::quat::identity(), vml::quat::mul(vml::quat::conjugate(p), p)));
    CHECK(vml::quat::equals(vml::quat::identity(), vml::quat::mul(p, vml::quat::conjugate(p))));
    CHECK(vml::quat::equals(vml::quat::identity(), vml::quat::mul(vml::quat::inverse(p), p)));
    CHECK(vml::quat::equals(vml::quat::identity(), vml::quat::mul(p, vml::quat::inverse(p))));
  }

  {
    vml::quat_t p = vml::quat::from_axis_angle(vml::vec3::set(1.0, 0.0f, 0), vml::to_radians(180.0f));
    vml::mat4_t m = vml::mat4::from_quat(p);
    vml::mat4_t t = vml::mat4::from_translation(vml::vec3a::set(10.1f, 42.f, 0));
    m             = vml::mat4::mul(m, t);

    CHECK(vml::quat::equals(p, vml::quat::from_mat4(m)));
    CHECK(vml::quat::equals(p, vml::quat::from_mat3(vml::mat4::as_mat3(m))));
    CHECK(vml::quat::equals(p, vml::quat::mul(p, vml::quat::identity())));
    CHECK(vml::quat::equals(p, vml::quat::mul(vml::quat::identity(), p)));
    CHECK(vml::quat::equals(vml::quat::identity(), vml::quat::mul(vml::quat::conjugate(p), p)));
    CHECK(vml::quat::equals(vml::quat::identity(), vml::quat::mul(p, vml::quat::conjugate(p))));
    CHECK(vml::quat::equals(vml::quat::identity(), vml::quat::mul(vml::quat::inverse(p), p)));
    CHECK(vml::quat::equals(vml::quat::identity(), vml::quat::mul(p, vml::quat::inverse(p))));
  }

  {
    vml::quat_t p = vml::quat::from_axis_angle(vml::vec3::set(0.0, 0.0f, 1.0f), vml::to_radians(180.0f));
    vml::mat4_t m = vml::mat4::from_quat(p);
    vml::mat4_t t = vml::mat4::from_translation(vml::vec3a::set(10.1f, 42.f, 0));
    m             = vml::mat4::mul(m, t);

    CHECK(vml::quat::equals(p, vml::quat::from_mat4(m)));
    CHECK(vml::quat::equals(p, vml::quat::from_mat3(vml::mat4::as_mat3(m))));
    CHECK(vml::quat::equals(p, vml::quat::mul(p, vml::quat::identity())));
    CHECK(vml::quat::equals(p, vml::quat::mul(vml::quat::identity(), p)));
    CHECK(vml::quat::equals(vml::quat::identity(), vml::quat::mul(vml::quat::conjugate(p), p)));
    CHECK(vml::quat::equals(vml::quat::identity(), vml::quat::mul(p, vml::quat::conjugate(p))));
    CHECK(vml::quat::equals(vml::quat::identity(), vml::quat::mul(vml::quat::inverse(p), p)));
    CHECK(vml::quat::equals(vml::quat::identity(), vml::quat::mul(p, vml::quat::inverse(p))));
  }
}

TEST_CASE("Validate quat::slerp", "[quat::slerp]")
{

  vml::quat_t p = vml::quat::from_axis_angle(vml::vec3::set(0, 1.0f, 0), vml::to_radians(20.0f));
  vml::quat_t q = vml::quat::from_axis_angle(vml::vec3::set(0, 1.0f, 0), vml::to_radians(120.0f));
  vml::quat_t r = vml::quat::from_axis_angle(vml::vec3::set(0, 1.0f, 0), vml::to_radians(70.0f));
  CHECK(vml::quat::equals(r, vml::quat::slerp(p, q, 0.5f)));
}
