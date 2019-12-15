#include <catch2/catch.hpp>
#include <vml.hpp>

TEST_CASE("Validate bounds_info::update", "[bounds_info::update]") {
    vml::bounds_info_t bounds1 = {
        {-2.0f, -2.0f, -2.0f},
        {2.0f, 2.0f, 2.0f},
        3.4641f
    };
    vml::bounds_info_t bounds2 = {
        {2.0f, 2.0f, 2.0f},
        {2.0f, 2.0f, 2.0f},
        3.4641f
    };

    vml::bounds_info_t bounds = vml::bounds_info::update(bounds1, bounds2);

    REQUIRE(vml::vec3::x(bounds.center) == Approx(0.0f));
    REQUIRE(vml::vec3::y(bounds.center) == Approx(0.0f));
    REQUIRE(vml::vec3::z(bounds.center) == Approx(0.0f));

    REQUIRE(vml::vec3::x(bounds.half_extends) == Approx(4.0));
    REQUIRE(vml::vec3::y(bounds.half_extends) == Approx(4.0));
    REQUIRE(vml::vec3::z(bounds.half_extends) == Approx(4.0));

    REQUIRE(bounds.radius == Approx(6.9282));
}

TEST_CASE("Validate bounding_volume::center", "[bounding_volume::center]") {
    vml::bounding_volume_t bounds1 = {
        vml::sphere::set(5.0f, 2.2f, 5.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
        vml::sphere::set(0.0f, 0.0f, 0.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
    };
    
    REQUIRE(vml::vec3a::x(vml::bounding_volume::center(bounds1)) == Approx(5.0f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::center(bounds1)) == Approx(2.2f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::center(bounds1)) == Approx(5.0f));
}

TEST_CASE("Validate bounding_volume::half_extends", "[bounding_volume::half_extends]") {
    vml::bounding_volume_t bounds1 = {
        vml::sphere::set(5.0f, 2.2f, 5.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
        vml::sphere::set(0.0f, 0.0f, 0.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
    };
    
    REQUIRE(vml::vec3a::x(vml::bounding_volume::half_extends(bounds1)) == Approx(2.0f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::half_extends(bounds1)) == Approx(2.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::half_extends(bounds1)) == Approx(2.0f));
}

TEST_CASE("Validate bounding_volume::radius", "[bounding_volume::radius]") {
    vml::bounding_volume_t bounds1 = {
        vml::sphere::set(5.0f, 2.2f, 5.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
        vml::sphere::set(0.0f, 0.0f, 0.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
    };
    
    REQUIRE(vml::bounding_volume::radius(bounds1) == Approx(3.4641f));
}

TEST_CASE("Validate bounding_volume::vradius", "[bounding_volume::vradius]") {
    vml::bounding_volume_t bounds1 = {
        vml::sphere::set(5.0f, 2.2f, 5.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
        vml::sphere::set(0.0f, 0.0f, 0.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
    };
    
    REQUIRE(vml::vec3a::x(vml::bounding_volume::vradius(bounds1)) == Approx(3.4641f));
}

TEST_CASE("Validate bounding_volume::nullify", "[bounding_volume::nullify]") {
    vml::bounding_volume_t bounds1 = {
        vml::sphere::set(5.0f, 2.2f, 5.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
        vml::sphere::set(0.0f, 0.0f, 0.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
    };
    vml::bounding_volume::nullify(bounds1);
    REQUIRE(vml::vec3a::x(vml::bounding_volume::center(bounds1)) == Approx(0.0f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::center(bounds1)) == Approx(0.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::center(bounds1)) == Approx(0.0f));
    REQUIRE(vml::bounding_volume::radius(bounds1) == Approx(0.0f));
    REQUIRE(vml::vec3a::x(vml::bounding_volume::half_extends(bounds1)) == Approx(0.0f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::half_extends(bounds1)) == Approx(0.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::half_extends(bounds1)) == Approx(0.0f));
}

TEST_CASE("Validate bounding_volume::from_box", "[bounding_volume::from_box]") {
    vml::bounding_volume_t bounds1 = vml::bounding_volume::from_box(vml::vec3a::set(2.0f, 1.0f, 5.0f), vml::vec3a::set(4.0f, 1.0f, 6.0f));
    REQUIRE(vml::vec3a::x(vml::bounding_volume::center(bounds1)) == Approx(2.0f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::center(bounds1)) == Approx(1.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::center(bounds1)) == Approx(5.0f));
    REQUIRE(vml::bounding_volume::radius(bounds1) == Approx(7.28011f));
    REQUIRE(vml::vec3a::x(vml::bounding_volume::half_extends(bounds1)) == Approx(4.0f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::half_extends(bounds1)) == Approx(1.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::half_extends(bounds1)) == Approx(6.0f));
}

TEST_CASE("Validate bounding_volume::set", "[bounding_volume::set]") {
    vml::bounding_volume_t bounds1 = {
        vml::sphere::set(5.0f, 2.2f, 5.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
        vml::sphere::set(0.0f, 0.0f, 0.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
    };
    bounds1 = vml::bounding_volume::set(vml::vec3a::set(2.0f, 1.0f, 5.0f), vml::vec3a::set(4.0f, 1.0f, 6.0f), 1100.0f);
    REQUIRE(vml::vec3a::x(vml::bounding_volume::center(bounds1)) == Approx(2.0f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::center(bounds1)) == Approx(1.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::center(bounds1)) == Approx(5.0f));
    REQUIRE(vml::bounding_volume::radius(bounds1) == Approx(1100.0f));
    REQUIRE(vml::vec3a::x(vml::bounding_volume::half_extends(bounds1)) == Approx(4.0f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::half_extends(bounds1)) == Approx(1.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::half_extends(bounds1)) == Approx(6.0f));

    bounds1 = vml::bounding_volume::set( vml::sphere::set(12.0f, 31.0f, 5.0f, 22.0f), vml::vec3a::set(14.0f, 15.0f, 61.0f));
    REQUIRE(vml::vec3a::x(vml::bounding_volume::center(bounds1)) == Approx(12.0f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::center(bounds1)) == Approx(31.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::center(bounds1)) == Approx(5.0f));
    REQUIRE(vml::bounding_volume::radius(bounds1) == Approx(22.0f));
    REQUIRE(vml::vec3a::x(vml::bounding_volume::half_extends(bounds1)) == Approx(14.0f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::half_extends(bounds1)) == Approx(15.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::half_extends(bounds1)) == Approx(61.0f));
}

TEST_CASE("Validate bounding_volume::update(matrix)", "[bounding_volume::update(matrix)]") {
    vml::bounding_volume_t bounds1 = {
        vml::sphere::set(5.0f, 2.2f, 5.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
        vml::sphere::set(0.0f, 0.0f, 0.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
    };

    vml::mat4_t m = vml::mat4::from_scale_rotation_translation(2.0f, 
        vml::quat::from_axis_angle(vml::vec3::set(0.0f, 1.0f, 0.0f), 
        vml::to_radians(45.0f)), 
        vml::vec3a::set(2.0f, 0.0f, 0.0f));

    vml::bounding_volume::update(bounds1, m);

    REQUIRE(vml::vec3a::x(vml::bounding_volume::center(bounds1)) == Approx(2.0f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::center(bounds1)) == Approx(0.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::center(bounds1)) == Approx(0.0f));
    REQUIRE(vml::bounding_volume::radius(bounds1) == Approx(6.9282f));
    REQUIRE(vml::vec3a::x(vml::bounding_volume::half_extends(bounds1)) == Approx(5.65685f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::half_extends(bounds1)) == Approx(4.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::half_extends(bounds1)) == Approx(5.65685));
}

TEST_CASE("Validate bounding_volume::update(srt)", "[bounding_volume::update(srt)]") {
    vml::bounding_volume_t bounds1 = {
        vml::sphere::set(5.0f, 2.2f, 5.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
        vml::sphere::set(0.0f, 0.0f, 0.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
    };

    vml::bounding_volume::update(bounds1, 2.0f, 
        vml::quat::from_axis_angle(vml::vec3::set(0.0f, 1.0f, 0.0f), 
        vml::to_radians(45.0f)), 
        vml::vec3a::set(2.0f, 0.0f, 0.0f));

    REQUIRE(vml::vec3a::x(vml::bounding_volume::center(bounds1)) == Approx(2.0f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::center(bounds1)) == Approx(0.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::center(bounds1)) == Approx(0.0f));
    REQUIRE(vml::bounding_volume::radius(bounds1) == Approx(6.9282f));
    REQUIRE(vml::vec3a::x(vml::bounding_volume::half_extends(bounds1)) == Approx(5.65685f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::half_extends(bounds1)) == Approx(4.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::half_extends(bounds1)) == Approx(5.65685));
}


TEST_CASE("Validate bounding_volume::update(transform)", "[bounding_volume::update(transform)]") {
    vml::bounding_volume_t bounds1 = {
        vml::sphere::set(5.0f, 2.2f, 5.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
        vml::sphere::set(0.0f, 0.0f, 0.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
    };

    vml::transform_t tf;
    vml::transform::set_rotation(tf, vml::quat::from_axis_angle(vml::vec3::set(0.0f, 1.0f, 0.0f), vml::to_radians(45.0f)));
    vml::transform::set_scale(tf, 2.0f);
    vml::transform::set_translation(tf, vml::vec3a::set(2.0f, 0.0f, 0.0f));

    vml::bounding_volume::update(bounds1, tf);

    REQUIRE(vml::vec3a::x(vml::bounding_volume::center(bounds1)) == Approx(2.0f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::center(bounds1)) == Approx(0.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::center(bounds1)) == Approx(0.0f));
    REQUIRE(vml::bounding_volume::radius(bounds1) == Approx(6.9282f));
    REQUIRE(vml::vec3a::x(vml::bounding_volume::half_extends(bounds1)) == Approx(5.65685f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::half_extends(bounds1)) == Approx(4.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::half_extends(bounds1)) == Approx(5.65685));
}


TEST_CASE("Validate bounding_volume::update(bounding_volume)", "[bounding_volume::update(bounding_volume)]") {
    vml::bounding_volume_t bounds1 = {
        vml::sphere::set(5.0f, 4.0f, 5.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
        vml::sphere::set(0.0f, 0.0f, 0.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
    };
    vml::bounding_volume_t bounds2 = {
        vml::sphere::set(-5.0f,-4.0f, 5.0f, 13.4641f),
        vml::vec3a::set(10.0f, 10.0f, 10.0f),
        vml::sphere::set(0.0f, 0.0f, 0.0f, 13.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
    };

    vml::bounding_volume::update(bounds1, bounds2);

    REQUIRE(vml::vec3a::x(vml::bounding_volume::center(bounds1)) == Approx(-4.0f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::center(bounds1)) == Approx(-4.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::center(bounds1)) == Approx(5.0f));
    REQUIRE(vml::bounding_volume::radius(bounds1) == Approx(14.86722f));
    REQUIRE(vml::vec3a::x(vml::bounding_volume::half_extends(bounds1)) == Approx(11.0f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::half_extends(bounds1)) == Approx(10.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::half_extends(bounds1)) == Approx(10.0f));
}



TEST_CASE("Validate bounding_volume::update(points)", "[bounding_volume::update(points)]") {
    vml::bounding_volume_t bounds1 = {
        vml::sphere::set(5.0f, 4.0f, 5.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
        vml::sphere::set(0.0f, 0.0f, 0.0f, 3.4641f),
        vml::vec3a::set(2.0f, 2.0f, 2.0f),
    };
    vml::vec3a_t points[3] = {
        vml::vec3a::set(-5.0f,-4.0f, 5.0f),
        vml::vec3a::set(-15.0f,-14.0f, -5.0f),
        vml::vec3a::set(5.0f, 6.0f, 15.0f)
    };

    vml::bounding_volume::update(bounds1, points, 3);

    REQUIRE(vml::vec3a::x(vml::bounding_volume::center(bounds1)) == Approx(-4.0f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::center(bounds1)) == Approx(-4.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::center(bounds1)) == Approx(5.0f));
    REQUIRE(vml::bounding_volume::radius(bounds1) == Approx(17.91647f));
    REQUIRE(vml::vec3a::x(vml::bounding_volume::half_extends(bounds1)) == Approx(11.0f));
    REQUIRE(vml::vec3a::y(vml::bounding_volume::half_extends(bounds1)) == Approx(10.0f));
    REQUIRE(vml::vec3a::z(vml::bounding_volume::half_extends(bounds1)) == Approx(10.0f));
}

