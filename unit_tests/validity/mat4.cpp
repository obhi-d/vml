#include <catch2/catch.hpp>
#include <cstring>
#include <vml.hpp>

TEST_CASE("Validate mat4::mul", "[mat4::mul]") {
	vml::mat4_t m1 = {
	    5.0f, 7.0f,  9.0f, 10.0f, 2.0f, 3.0f, 3.0f, 8.0f,
	    8.0f, 10.0f, 2.0f, 3.0f,  3.0f, 3.0f, 4.0f, 8.0f,
	};

	vml::mat4_t m2 = {
	    3.0f, 10.0f, 12.0f, 18.0f, 12.0f, 1.0f,  4.0f, 9.0f,
	    9.0f, 10.0f, 12.0f, 2.0f,  3.0f,  12.0f, 4.0f, 10.0f,
	};

	vml::mat4_t m3 = {
	    210.0f, 267.0f, 236.0f, 271.0f, 93.0f,  149.0f, 104.0f, 149.0f,
	    171.0f, 146.0f, 172.0f, 268.0f, 105.0f, 169.0f, 128.0f, 169.0f,
	};


	vml::mat4_t m2_times_3 = {
	    9.0f, 30.0f, 36.0f, 54.0f, 36.0f, 3.0f,  12.0f, 27.0f,
	    27.0f, 30.0f, 36.0f, 6.0f,  9.0f,  36.0f, 12.0f, 30.0f,
	};


	vml::mat4_t identity = vml::mat4::identity();

	vml::mat4_t im1  = vml::mat4::mul(identity, m1);
	vml::mat4_t m1i  = vml::mat4::mul(m1, identity);
	vml::mat4_t m1m2 = vml::mat4::mul(m1, m2);

	CHECK(vml::mat4::equals(im1, m1i));
	CHECK(std::memcmp(&im1, &m1i, sizeof(im1)) == 0);
	CHECK(vml::mat4::equals(m1m2, m3));

	CHECK(vml::mat4::equals(vml::mat4::mul(3.0f, m2), m2_times_3));
	CHECK(vml::vec4::equals(vml::mat4::mul(vml::mat4::row(m1, 0), m2), vml::mat4::row(m1m2, 0)));
}

TEST_CASE("Validate mat4::transform_assume_ortho",
          "[mat4::transform_assume_ortho]") {
	vml::mat4_t m = {
	    0.0f,   0.80f, 0.60f,  0.0f, -0.80f, -0.36f, 0.48f, 0.0f,
	    -0.60f, 0.48f, -0.64f, 0.0f, 0.0f,   0.0f,   0.0f,  1.0f,
	};

	vml::vec3_t const points[4] = {
	    {3.000, 10.000, 12.000},
	    {12.000, 1.000, 4.000},
	    {9.000, 10.000, 12.000},
	    {3.000, 12.000, 4.000},
	};

	vml::vec3_t const expected[4] = {
	    {-5.200, 24.560, 18.920},
	    {6.800, 31.160, 25.120},
	    {-5.200, 29.360, 22.520},
	    {-2.000, 20.000, 25.000},
	};

	vml::vec3_t output[4];

	vml::mat4::transform_assume_ortho(m, points, sizeof(vml::vec3_t), 4, output,
	                                  sizeof(vml::vec3_t));

	for (int i = 0; i < 4; ++i) {
		CHECK(vml::vec3::equals(output[i], expected[i]));
		output[i] = points[i];
	}

	vml::mat4::transform_assume_ortho(m, output, sizeof(vml::vec3_t), 4);

	for (int i = 0; i < 4; ++i) {
		CHECK(vml::vec3::equals(output[i], expected[i]));
	}

	CHECK(vml::vec3a::equals(vml::mat4::transform_assume_ortho(
	                             m, vml::vec3a::set(3.000, 10.000, 12.000)),
	                         vml::vec4::set(-5.200, 24.560, 18.920)));
}

TEST_CASE("Validate mat4::transform_and_project",
          "[mat4::transform_and_project]") {
	vml::mat4_t m = {
	    5.0f, 7.0f,  9.0f, 10.0f, 2.0f, 3.0f, 3.0f, 8.0f,
	    8.0f, 10.0f, 2.0f, 3.0f,  3.0f, 3.0f, 4.0f, 8.0f,
	};

	vml::vec3_t const points[4] = {
	    {3.000, 10.000, 12.000},
	    {12.000, 1.000, 4.000},
	    {9.000, 10.000, 12.000},
	    {3.000, 12.000, 4.000},
	};

	vml::vec3_t const expected[4] = {
	   { 0.87012987013f, 1.12987012987f, 0.55194805194f },
       { 0.65540540540f, 0.87837837837f, 0.83108108108f },
       { 0.76635514018f, 1.00934579439f, 0.64953271028f },
       { 0.50684931506f, 0.68493150684f, 0.51369863013f },
	};

	vml::vec3_t output[4];

	vml::mat4::transform_and_project(m, points, sizeof(vml::vec3_t), 4, output,
	                                  sizeof(vml::vec3_t));

	for (int i = 0; i < 4; ++i) {
		CHECK(vml::vec3::equals(output[i], expected[i]));
		output[i] = points[i];
	}

	CHECK(vml::vec3a::equals(vml::mat4::transform_and_project(
	                             m, vml::vec3a::set(3.000, 10.000, 12.000)),
	                         vml::vec4::set(0.87012987013f, 1.12987012987f, 0.55194805194f)));
}

TEST_CASE("Validate mat4::transform_aabb",
          "[mat4::transform_aabb]") {
    vml::aabb_t aabb = vml::aabb::set(vml::vec3a::zero(), vml::vec3a::set(4, 2, 2));
    vml::mat4_t scale = vml::mat4::from_scale(vml::vec3a::set(2, 2, 2));
    vml::mat4_t rotate = vml::mat4::from_rotation(vml::quat::from_axis_angle(vml::axis_angle::set(0, 0, 1, vml::to_radians(90.0))));
    vml::mat4_t translate = vml::mat4::from_translation(vml::vec3a::set(10, 0, 0));
    vml::mat4_t combined = vml::mat4::mul(scale, vml::mat4::mul(rotate, translate));
    vml::aabb_t expected = vml::aabb::set(vml::vec3a::set(10, 0, 0), vml::vec3a::set(4, 8, 4));

    vml::aabb_t result = vml::mat4::transform_aabb(combined, aabb);

    CHECK(vml::aabb::equals(expected, result));
}


TEST_CASE("Validate mat4::from_perspective_projection",
          "[mat4::from_perspective_projection]") {
    vml::mat4_t proj = vml::mat4::from_perspective_projection(vml::k_pi_by_2, 1.2f, 1.0f, 100.0f);
    float y_scale = 1.0f / std::tan(static_cast<float>(3.14159265358979323846 / 2.0));
	float x_scale = y_scale / 1.2f;
    CHECK(vml::mat4::get(proj, 0, 0) == Approx(x_scale));
	CHECK(vml::mat4::get(proj, 0, 1) == Approx(0.0f));
	CHECK(vml::mat4::get(proj, 0, 2) == Approx(0.0f));
	CHECK(vml::mat4::get(proj, 0, 3) == Approx(0.0f));
    CHECK(vml::mat4::get(proj, 1, 0) == Approx(0.0f));
	CHECK(vml::mat4::get(proj, 1, 1) == Approx(y_scale));
	CHECK(vml::mat4::get(proj, 1, 2) == Approx(0.0f));
	CHECK(vml::mat4::get(proj, 1, 3) == Approx(0.0f));
    CHECK(vml::mat4::get(proj, 2, 0) == Approx(0.0f));
	CHECK(vml::mat4::get(proj, 2, 1) == Approx(0.0f));
	CHECK(vml::mat4::get(proj, 2, 2) == Approx(100.0f / 99.0f));
	CHECK(vml::mat4::get(proj, 2, 3) == Approx(1.0f));
    CHECK(vml::mat4::get(proj, 3, 0) == Approx(0.0f));
	CHECK(vml::mat4::get(proj, 3, 1) == Approx(0.0f));
	CHECK(vml::mat4::get(proj, 3, 2) == Approx(-1.0f));
	CHECK(vml::mat4::get(proj, 3, 3) == Approx(0.0f));
}

TEST_CASE("Validate mat4::inverse",
          "[mat4::inverse]") {
	vml::mat4_t m   = {3.6, 6.3, 4.4, 6.7, 1.2, 5.7, 6.5, 2.2,
                   7.8, 5.5, 3.6, 7.7, 3.3, 5.3, 5.6, 7.7};
	vml::mat4_t inv = {
	    -0.23284, 0.08172,  0.25525,  -0.07600, 0.56183, 0.01669,
	    -0.09192, -0.40172, -0.46771, 0.18554,  0.06512, 0.28884,
	    0.05323,  -0.18145, -0.09349, 0.22888
	};
	
	CHECK(vml::mat4::equals(vml::mat4::inverse(m), inv));

	vml::mat4_t o = {
	    0.0f,    0.80f,  0.60f,  0.0f, 
		-0.80f, -0.36f,  0.48f, 0.0f,
	    -0.60f,  0.48f, -0.64f, 0.0f, 
		12.0f,   20.0f,  3.0f,  1.0f,
	};

	vml::mat4_t oi = {
		0 		,-0.8 ,	-0.6 ,	0,
		0.8		,-0.36, 0.48 ,	0,
		0.6  	,0.48 ,	-0.64, 	0,
		-17.8	,15.36, -0.48, 	1
	};

	CHECK(vml::mat4::equals(vml::mat4::inverse_assume_ortho(o), oi));
}