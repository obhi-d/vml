
# Introduction

VML is a single precision 3D vector math library, with SSE friendly implemenation alongside a reference pure C++ implementation.
VML is a header only library, so including vml.hpp is enough to include all files.

## Structure

VML classes are under the namespace `vml`. All types are typed as `type_t` while operations corresponding to those types are under `type` structure. An example:

    vml::vec3_t v = vml::vec3::set(1.0f, 2.0f, 3.0f);

Examples usage can be found under the unit_tests directory.

## Memory layout

Vectors with 4 elements store x as 0th element (low bits)

For vectors -> hi[w, z, y, x]low

Matrices are row vector, row major.

# Building

VML is header only. CMake configuration for build is provided to install the headers in a user location. Note that you have to provide `CMAKE_INSTALL_PREFIX` or `DSTDIR` to install in custom location.
To build from source directory `vml`:

    mkdir ../build &&  cd ../build
    cmake vml -G Ninja
    cmake --build . --target install --config Release


# Running Tests

If `VML_BUILD_TESTS` is set to `ON` in CMake configuration, tests will be automatically build.
In debug build, tests will generate coverage report for lcov if the build is done in Linux.
A handy script is provided to merge the converage report:
    unit_tests/coverage.sh

# License

VML is under MIT license. See LICENSE file for details.
