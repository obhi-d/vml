
#include <intersect.hpp>

namespace vml
{
namespace intersect
{

VML_API result_t bounding_volume_frustum_coherent(bounding_volume_t const& i_vol, frustum_t const& i_frustum,
                                                  frustum_t::coherency& i_coherency)
{

  result_t      result   = result_t::k_inside;
  auto          planes   = frustum::get_planes(i_frustum);
  std::uint32_t out_mask = 0;
#ifndef NDEBUG
  i_coherency.iterations = 0;
#endif

  for (std::uint32_t i = 0; i < planes.second; i++
#ifndef NDEBUG
                                               ,
                     i_coherency.iterations++
#endif
  )
  {
    std::uint32_t plane = (i + i_coherency.plane) % planes.second;
    std::uint32_t k     = 1 << plane;
    if ((k & i_coherency.mask_hierarchy))
    {
      vec3a_t abs_norm = plane::abs_normal(planes.first[plane]);
      auto    m        = plane::vdot(planes.first[plane], sphere::center(i_vol.spherical_vol));
      auto    n        = vec3a::vdot(abs_norm, i_vol.half_extends);
      if (quad::isnegative_x(quad::add_x(m, n)))
      {
        i_coherency.plane = plane;
        return result_t::k_outside;
      }
      if (quad::isnegative_x(quad::sub_x(m, n)))
      {
        out_mask |= k;
        result = result_t::k_intersecting;
      }
    }
  }
  i_coherency.mask_hierarchy = out_mask;
  return result;
}

VML_API result_t bounding_volume_frustum(bounding_volume_t const& i_vol, frustum_t const& i_frustum)
{

  auto planes = frustum::get_planes(i_frustum);

  for (std::uint32_t i = 0; i < planes.second; i++)
  {
    std::uint32_t plane    = i;
    vec3a_t       abs_norm = plane::abs_normal(planes.first[plane]);
    auto          m        = plane::vdot(planes.first[plane], sphere::center(i_vol.spherical_vol));
    auto          n        = vec3a::vdot(abs_norm, i_vol.half_extends);
    if (quad::isnegative_x(quad::add_x(m, n)))
      return result_t::k_outside;

    if (quad::isnegative_x(quad::sub_x(m, n)))
      return result_t::k_intersecting;
  }
  return result_t::k_inside;
}

VML_API result_t bounding_sphere_frustum(sphere::pref i_sphere, frustum_t const& i_frustum)
{

  auto   planes = frustum::get_planes(i_frustum);
  quad_t vrad   = vec3a::negate(sphere::vradius(i_sphere));
  for (std::uint32_t i = 0; i < planes.second; i++)
  {
    std::uint32_t plane = i;
    auto          m     = plane::vdot(planes.first[plane], sphere::center(i_sphere));
    if (quad::islesser_x(m, vrad))
      return result_t::k_outside;
    if (quad::isnegative_x(quad::add_x(m, vrad)))
      return result_t::k_intersecting;
  }
  return result_t::k_inside;
}
} // namespace intersect
} // namespace vml