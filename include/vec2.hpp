#pragma once
#include "vec_base.hpp"

namespace vml
{

namespace detail
{
struct vec2_traits
{

  using type        = types::vec2_t<float>;
  using ref         = type&;
  using pref        = std::conditional_t<types::is_pref_cref, type const&, type>;
  using cref        = type const&;
  using scalar_type = float;
  using row_type    = float;

  static constexpr std::uint32_t element_count = 2;
};
} // namespace detail
struct vec2 : public vec_base<detail::vec2_traits>
{};
} // namespace vml