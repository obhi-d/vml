#pragma once
#include "vec_base.hpp"

namespace vml
{

namespace detail
{
struct ivec4_traits
{

  using scalar_type = int;
  using row_type    = scalar_type;
  using type        = types::vec4_t<scalar_type>;
  using ref         = type&;
  using pref        = std::conditional_t<types::is_pref_cref, type const&, type>;
  using cref        = type const&;

  static constexpr std::uint32_t element_count = 4;
  static constexpr std::uint32_t row_count     = 1;
  static constexpr std::uint32_t column_count  = 4;
};
} // namespace detail
struct ivec4 : public vec_base<detail::ivec4_traits>
{};
} // namespace vml