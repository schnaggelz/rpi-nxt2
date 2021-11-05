/*******************************************************************************
 * Copyright (C) 2015 T. Reich
 *
 * NXT shared utilities
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_COMMON_UTILS_CONVERSION_HPP__
#define __NXT_COMMON_UTILS_CONVERSION_HPP__

#include <type_traits>

namespace nxt
{
namespace utils
{
template <typename E>
constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept
{
    return static_cast<typename std::underlying_type<E>::type>(e);
}
} // namespace utils
} // namespace nxt

#endif /* __NXT_COMMON_UTILS_CONVERSION_HPP__ */