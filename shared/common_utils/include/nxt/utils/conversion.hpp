/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
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
template <typename E>
constexpr E to_enum(typename std::underlying_type<E>::type v) noexcept
{
    return static_cast<E>(v);
}
} // namespace utils
} // namespace nxt

#endif /* __NXT_COMMON_UTILS_CONVERSION_HPP__ */