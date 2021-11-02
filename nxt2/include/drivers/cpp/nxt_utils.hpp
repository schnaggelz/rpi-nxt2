#ifndef __NXT_UTILS_HPP__
#define __NXT_UTILS_HPP__

#include <stdint.h>

#include <type_traits>

namespace nxt
{
    template <typename E>
    constexpr typename std::underlying_type<E>::type
        to_underlying(E e) noexcept
    {
        return static_cast<typename std::underlying_type<E>::type>(e);
    }
} // nxt

#endif /* __NXT_UTILS_HPP__ */