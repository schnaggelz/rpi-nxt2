/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * This file is part of rpi-nxt2 experiment.
 *
 * NXT application base definitions
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_APP_BASE_MESSAGE_BASE_HPP__
#define __NXT_APP_BASE_MESSAGE_BASE_HPP__

namespace nxt
{
namespace app_base
{
struct MessageBase
{
    template <typename T> static T& instance()
    {
        static T _instance;
        return _instance;
    }
};

} // namespace libs
} // namespace nxt

#endif /* __NXT_APP_BASE_MESSAGE_BASE_HPP__ */