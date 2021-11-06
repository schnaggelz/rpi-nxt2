/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* This file is part of rpi-nxt2 experiment.
*
* NXT application base definitions
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef __NXT_APP_BASE_RUNNABLE_BASE_HPP__
#define __NXT_APP_BASE_RUNNABLE_BASE_HPP__

#include <cstdint>

namespace nxt
{
namespace app_base
{
class Runnable
{
public:
    virtual void init() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;
};

} // shared
} // nxt

#endif /* __NXT_APP_BASE_RUNNABLE_BASE_HPP__ */