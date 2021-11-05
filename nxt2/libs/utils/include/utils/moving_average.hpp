/*******************************************************************************
* Copyright (C) 2015 T. Reich
*
* This file is part of rpi-nxt2 experiment.
*
* NXT application utilities
*
* License notes see LICENSE.txt
*******************************************************************************/


#ifndef __NXT_UTILS_MOVING_AVERAGE_HPP__
#define __NXT_UTILS_MOVING_AVERAGE_HPP__

namespace nxt
{
namespace utils
{
template <typename Value_T, int N> class MovingAverage
{
public:

    MovingAverage(Value_T def = 0)
       : _sum(0), _idx(0)
    {
        for (int i = 0; i < N; i++)
        {
            _samples[i] = def;
            _sum += _samples[i];
        }
    }

    void add(Value_T val)
    {
        _sum = _sum - _samples[_idx] + val;
        _samples[_idx++] = val;

        if (_idx >= N)
        {
            _idx = 0;
        }
    }

    Value_T get()
    {
        return _sum / N;
    }

private:

    Value_T _samples[N];
    Value_T _sum;

    int _idx;
};

} // shared
} // nxt

#endif /* __NXT_UTILS_MOVING_AVERAGE_HPP__ */