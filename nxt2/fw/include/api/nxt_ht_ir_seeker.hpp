/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#ifndef __NXT_HT_IR_SEEKER_HPP__
#define __NXT_HT_IR_SEEKER_HPP__

#include "nxt_sensor.hpp"

namespace nxt
{
namespace ht
{
class InfraredSeeker : public Sensor
{
  public:
    static constexpr int NUM_VALUES = 12;

    InfraredSeeker(Port port)
        : Sensor(port)
    {
    }

    std::array<std::int8_t, NUM_VALUES>& getData()
    {
        return _ir_data;
    }

    void init() override;
    void read() override;
    void exit() override;

  private:
    std::array<std::int8_t, NUM_VALUES> _ir_data;
};

} // namespace ht
} // namespace nxt

#endif /* __NXT_HT_IR_SEEKER_HPP__ */
