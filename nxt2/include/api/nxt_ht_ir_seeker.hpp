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
        InfraredSeeker(uint8_t port_number)
            : Sensor(port_number)
        {
        }


        std::array<int8_t, 12>& getData() { return _ir_data; };

        void init() override;
        void read() override;
        void exit() override;

    private:

        std::array<int8_t, 12> _ir_data;
    };
} // ht
} // nxt

#endif /* __NXT_HT_IR_SEEKER_HPP__ */
