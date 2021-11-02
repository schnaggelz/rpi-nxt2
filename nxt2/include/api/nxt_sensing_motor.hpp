#ifndef __NXT_SENSING_MOTOR_HPP__
#define __NXT_SENSING_MOTOR_HPP__

#include "nxt_sensor.hpp"

namespace nxt
{
    class SensingMotor : public Sensor
    {
    public:

        SensingMotor(uint8_t port_number, bool brake)
            : Sensor(port_number)
            , _current_speed(0)
            , _brake(brake)
        {
        }

        void setSpeed(int32_t speed);

        void setCount(int32_t count);

        int32_t getCount() const;

        int32_t getSpeed() const
        {
            return _current_speed;
        }

        void init() override;
        void read() override;
        void exit() override;

    private:

        int32_t _current_speed;

        bool _brake;
    };
};

#endif /* __NXT_SENSING_MOTOR_HPP__ */