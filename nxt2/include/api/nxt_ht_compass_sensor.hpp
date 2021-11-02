#ifndef __NXT_HT_COMPASS_SENSOR_HPP__
#define __NXT_HT_COMPASS_SENSOR_HPP__

#include "nxt_sensor.hpp"

namespace nxt
{
namespace ht
{
class CompassSensor : public Sensor
{
  public:
    CompassSensor(std::uint8_t port_number)
        : Sensor(port_number), _heading(0xFFFF)
    {
    }

    std::uint16_t getHeading()
    {
        return _heading;
    }

    void init() override;
    void read() override;
    void exit() override;

    bool calibrate();

  private:
    std::int16_t _heading;
};

} // namespace ht
} // namespace nxt

#endif /* __NXT_HT_COMPASS_SENSOR_HPP__ */
