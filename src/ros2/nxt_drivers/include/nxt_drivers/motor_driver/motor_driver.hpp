#pragma once

namespace nxt2
{
class MotorDriver
{
   public:
    MotorDriver();
    void initialize();
    void control(int speed, bool direction);
};
}  // namespace nxt2
