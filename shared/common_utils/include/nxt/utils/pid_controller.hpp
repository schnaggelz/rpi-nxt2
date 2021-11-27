/*******************************************************************************
* Copyright (C) 2021 Timon Reich
*
* This file is part of rpi-nxt2 experiment.
*
* Remote control application. Brick is controlled by a Raspberry Pi.
*
* License notes see LICENSE.txt
*******************************************************************************/

#ifndef __NXT_PID_CONTROLLER_HPP__
#define __NXT_PID_CONTROLLER_HPP__

class PIDController
{
  public:
    PID(double sample_rate, double max_value, double min_value, double pid_Kp,
        double pid_Kd, double pid_Ki);

    // Returns the manipulated variable given a set_point and the current
    // process value
    double calculate(double set_point, double process_value);
};

#endif // __NXT_PID_CONTROLLER_HPP__
