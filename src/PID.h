#pragma once

#include <BasicLinearAlgebra.h>
#ifndef USE_SIM
#include <Streaming.h>
#endif

struct PDGains {
    float kp;
    float kd;
};

struct PDGains3x3 {
    BLA::Matrix<3, 3> kp;
    BLA::Matrix<3, 3> kd;
};

void PD(float &torque_command, float measurement_pos, float measurement_vel,
        float reference_pos, float reference_vel, PDGains gains);
#ifndef USE_SIM
Print &operator<<(Print &stream, const PDGains &gains);
#endif
BLA::Matrix<3> PDControl3(BLA::Matrix<3> measured_position,
                          BLA::Matrix<3> measured_velocity,
                          BLA::Matrix<3> reference_position,
                          BLA::Matrix<3> reference_velocity, PDGains3x3 gains);