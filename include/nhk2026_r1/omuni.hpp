#pragma once
#include <cmath>
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
#include "nhk2026_r1/r1_constants.hpp"

inline void set_omni_velocity(float vx, float vy, float omega, robomas_interfaces::msg::RobomasPacket& packet) {

    float v1 =  vx + vy + omega;
    float v2 = -vx + vy + omega;
    float v3 = -vx - vy + omega;
    float v4 =  vx - vy + omega;
    
    robomas_interfaces::msg::MotorCommand cmd;

    cmd.motor_id = MotorId::OMUNI_LF;
    cmd.mode = Mode::VELOCITY;
    cmd.target = v1;
    packet.motors.push_back(cmd);

    cmd.motor_id = MotorId::OMUNI_LB;
    cmd.mode = Mode::VELOCITY;
    cmd.target = v2;
    packet.motors.push_back(cmd);

    cmd.motor_id = MotorId::OMUNI_RB;
    cmd.mode = Mode::VELOCITY;
    cmd.target = v3;
    packet.motors.push_back(cmd);

    cmd.motor_id = MotorId::OMUNI_RF;
    cmd.mode = Mode::VELOCITY;
    cmd.target = v4;
    packet.motors.push_back(cmd);
}