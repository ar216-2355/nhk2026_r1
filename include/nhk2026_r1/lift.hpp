#pragma once
#include <algorithm>
#include <cmath>
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
#include "nhk2026_r1/r1_constants.hpp"

enum class SystemMode {
    EMERGENCY,
    HOMING,
    HOMING_ASCEND,
    DRIVE
};

void append_motor_command(std::vector<robomas_interfaces::msg::MotorCommand> &motors, int id, int mode,float target) {
    robomas_interfaces::msg::MotorCommand command;
    command.motor_id = id;
    command.mode = mode;
    command.target = target;
    motors.push_back(command);
}

SystemMode lift_state[4] = {SystemMode::EMERGENCY, SystemMode::EMERGENCY, SystemMode::EMERGENCY, SystemMode::EMERGENCY}; // 4つのモーターの状態を管理
float maxpos = 25585.0f; // 昇降の最大位置
float minpos = 0.0f; // 昇降の最小位置(3番/4番)
float rb_rf_maxpos = 0.0f; // 11番/12番の最大位置
float rb_rf_minpos = -25585.0f; // 11番/12番の最小位置
float homing_current_threshold = 1500.0f; // ホーミングとみなす電流の閾値[mA]
float homing_ascend_time = 0.5f; // ホーミング完了後に少し上昇させる時間
float homing_backoff[4] = {100.0f, 100.0f, 100.0f, 100.0f}; // ホーミング完了後に壁から離す量
float homing_offset[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // position=0 の基準位置
constexpr int homing_current_confirm_cycles = 5; // 連続判定回数(10ms*5=50ms)
int homing_current_over_count[4] = {0, 0, 0, 0};

constexpr float lift_control_period_sec = 0.01f;
constexpr float lift_max_velocity_rpm = 100.0f;
constexpr float lift_max_accel_rpm_per_sec = 300.0f;
constexpr float lift_degrees_per_rpm = 6.0f;
constexpr float lift_position_tolerance = 5.0f;
constexpr float lift_velocity_tolerance_rpm = 1.0f;

float lift_profile_target[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float lift_profile_velocity_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};

inline void reset_lift_profile() {
    lift_profile_target[0] = 0.0f;
    lift_profile_target[1] = 0.0f;
    lift_profile_target[2] = 0.0f;
    lift_profile_target[3] = 0.0f;
    lift_profile_velocity_rpm[0] = 0.0f;
    lift_profile_velocity_rpm[1] = 0.0f;
    lift_profile_velocity_rpm[2] = 0.0f;
    lift_profile_velocity_rpm[3] = 0.0f;
}

inline float update_lift_trapezoid(float final_target, float &profile_position, float &profile_velocity_rpm) {
    const float position_error = final_target - profile_position;

    if(std::fabs(position_error) <= lift_position_tolerance && std::fabs(profile_velocity_rpm) <= lift_velocity_tolerance_rpm) {
        profile_position = final_target;
        profile_velocity_rpm = 0.0f;
        return profile_position;
    }

    const float desired_velocity_rpm = (position_error > 0.0f ? lift_max_velocity_rpm : -lift_max_velocity_rpm);
    const float profile_velocity_deg_per_sec = profile_velocity_rpm * lift_degrees_per_rpm;
    const float max_accel_deg_per_sec2 = lift_max_accel_rpm_per_sec * lift_degrees_per_rpm;
    const float stopping_distance = (profile_velocity_deg_per_sec * profile_velocity_deg_per_sec) / (2.0f * max_accel_deg_per_sec2);

    if(std::fabs(position_error) <= stopping_distance) {
        if(profile_velocity_rpm > 0.0f) {
            profile_velocity_rpm = std::max(0.0f, profile_velocity_rpm - lift_max_accel_rpm_per_sec * lift_control_period_sec);
        } else if(profile_velocity_rpm < 0.0f) {
            profile_velocity_rpm = std::min(0.0f, profile_velocity_rpm + lift_max_accel_rpm_per_sec * lift_control_period_sec);
        }
    } else {
        if(desired_velocity_rpm > profile_velocity_rpm) {
            profile_velocity_rpm = std::min(desired_velocity_rpm, profile_velocity_rpm + lift_max_accel_rpm_per_sec * lift_control_period_sec);
        } else {
            profile_velocity_rpm = std::max(desired_velocity_rpm, profile_velocity_rpm - lift_max_accel_rpm_per_sec * lift_control_period_sec);
        }
    }

    profile_position += profile_velocity_rpm * lift_degrees_per_rpm * lift_control_period_sec;

    if((final_target - profile_position) * position_error <= 0.0f) {
        profile_position = final_target;
        profile_velocity_rpm = 0.0f;
    }

    return profile_position;
}

inline void homing_lift(){
    lift_state[0] = SystemMode::HOMING;
    lift_state[1] = SystemMode::HOMING;
    lift_state[2] = SystemMode::HOMING;
    lift_state[3] = SystemMode::HOMING;
    homing_current_over_count[0] = 0;
    homing_current_over_count[1] = 0;
    homing_current_over_count[2] = 0;
    homing_current_over_count[3] = 0;
    reset_lift_profile();
}

inline bool set_lift_position(float position, float LFpos_fb, float LBpos_fb, float RBpos_fb, float RFpos_fb, float LFcur_fb, float LBcur_fb, float RBcur_fb, float RFcur_fb, robomas_interfaces::msg::RobomasPacket& packet) {
    if((lift_state[0] == SystemMode::EMERGENCY) || (lift_state[1] == SystemMode::EMERGENCY) || (lift_state[2] == SystemMode::EMERGENCY) || (lift_state[3] == SystemMode::EMERGENCY)) {
        append_motor_command(packet.motors, MotorId::LIFT_LF, Mode::CURRENT, 0.0f);
        append_motor_command(packet.motors, MotorId::LIFT_RF, Mode::CURRENT, 0.0f);
        append_motor_command(packet.motors, MotorId::LIFT_LB, Mode::CURRENT, 0.0f);
        append_motor_command(packet.motors, MotorId::LIFT_RB, Mode::CURRENT, 0.0f);
    } else if((lift_state[0] == SystemMode::HOMING) || (lift_state[1] == SystemMode::HOMING) || (lift_state[2] == SystemMode::HOMING) || (lift_state[3] == SystemMode::HOMING)) {
        if(lift_state[0] == SystemMode::HOMING) {
            if(std::fabs(LFcur_fb) > homing_current_threshold) {
                homing_current_over_count[0]++;
                if(homing_current_over_count[0] >= homing_current_confirm_cycles) {
                    lift_state[0] = SystemMode::HOMING_ASCEND;
                    homing_offset[0] = LFpos_fb + homing_backoff[0]; // LF/LBはホーミング方向(-)と逆向き(+)へ退避
                    append_motor_command(packet.motors, MotorId::LIFT_LF, Mode::CURRENT, 0.0f);
                } else {
                    append_motor_command(packet.motors, MotorId::LIFT_LF, Mode::VELOCITY, -100.0f);
                }
            } else {
                homing_current_over_count[0] = 0;
                append_motor_command(packet.motors, MotorId::LIFT_LF, Mode::VELOCITY, -100.0f);
            }
        }
        if(lift_state[1] == SystemMode::HOMING) {
            if(std::fabs(LBcur_fb) > homing_current_threshold) {
                homing_current_over_count[1]++;
                if(homing_current_over_count[1] >= homing_current_confirm_cycles) {
                    lift_state[1] = SystemMode::HOMING_ASCEND;
                    homing_offset[1] = LBpos_fb + homing_backoff[1]; // LF/LBはホーミング方向(-)と逆向き(+)へ退避
                    append_motor_command(packet.motors, MotorId::LIFT_LB, Mode::CURRENT, 0.0f);
                } else {
                    append_motor_command(packet.motors, MotorId::LIFT_LB, Mode::VELOCITY, -100.0f);
                }
            } else {
                homing_current_over_count[1] = 0;
                append_motor_command(packet.motors, MotorId::LIFT_LB, Mode::VELOCITY, -100.0f);
            }
        }
        if(lift_state[2] == SystemMode::HOMING) {
            if(std::fabs(RBcur_fb) > homing_current_threshold) {
                homing_current_over_count[2]++;
                if(homing_current_over_count[2] >= homing_current_confirm_cycles) {
                    lift_state[2] = SystemMode::HOMING_ASCEND;
                    homing_offset[2] = RBpos_fb - homing_backoff[2]; // RB/RFはホーミング方向(+)と逆向き(-)へ退避
                    append_motor_command(packet.motors, MotorId::LIFT_RB, Mode::CURRENT, 0.0f);
                } else {
                    append_motor_command(packet.motors, MotorId::LIFT_RB, Mode::VELOCITY,  100.0f);
                }
            } else {
                homing_current_over_count[2] = 0;
                append_motor_command(packet.motors, MotorId::LIFT_RB, Mode::VELOCITY,  100.0f);
            }
        }
        if(lift_state[3] == SystemMode::HOMING) {
            if(std::fabs(RFcur_fb) > homing_current_threshold) {
                homing_current_over_count[3]++;
                if(homing_current_over_count[3] >= homing_current_confirm_cycles) {
                    lift_state[3] = SystemMode::HOMING_ASCEND;
                    homing_offset[3] = RFpos_fb - homing_backoff[3]; // RB/RFはホーミング方向(+)と逆向き(-)へ退避
                    append_motor_command(packet.motors, MotorId::LIFT_RF, Mode::CURRENT, 0.0f);
                } else {
                    append_motor_command(packet.motors, MotorId::LIFT_RF, Mode::VELOCITY,  100.0f);
                }
            } else {
                homing_current_over_count[3] = 0;
                append_motor_command(packet.motors, MotorId::LIFT_RF, Mode::VELOCITY,  100.0f);
            }
        }
    } else if((lift_state[0] == SystemMode::HOMING_ASCEND) && (lift_state[1] == SystemMode::HOMING_ASCEND) && (lift_state[2] == SystemMode::HOMING_ASCEND) && (lift_state[3] == SystemMode::HOMING_ASCEND)) {
        lift_state[0] = SystemMode::DRIVE;
        lift_state[1] = SystemMode::DRIVE;
        lift_state[2] = SystemMode::DRIVE;
        lift_state[3] = SystemMode::DRIVE;
        lift_profile_target[0] = LFpos_fb;
        lift_profile_target[1] = LBpos_fb;
        lift_profile_target[2] = RBpos_fb;
        lift_profile_target[3] = RFpos_fb;
        lift_profile_velocity_rpm[0] = 0.0f;
        lift_profile_velocity_rpm[1] = 0.0f;
        lift_profile_velocity_rpm[2] = 0.0f;
        lift_profile_velocity_rpm[3] = 0.0f;
    } else if((lift_state[0] == SystemMode::DRIVE) && (lift_state[1] == SystemMode::DRIVE) && (lift_state[2] == SystemMode::DRIVE) && (lift_state[3] == SystemMode::DRIVE)) {
        const float target_LF = std::clamp(homing_offset[0] + position, minpos, maxpos);
        const float target_LB = std::clamp(homing_offset[1] + position, minpos, maxpos);
        const float target_RB = std::clamp(homing_offset[2] - position, rb_rf_minpos, rb_rf_maxpos);
        const float target_RF = std::clamp(homing_offset[3] - position, rb_rf_minpos, rb_rf_maxpos);

        const float profile_LF = update_lift_trapezoid(target_LF, lift_profile_target[0], lift_profile_velocity_rpm[0]);
        const float profile_LB = update_lift_trapezoid(target_LB, lift_profile_target[1], lift_profile_velocity_rpm[1]);
        const float profile_RB = update_lift_trapezoid(target_RB, lift_profile_target[2], lift_profile_velocity_rpm[2]);
        const float profile_RF = update_lift_trapezoid(target_RF, lift_profile_target[3], lift_profile_velocity_rpm[3]);

        append_motor_command(packet.motors, MotorId::LIFT_LF, Mode::POSITION, profile_LF);
        append_motor_command(packet.motors, MotorId::LIFT_LB, Mode::POSITION, profile_LB);
        append_motor_command(packet.motors, MotorId::LIFT_RB, Mode::POSITION, profile_RB);
        append_motor_command(packet.motors, MotorId::LIFT_RF, Mode::POSITION, profile_RF);
        if(std::abs(target_LF - LFpos_fb) < 10.0f &&
           std::abs(target_LB - LBpos_fb) < 10.0f &&
           std::abs(target_RB - RBpos_fb) < 10.0f &&
           std::abs(target_RF - RFpos_fb) < 10.0f) {
            return true; // 目標位置に到達したとみなす
        }
    } else {
        reset_lift_profile();
    }
    return false;
}