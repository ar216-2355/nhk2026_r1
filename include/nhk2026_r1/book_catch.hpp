#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>

#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
#include "robomas_interfaces/msg/can_frame.hpp"
#include "nhk2026_r1/r1_constants.hpp"

enum class BookStretchMode {
    EMERGENCY,
    HOMING,
    DRIVE
};

// BOOK_STRETCH state variables
BookStretchMode book_stretch_state = BookStretchMode::EMERGENCY;
float book_stretch_offset = 0.0f;
uint8_t book_stretch_prev_system_state = 0;
int homing_current_count = 0;
float book_stretch_profile_target = 0.0f;
float book_stretch_profile_velocity_rpm = 0.0f;

// Control parameters
constexpr float BOOK_STRETCH_MIN_POS = -64174.0f;
constexpr float BOOK_STRETCH_MAX_POS = 0.0f;
constexpr float BOOK_STRETCH_HOMING_VELOCITY = 100.0f;  // 正転でホーミング
constexpr float BOOK_STRETCH_HOMING_CURRENT_THRESHOLD = 3000.0f;  // mA - 限界検出しきい値
constexpr float BOOK_STRETCH_HOMING_DEBOUNCE_CYCLES = 5;  // 電流しきい値確認のデバウンス周期数
constexpr float BOOK_STRETCH_HOMING_BACKOFF = 36.0f;
constexpr float BOOK_STRETCH_CONTROL_PERIOD_SEC = 0.01f;
constexpr float BOOK_STRETCH_MAX_VELOCITY_RPM = 5000.0f;
constexpr float BOOK_STRETCH_MAX_ACCEL_RPM_PER_SEC = 4800.0f;
constexpr float BOOK_STRETCH_DEGREES_PER_RPM = 6.0f;
constexpr float BOOK_STRETCH_POSITION_TOLERANCE = 5.0f;
constexpr float BOOK_STRETCH_VELOCITY_TOLERANCE_RPM = 1.0f;

inline uint16_t prev_book_stretch_angle = 0xFFFF;

inline void reset_book_stretch_profile() {
    book_stretch_profile_target = 0.0f;
    book_stretch_profile_velocity_rpm = 0.0f;
}

inline float update_book_stretch_trapezoid(float final_target) {
    const float position_error = final_target - book_stretch_profile_target;

    if (std::fabs(position_error) <= BOOK_STRETCH_POSITION_TOLERANCE &&
        std::fabs(book_stretch_profile_velocity_rpm) <= BOOK_STRETCH_VELOCITY_TOLERANCE_RPM) {
        book_stretch_profile_target = final_target;
        book_stretch_profile_velocity_rpm = 0.0f;
        return book_stretch_profile_target;
    }

    const float desired_velocity_rpm =
        (position_error > 0.0f ? BOOK_STRETCH_MAX_VELOCITY_RPM : -BOOK_STRETCH_MAX_VELOCITY_RPM);
    const float profile_velocity_deg_per_sec = book_stretch_profile_velocity_rpm * BOOK_STRETCH_DEGREES_PER_RPM;
    const float max_accel_deg_per_sec2 = BOOK_STRETCH_MAX_ACCEL_RPM_PER_SEC * BOOK_STRETCH_DEGREES_PER_RPM;
    const float stopping_distance =
        (profile_velocity_deg_per_sec * profile_velocity_deg_per_sec) / (2.0f * max_accel_deg_per_sec2);

    if (std::fabs(position_error) <= stopping_distance) {
        if (book_stretch_profile_velocity_rpm > 0.0f) {
            book_stretch_profile_velocity_rpm = std::max(
                0.0f,
                book_stretch_profile_velocity_rpm - BOOK_STRETCH_MAX_ACCEL_RPM_PER_SEC * BOOK_STRETCH_CONTROL_PERIOD_SEC);
        } else if (book_stretch_profile_velocity_rpm < 0.0f) {
            book_stretch_profile_velocity_rpm = std::min(
                0.0f,
                book_stretch_profile_velocity_rpm + BOOK_STRETCH_MAX_ACCEL_RPM_PER_SEC * BOOK_STRETCH_CONTROL_PERIOD_SEC);
        }
    } else {
        if (desired_velocity_rpm > book_stretch_profile_velocity_rpm) {
            book_stretch_profile_velocity_rpm = std::min(
                desired_velocity_rpm,
                book_stretch_profile_velocity_rpm + BOOK_STRETCH_MAX_ACCEL_RPM_PER_SEC * BOOK_STRETCH_CONTROL_PERIOD_SEC);
        } else {
            book_stretch_profile_velocity_rpm = std::max(
                desired_velocity_rpm,
                book_stretch_profile_velocity_rpm - BOOK_STRETCH_MAX_ACCEL_RPM_PER_SEC * BOOK_STRETCH_CONTROL_PERIOD_SEC);
        }
    }

    book_stretch_profile_target +=
        book_stretch_profile_velocity_rpm * BOOK_STRETCH_DEGREES_PER_RPM * BOOK_STRETCH_CONTROL_PERIOD_SEC;

    if ((final_target - book_stretch_profile_target) * position_error <= 0.0f) {
        book_stretch_profile_target = final_target;
        book_stretch_profile_velocity_rpm = 0.0f;
    }

    return book_stretch_profile_target;
}

inline void set_book_stretch(uint8_t system_state, float position, float pos_fb, float motor_current_ma, robomas_interfaces::msg::RobomasPacket& packet) {
    // Helper lambda to append motor command
    auto append_command = [&packet](int id, int mode, float target) {
        robomas_interfaces::msg::MotorCommand command;
        command.motor_id = id;
        command.mode = mode;
        command.target = target;
        packet.motors.push_back(command);
    };

    // 非DRIVE -> DRIVE edge detection: ホーミング開始
    if (book_stretch_prev_system_state != 2 && system_state == 2) {
        book_stretch_state = BookStretchMode::HOMING;
        book_stretch_offset = 0.0f;
        homing_current_count = 0;
        reset_book_stretch_profile();
    }
    book_stretch_prev_system_state = system_state;

    // HOMING mode
    if (book_stretch_state == BookStretchMode::HOMING) {
        // Homing: 正転方向（正の電流指令）で移動
        append_command(MotorId::BOOK_STRETCH, Mode::VELOCITY, BOOK_STRETCH_HOMING_VELOCITY);
        
        // 電流しきい値確認（デバウンス付き）
        if (motor_current_ma > BOOK_STRETCH_HOMING_CURRENT_THRESHOLD) {
            homing_current_count++;
            if (homing_current_count >= BOOK_STRETCH_HOMING_DEBOUNCE_CYCLES) {
                // ホーミング完了：逆方向へ指定バックオフ量だけ戻した位置を基準オフセットとして記録
                book_stretch_offset = std::clamp(pos_fb - BOOK_STRETCH_HOMING_BACKOFF, BOOK_STRETCH_MIN_POS, BOOK_STRETCH_MAX_POS);
                book_stretch_state = BookStretchMode::DRIVE;
                homing_current_count = 0;
                book_stretch_profile_target = pos_fb;
                book_stretch_profile_velocity_rpm = 0.0f;
            }
        } else {
            // 電流がしきい値以下に戻ったらカウントをリセット
            if (homing_current_count > 0) {
                homing_current_count = 0;
            }
        }
    }
    // DRIVE mode
    else if (book_stretch_state == BookStretchMode::DRIVE && system_state == 2) {  // DRIVE mode
        // Position control: offset基準に相対位置制御 + 台形プロファイル
        float target_pos = book_stretch_offset + position;
        target_pos = std::clamp(target_pos, BOOK_STRETCH_MIN_POS, BOOK_STRETCH_MAX_POS);

        const float profile_pos = update_book_stretch_trapezoid(target_pos);
        append_command(MotorId::BOOK_STRETCH, Mode::POSITION, profile_pos);
    }
    // EMERGENCY mode
    else {
        book_stretch_state = BookStretchMode::EMERGENCY;
        reset_book_stretch_profile();
        append_command(MotorId::BOOK_STRETCH, Mode::CURRENT, 0.0f);
    }
}

inline void servo_book_stretch(
    uint16_t angle,
    const rclcpp::Publisher<robomas_interfaces::msg::CanFrame>::SharedPtr& can_pub) {
    if (!can_pub) {
        return;
    }

    if (prev_book_stretch_angle == angle) {
        return; // 角度が前回と同じならコマンドを送らない
    } else {
        prev_book_stretch_angle = angle;
    }

    const uint16_t raw = static_cast<uint16_t>(std::lround(angle * 10.18f));
    const uint8_t a = static_cast<uint8_t>(raw & 0xFF);         // 下位8ビット
    const uint8_t b = static_cast<uint8_t>((raw >> 8) & 0xFF);  // 上位8ビット

    auto msg = robomas_interfaces::msg::CanFrame();
    msg.id = 0x301;
    msg.dlc = 8;
    msg.data = {b, a, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00};
    can_pub->publish(msg);
}
