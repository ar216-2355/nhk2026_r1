#pragma once
#include <algorithm>
#include <cmath>
#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
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

// Control parameters
constexpr float BOOK_STRETCH_MIN_POS = -14421.0f;
constexpr float BOOK_STRETCH_MAX_POS = 0.0f;
constexpr float BOOK_STRETCH_HOMING_VELOCITY = 100.0f;  // 正転でホーミング
constexpr float BOOK_STRETCH_HOMING_CURRENT_THRESHOLD = 3000.0f;  // mA - 限界検出しきい値
constexpr float BOOK_STRETCH_HOMING_DEBOUNCE_CYCLES = 5;  // 電流しきい値確認のデバウンス周期数

inline void set_book_stretch(uint8_t system_state, float position, float pos_fb, float motor_current_ma, robomas_interfaces::msg::RobomasPacket& packet) {
    // Helper lambda to append motor command
    auto append_command = [&packet](int id, int mode, float target) {
        robomas_interfaces::msg::MotorCommand command;
        command.motor_id = id;
        command.mode = mode;
        command.target = target;
        packet.motors.push_back(command);
    };

    // READY -> DRIVE edge detection: ホーミング開始
    if (book_stretch_prev_system_state == 1 && system_state == 2) {  // READY -> DRIVE transition
        book_stretch_state = BookStretchMode::HOMING;
        book_stretch_offset = 0.0f;
        homing_current_count = 0;
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
                // ホーミング完了：現在位置をオフセットとして記録
                book_stretch_offset = pos_fb;
                book_stretch_state = BookStretchMode::DRIVE;
                homing_current_count = 0;
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
        // Position control: offset基準に相対位置制御
        float target_pos = book_stretch_offset + position;
        target_pos = std::clamp(target_pos, BOOK_STRETCH_MIN_POS, BOOK_STRETCH_MAX_POS);
        
        const float position_error = target_pos - pos_fb;
        const float target_current = std::clamp(position_error * 0.1f, -10000.0f, 10000.0f);
        append_command(MotorId::BOOK_STRETCH, Mode::CURRENT, target_current);
    }
    // EMERGENCY mode
    else {
        book_stretch_state = BookStretchMode::EMERGENCY;
        append_command(MotorId::BOOK_STRETCH, Mode::CURRENT, 0.0f);
    }
}
