#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>

#include "robomas_interfaces/msg/robomas_packet.hpp"
#include "robomas_interfaces/msg/motor_command.hpp"
#include "robomas_interfaces/msg/can_frame.hpp"
#include "nhk2026_r1/r1_constants.hpp"

enum class PoleStretchMode {
	EMERGENCY,
	HOMING,
	HOMING_BACKOFF,
	DRIVE
};

// POLE_STRETCH state variables
PoleStretchMode pole_stretch_state = PoleStretchMode::EMERGENCY;
float pole_stretch_offset = 0.0f;
uint8_t pole_stretch_prev_system_state = 0;
int pole_homing_current_count = 0;
float pole_stretch_profile_target = 0.0f;
float pole_stretch_profile_velocity_rpm = 0.0f;
float pole_stretch_backoff_target = 0.0f;
float pole_homing_contact_position = 0.0f;

// Control parameters
constexpr float POLE_STRETCH_MIN_POS = 0.0f;
constexpr float POLE_STRETCH_MAX_POS = 27500.0f;
constexpr float POLE_STRETCH_HOMING_VELOCITY = -100.0f;  // 逆転でホーミング
constexpr float POLE_STRETCH_HOMING_CURRENT_THRESHOLD = 3000.0f;  // mA
constexpr float POLE_STRETCH_HOMING_DEBOUNCE_CYCLES = 5;
constexpr float POLE_STRETCH_HOMING_BACKOFF = 720.0f;
constexpr float POLE_STRETCH_CONTROL_PERIOD_SEC = 0.01f;
constexpr float POLE_STRETCH_MAX_VELOCITY_RPM = 5000.0f;
constexpr float POLE_STRETCH_MAX_ACCEL_RPM_PER_SEC = 4800.0f;
constexpr float POLE_STRETCH_DEGREES_PER_RPM = 6.0f;
constexpr float POLE_STRETCH_POSITION_TOLERANCE = 5.0f;
constexpr float POLE_STRETCH_VELOCITY_TOLERANCE_RPM = 1.0f;

inline uint16_t prev_pole_stretch_angle = 0xFFFF;

inline void reset_pole_stretch_profile() {
	pole_stretch_profile_target = 0.0f;
	pole_stretch_profile_velocity_rpm = 0.0f;
}

inline float update_pole_stretch_trapezoid(float final_target) {
	const float position_error = final_target - pole_stretch_profile_target;

	if (std::fabs(position_error) <= POLE_STRETCH_POSITION_TOLERANCE &&
		std::fabs(pole_stretch_profile_velocity_rpm) <= POLE_STRETCH_VELOCITY_TOLERANCE_RPM) {
		pole_stretch_profile_target = final_target;
		pole_stretch_profile_velocity_rpm = 0.0f;
		return pole_stretch_profile_target;
	}

	const float desired_velocity_rpm =
		(position_error > 0.0f ? POLE_STRETCH_MAX_VELOCITY_RPM : -POLE_STRETCH_MAX_VELOCITY_RPM);
	const float profile_velocity_deg_per_sec = pole_stretch_profile_velocity_rpm * POLE_STRETCH_DEGREES_PER_RPM;
	const float max_accel_deg_per_sec2 = POLE_STRETCH_MAX_ACCEL_RPM_PER_SEC * POLE_STRETCH_DEGREES_PER_RPM;
	const float stopping_distance =
		(profile_velocity_deg_per_sec * profile_velocity_deg_per_sec) / (2.0f * max_accel_deg_per_sec2);

	if (std::fabs(position_error) <= stopping_distance) {
		if (pole_stretch_profile_velocity_rpm > 0.0f) {
			pole_stretch_profile_velocity_rpm = std::max(
				0.0f,
				pole_stretch_profile_velocity_rpm - POLE_STRETCH_MAX_ACCEL_RPM_PER_SEC * POLE_STRETCH_CONTROL_PERIOD_SEC);
		} else if (pole_stretch_profile_velocity_rpm < 0.0f) {
			pole_stretch_profile_velocity_rpm = std::min(
				0.0f,
				pole_stretch_profile_velocity_rpm + POLE_STRETCH_MAX_ACCEL_RPM_PER_SEC * POLE_STRETCH_CONTROL_PERIOD_SEC);
		}
	} else {
		if (desired_velocity_rpm > pole_stretch_profile_velocity_rpm) {
			pole_stretch_profile_velocity_rpm = std::min(
				desired_velocity_rpm,
				pole_stretch_profile_velocity_rpm + POLE_STRETCH_MAX_ACCEL_RPM_PER_SEC * POLE_STRETCH_CONTROL_PERIOD_SEC);
		} else {
			pole_stretch_profile_velocity_rpm = std::max(
				desired_velocity_rpm,
				pole_stretch_profile_velocity_rpm - POLE_STRETCH_MAX_ACCEL_RPM_PER_SEC * POLE_STRETCH_CONTROL_PERIOD_SEC);
		}
	}

	pole_stretch_profile_target +=
		pole_stretch_profile_velocity_rpm * POLE_STRETCH_DEGREES_PER_RPM * POLE_STRETCH_CONTROL_PERIOD_SEC;

	if ((final_target - pole_stretch_profile_target) * position_error <= 0.0f) {
		pole_stretch_profile_target = final_target;
		pole_stretch_profile_velocity_rpm = 0.0f;
	}

	return pole_stretch_profile_target;
}

inline void set_pole_stretch(uint8_t system_state, float position, float pos_fb, float motor_current_ma, robomas_interfaces::msg::RobomasPacket& packet) {
	auto append_command = [&packet](int id, int mode, float target) {
		robomas_interfaces::msg::MotorCommand command;
		command.motor_id = id;
		command.mode = mode;
		command.target = target;
		packet.motors.push_back(command);
	};

	// 非DRIVE -> DRIVE edge detection: ホーミング開始
	if (pole_stretch_prev_system_state != 2 && system_state == 2) {
		pole_stretch_state = PoleStretchMode::HOMING;
		pole_stretch_offset = 0.0f;
		pole_homing_current_count = 0;
		reset_pole_stretch_profile();
	}
	pole_stretch_prev_system_state = system_state;

	if (pole_stretch_state == PoleStretchMode::HOMING) {
		append_command(MotorId::POLE_STRETCH, Mode::VELOCITY, POLE_STRETCH_HOMING_VELOCITY);

		if (std::fabs(motor_current_ma) > POLE_STRETCH_HOMING_CURRENT_THRESHOLD) {
			pole_homing_current_count++;
			if (pole_homing_current_count >= POLE_STRETCH_HOMING_DEBOUNCE_CYCLES) {
				// 壁接触点を原点(offset)として記録し、そこから固定量だけ戻す。
				pole_homing_contact_position = pos_fb;
				pole_stretch_offset = pole_homing_contact_position;
				pole_stretch_backoff_target = std::clamp(
					pole_stretch_offset + POLE_STRETCH_HOMING_BACKOFF,
					POLE_STRETCH_MIN_POS,
					POLE_STRETCH_MAX_POS);
				pole_stretch_state = PoleStretchMode::HOMING_BACKOFF;
				pole_homing_current_count = 0;
				pole_stretch_profile_target = pos_fb;
				pole_stretch_profile_velocity_rpm = 0.0f;
			}
		} else if (pole_homing_current_count > 0) {
			pole_homing_current_count = 0;
		}
	} else if (pole_stretch_state == PoleStretchMode::HOMING_BACKOFF && system_state == 2) {
		const float profile_pos = update_pole_stretch_trapezoid(pole_stretch_backoff_target);
		append_command(MotorId::POLE_STRETCH, Mode::POSITION, profile_pos);

		if (std::fabs(pole_stretch_backoff_target - pos_fb) <= POLE_STRETCH_POSITION_TOLERANCE) {
			// 原点は壁接触点のまま、以降は位置指令を相対値として扱う。
			pole_stretch_profile_target = pos_fb;
			pole_stretch_profile_velocity_rpm = 0.0f;
			pole_stretch_state = PoleStretchMode::DRIVE;
		}
	} else if (pole_stretch_state == PoleStretchMode::DRIVE && system_state == 2) {
		const float relative_target = std::clamp(position, POLE_STRETCH_MIN_POS, POLE_STRETCH_MAX_POS);
		const float target_pos = pole_stretch_offset + relative_target;

		const float profile_pos = update_pole_stretch_trapezoid(target_pos);
		append_command(MotorId::POLE_STRETCH, Mode::POSITION, profile_pos);
	} else {
		pole_stretch_state = PoleStretchMode::EMERGENCY;
		reset_pole_stretch_profile();
		append_command(MotorId::POLE_STRETCH, Mode::CURRENT, 0.0f);
	}
}

// 21度（←縦）
// 111度（↓横）
// 201度（→縦）

inline void servo_pole_stretch(
	uint16_t angle,
	const rclcpp::Publisher<robomas_interfaces::msg::CanFrame>::SharedPtr& can_pub) {
	if (!can_pub) {
		return;
	}

	if (prev_pole_stretch_angle == angle) {
		return; // 角度が前回と同じならコマンドを送らない
	} else {
		prev_pole_stretch_angle = angle;
	}

	const uint16_t raw = static_cast<uint16_t>(std::lround(angle * 10.18f));
	const uint8_t a = static_cast<uint8_t>(raw & 0xFF);         // 下位8ビット
	const uint8_t b = static_cast<uint8_t>((raw >> 8) & 0xFF);  // 上位8ビット

	auto msg = robomas_interfaces::msg::CanFrame();
	msg.id = 0x302;
	msg.dlc = 8;
	msg.data = {b, a, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00};
	can_pub->publish(msg);
}

inline void denjiben(uint8_t valve_cmd, const rclcpp::Publisher<robomas_interfaces::msg::CanFrame>::SharedPtr& can_pub) {
    if (!can_pub) {
        return;
    }
    auto msg = robomas_interfaces::msg::CanFrame();
    msg.id = 0x400;
    msg.dlc = 1;
	msg.data = {valve_cmd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    can_pub->publish(msg);

}


