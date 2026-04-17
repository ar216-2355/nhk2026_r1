#pragma once

#include <cstdint>

namespace Mode {
	constexpr uint8_t CURRENT  = 0;
	constexpr uint8_t VELOCITY = 1;
	constexpr uint8_t POSITION = 2;
	constexpr uint8_t DISABLE  = 3;
}

namespace MotorId {
	constexpr uint8_t OMUNI_LF = 1;
	constexpr uint8_t OMUNI_LB = 2;
	constexpr uint8_t OMUNI_RB = 3;
	constexpr uint8_t OMUNI_RF = 4;

	constexpr uint8_t ARM_RB = 5;
	constexpr uint8_t ARM_RF = 6;
	constexpr uint8_t BELT_B = 7;
	constexpr uint8_t BELT_F = 8;
	constexpr uint8_t ARM_LF = 9;
	constexpr uint8_t ARM_LB = 10;

	constexpr uint8_t ELEVATOR = 11;

	constexpr uint8_t LIFT_LF = 3;
	constexpr uint8_t LIFT_LB = 4;
	constexpr uint8_t LIFT_RB = 11;
	constexpr uint8_t LIFT_RF = 12;

	constexpr uint8_t BOOK_STRETCH = 13;
	constexpr uint8_t POLE_STRETCH = 5;
}

namespace Joy {
	constexpr int A = 0;
	constexpr int B = 1;
	constexpr int X = 2;
	constexpr int Y = 3;
	constexpr int LB = 4;
	constexpr int RB = 5;
	constexpr int BACK = 6;
	constexpr int START = 7;
	constexpr int HOME = 8;
	constexpr int L_STICK = 9;
	constexpr int R_STICK = 10;

	constexpr int L_STICK_X = 0;
	constexpr int L_STICK_Y = 1;
	constexpr int LT = 2;
	constexpr int R_STICK_X = 3;
	constexpr int R_STICK_Y = 4;
	constexpr int RT = 5;
	constexpr int DPAD_X = 6;
	constexpr int DPAD_Y = 7;
}

struct MotorData {
	float angle = 0.0;
	float velocity = 0.0;
	int16_t torque = 0;
};