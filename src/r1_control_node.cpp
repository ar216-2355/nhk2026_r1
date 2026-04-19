#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <robomas_interfaces/msg/motor_command.hpp>
#include <robomas_interfaces/msg/robomas_packet.hpp>
#include "robomas_interfaces/msg/robomas_frame.hpp"
#include "robomas_interfaces/msg/can_frame.hpp"


#include "nhk2026_r1/r1_constants.hpp"
#include "nhk2026_r1/omuni.hpp"
#include "nhk2026_r1/lift.hpp"
#include "nhk2026_r1/book_catch.hpp"
#include "nhk2026_r1/pole_catch.hpp"

using namespace std::chrono_literals;

class R1ControlNode : public rclcpp::Node {
 public:
  R1ControlNode() : Node("r1_control") {
    cmd_pub_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>("/robomas/cmd", 10);
    can_pub_ = this->create_publisher<robomas_interfaces::msg::CanFrame>("/robomas/can_tx", 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&R1ControlNode::joy_callback, this, std::placeholders::_1));
    sub_feedback_ = this->create_subscription<robomas_interfaces::msg::RobomasFrame>(
        "/robomas/feedback", 
        10, 
        std::bind(&R1ControlNode::feedback_callback, this, std::placeholders::_1)
    );
    timer_ = this->create_wall_timer(10ms, std::bind(&R1ControlNode::control_loop_10ms, this));

    RCLCPP_INFO(this->get_logger(), "r1_control node started.");
  }

 private:
    MotorData current_motors_[16]; // 16台分のモーター状態を入れる棚
    uint8_t current_system_state_ = 0;      // 0:EMERGENCY, 1:READY, 2:DRIVE
     uint8_t prev_system_state_ = 0;

    float start_LF_omuni = 0;
    float current_LF_omuni = 0;
    float start_belt = 0;
    float current_belt = 0;
    bool kakuno_ok = false;
    bool prev_a_button_ = false;
    bool prev_b_button_ = false;
    bool prev_x_button_ = false;
    bool prev_y_button_ = false;
    bool prev_dpad_x_button_ = false;
    bool prev_dpad_y_button_ = false;
    float target_lift_position_ = 0.0f;
    float target_book_stretch_position_ = 0.0f;
    float target_pole_stretch_position_ = 0.0f;
    uint16_t target_book_angle = 0;
    uint16_t target_pole_angle = 0;
    float target_book_catch_current = 0.0f;
    uint8_t denjiben_catch = 0;

    float terbo_mode = 1.0f;

    int automaton_state = 0;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) { latest_joy_ = *msg; }

    void append_motor_command(std::vector<robomas_interfaces::msg::MotorCommand> &motors, int id, int mode,float target) const {
        robomas_interfaces::msg::MotorCommand command;
        command.motor_id = id;
        command.mode = mode;
        command.target = target;
        motors.push_back(command);
    }  

    void publish_stop(robomas_interfaces::msg::RobomasPacket &packet) const {
        append_motor_command(packet.motors, MotorId::OMUNI_LF, Mode::CURRENT, 0.0f);
        append_motor_command(packet.motors, MotorId::OMUNI_RF, Mode::CURRENT, 0.0f);
        append_motor_command(packet.motors, MotorId::OMUNI_LB, Mode::CURRENT, 0.0f);
        append_motor_command(packet.motors, MotorId::OMUNI_RB, Mode::CURRENT, 0.0f);
        append_motor_command(packet.motors, MotorId::LIFT_LF, Mode::CURRENT, 0.0f);
        append_motor_command(packet.motors, MotorId::LIFT_RF, Mode::CURRENT, 0.0f);
        append_motor_command(packet.motors, MotorId::LIFT_LB, Mode::CURRENT, 0.0f);
        append_motor_command(packet.motors, MotorId::LIFT_RB, Mode::CURRENT, 0.0f);
    }

    void feedback_callback(const robomas_interfaces::msg::RobomasFrame::SharedPtr msg) {
        current_system_state_ = msg->system_state;
        for (int i = 0; i < 16; i++) {
            current_motors_[i].angle    = msg->angle[i];
            current_motors_[i].velocity = msg->velocity[i];
            current_motors_[i].torque   = msg->current[i];
        }
    }

    void control_loop_10ms() {
        robomas_interfaces::msg::RobomasPacket packet;

        if (latest_joy_.axes.empty()) {
        publish_stop(packet);
        return;
        }

        if (current_system_state_ != 2) {
            target_lift_position_ = 0.0f;
            target_book_stretch_position_ = 0.0f;
            target_pole_stretch_position_ = 0.0f;
            automaton_state = 0;
        }

        if (latest_joy_.buttons.size() > Joy::X) {
            const bool a_pressed = latest_joy_.buttons[Joy::A];
            const bool b_pressed = latest_joy_.buttons[Joy::B];
            const bool x_pressed = latest_joy_.buttons[Joy::X];
            const bool y_pressed = latest_joy_.buttons[Joy::Y];
            const bool lb_pressed = latest_joy_.buttons[Joy::LB];
            const float dpad_x_pressed = latest_joy_.axes[Joy::DPAD_X];
            const float dpad_y_pressed = latest_joy_.axes[Joy::DPAD_Y];

            if (a_pressed && !prev_a_button_ &&
                lift_state[0] == SystemMode::DRIVE &&
                lift_state[1] == SystemMode::DRIVE &&
                lift_state[2] == SystemMode::DRIVE &&
                lift_state[3] == SystemMode::DRIVE &&
                book_stretch_state == BookStretchMode::DRIVE &&
                pole_stretch_state == PoleStretchMode::DRIVE &&
                current_system_state_ == 2) 
            {
                automaton_state++;
            }

            if (b_pressed && !prev_b_button_ &&
                lift_state[0] == SystemMode::DRIVE &&
                lift_state[1] == SystemMode::DRIVE &&
                lift_state[2] == SystemMode::DRIVE &&
                lift_state[3] == SystemMode::DRIVE &&
                book_stretch_state == BookStretchMode::DRIVE &&
                pole_stretch_state == PoleStretchMode::DRIVE &&
                current_system_state_ == 2) 
            {
                automaton_state--;
                if (automaton_state < 0) automaton_state = 0;
            }

            if(y_pressed && !prev_y_button_){
                target_book_stretch_position_ = -60000.0f; // ブックの把持の位置
            }

            if(lb_pressed) {
                terbo_mode = 1.5;
            }else{
                terbo_mode = 1.0;
            }

            if(dpad_y_pressed && !prev_dpad_y_button_) {
                if (latest_joy_.axes.size() > Joy::DPAD_Y) {
                    if(latest_joy_.axes[Joy::DPAD_Y] > 0.5f) {
                        target_book_catch_current = -0.25f * terbo_mode;
                    } else if(latest_joy_.axes[Joy::DPAD_Y] < -0.5f) {
                        target_book_catch_current = 0.25f * terbo_mode;
                    }
                }
            }
            if(dpad_x_pressed && !prev_dpad_x_button_) {
                if (latest_joy_.axes.size() > Joy::DPAD_X) {
                    if(latest_joy_.axes[Joy::DPAD_X] > 0.5f) {
                        target_book_catch_current = 0.0f;
                    } else if(latest_joy_.axes[Joy::DPAD_X] < -0.5f) {
                        target_book_catch_current = 0.0f;
                    }
                    
                }
            }

            switch (automaton_state) {
                case 0: // 初期値
                    target_lift_position_ = 1000.0f; // 昇降位置
                    
                    target_pole_stretch_position_ = 25000.0f; // ポールの把持の位置
                    target_pole_angle = 111U; // ポールの把持の角度
                    denjiben_catch = 0; // 電磁弁のコマンド
                    
                    target_book_stretch_position_ = -1000.0f; // ブックの把持の位置
                    target_book_angle = 42U; // ブックの把持の角度

                    break;
                case 1: //ブックの把持を横に、ポールの把持を伸ばす
                    target_pole_stretch_position_ = 1000.0f;
                    target_book_angle = 128U; // ブックの把持の角度
                    target_lift_position_ = 1000.0f;
                    target_pole_angle = 111U; // ポールの把持の角度
                    break;
                case 2: // 昇降をポール取得用に上げる
                    target_pole_angle = 21U; // ポールの把持の角度 new
                    target_lift_position_ = 20000.0f; // 昇降位置 new
                    denjiben_catch = 0; 
                    break;
                case 3: // ポールを取得する（開く）
                    denjiben_catch = 1; // 電磁弁のコマンドnew
                    break;
                case 4: // ポールを取得する（閉じる）
                    denjiben_catch = 0; // 電磁弁のコマンドnew
                    target_pole_angle = 21U;
                    target_lift_position_ = 20000.0f;
                    break;
                case 5: // 昇降を少し上げる
                    target_lift_position_ = 22000.0f; // 昇降位置 new
                    break;
                case 6: // ポールを180度回転する
                    target_pole_angle = 195U; // ポールの把持の角度 new
                    target_lift_position_ = 22000.0f; // 昇降位置
                    break;
                case 7: // 昇降を少し下げる
                    target_lift_position_ = 11000.0f; // 昇降位置 new
                    break;
                case 8: // 昇降を自由に動かせるようにしたい
                    target_book_stretch_position_ = -1000.0f; // ブックの把持の位置
                    if (latest_joy_.axes.size() > Joy::R_STICK_Y) {
                        constexpr float kLiftManualDeadzone = 0.15f;
                        constexpr float kLiftManualSpeedPerSec = 12000.0f;
                        float lift_input = latest_joy_.axes[Joy::R_STICK_Y];
                        if (std::fabs(lift_input) < kLiftManualDeadzone) {
                            lift_input = 0.0f;
                        }
                        target_lift_position_ += lift_input * kLiftManualSpeedPerSec * 0.01f;
                        target_lift_position_ = std::clamp(target_lift_position_, lift_min_relative_pos, lift_max_relative_pos);
                    }
                    break;
                case 9: // ブックの把持を伸ばして昇降を下げてポールの把持を引く
                    target_lift_position_ = 5500.0f; // 昇降位置 new
                    target_book_stretch_position_ = -60000.0f; // ブックの把持の位置
                    target_pole_stretch_position_ = 18000.0f; // ポールの把持の位置
                    target_book_angle = 128U; // ブックの把持の角度
                    break;
                case 10: // 把持を上向に回転する
                    target_book_angle = 42U; // ブックの把持の角度 new
                    target_book_stretch_position_ = -60000.0f; // ブックの把持の位置 new
                    break;
                case 11: // ブックの把持を縮める
                    target_book_stretch_position_ = -25000.0f; // ブックの把持の位置 new
                    target_lift_position_ = 6000.0f; // 昇降位置
                    break;
                case 12: // 昇降を一番上に上げる
                    target_lift_position_ = 25000.0f; // 昇降位置 new
                    break;
                case 13: // ブックの向きを横に
                    target_book_angle = 128U; // ブックの把持の角度
                    target_book_stretch_position_ = -28000.0f; // ブックの把持の位置
                    break;
                case 14: // ブックの把持を伸ばす
                    target_book_stretch_position_ = -60000.0f; // ブックの把持の位置
                    target_book_angle = 128U; // ブックの把持の角度
                    break;
                case 15: // ブックの把持を引いて上向にする
                    target_book_stretch_position_ = -28000.0f; // ブックの把持の位置
                    target_book_angle = 42U; // ブックの把持の角度
                    break;
                case 16: // ブックの把持を前向きにする
                    target_book_angle = 106U; // ブックの把持の角度 new
                    target_book_stretch_position_ = -28000.0f; // ブックの把持の位置
                    target_pole_stretch_position_ = 18000.0f;
                    break;
                case 17: // ブックの把持を縮めてポールの把持を伸ばす
                    target_book_stretch_position_ = -360.0f; // ブックの把持の位置
                    target_pole_stretch_position_ = 360.0f;
                    break;
                case 18:
                    target_lift_position_ = 0.0f; // 昇降位置
                    break;
                case 19:
                    if (latest_joy_.axes.size() > Joy::R_STICK_Y) {
                        constexpr float kLiftManualDeadzone = 0.15f;
                        constexpr float kLiftManualSpeedPerSec = 12000.0f;
                        float lift_input = latest_joy_.axes[Joy::R_STICK_Y];
                        if (std::fabs(lift_input) < kLiftManualDeadzone) {
                            lift_input = 0.0f;
                        }
                        target_lift_position_ += lift_input * kLiftManualSpeedPerSec * 0.01f;
                        target_lift_position_ = std::clamp(target_lift_position_, lift_min_relative_pos, lift_max_relative_pos);
                    }
                    break;
                default:
                    break;
            }
            prev_a_button_ = a_pressed;
            prev_b_button_ = b_pressed;
            prev_x_button_ = x_pressed;
            prev_y_button_ = y_pressed;
        }

        set_lift_position(
            current_system_state_,
            target_lift_position_,
            current_motors_[MotorId::LIFT_LF - 1].angle,
            current_motors_[MotorId::LIFT_LB - 1].angle,
            current_motors_[MotorId::LIFT_RB - 1].angle,
            current_motors_[MotorId::LIFT_RF - 1].angle,
            packet);

        set_book_stretch(
            current_system_state_,
            target_book_stretch_position_,
            current_motors_[MotorId::BOOK_STRETCH - 1].angle,
            current_motors_[MotorId::BOOK_STRETCH - 1].torque,
            packet);
        servo_book_stretch(target_book_angle, can_pub_);
        if (prev_system_state_ == 1 && current_system_state_ == 2) {
            start_can_send_book(can_pub_);
        }
        dc_book_catch(target_book_catch_current, can_pub_);

        set_pole_stretch(
            current_system_state_,
            target_pole_stretch_position_,
            current_motors_[MotorId::POLE_STRETCH - 1].angle,
            current_motors_[MotorId::POLE_STRETCH - 1].torque,
            packet);
        servo_pole_stretch(target_pole_angle, can_pub_);
        denjiben(denjiben_catch, can_pub_);

        float omuni_speed_multiplier = 1.0f;
        if (latest_joy_.buttons.size() > Joy::RB && latest_joy_.buttons[Joy::RB]) {
            omuni_speed_multiplier = 0.1f;  // RBが押されている間は速度を半分に
        }

        set_omni_velocity(
            latest_joy_.axes[Joy::L_STICK_X] * -3000 * omuni_speed_multiplier, 
            latest_joy_.axes[Joy::L_STICK_Y] * 3000 * omuni_speed_multiplier, 
            (latest_joy_.axes[Joy::LT] - latest_joy_.axes[Joy::RT]) * 1400.0f * omuni_speed_multiplier, 
            packet);
        
        cmd_pub_->publish(packet);
        prev_system_state_ = current_system_state_;
    }



  sensor_msgs::msg::Joy latest_joy_;
  rclcpp::Publisher<robomas_interfaces::msg::RobomasPacket>::SharedPtr cmd_pub_;
    rclcpp::Publisher<robomas_interfaces::msg::CanFrame>::SharedPtr can_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<robomas_interfaces::msg::RobomasFrame>::SharedPtr sub_feedback_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<R1ControlNode>());
  rclcpp::shutdown();
  return 0;
}