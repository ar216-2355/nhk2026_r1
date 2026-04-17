#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <robomas_interfaces/msg/motor_command.hpp>
#include <robomas_interfaces/msg/robomas_packet.hpp>
#include "robomas_interfaces/msg/robomas_frame.hpp"

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
    static constexpr float LIFT_RETRACT_POS = -20000.0f;
    static constexpr float LIFT_EXTEND_POS = 20000.0f;
     static constexpr float BOOK_RETRACT_POS = 0.0f;
     static constexpr float BOOK_EXTEND_POS = -60000.0f;
     static constexpr float POLE_RETRACT_POS = 0.0f;
     static constexpr float POLE_EXTEND_POS = 14000.0f;

    MotorData current_motors_[16]; // 16台分のモーター状態を入れる棚
    uint8_t current_system_state_ = 0;      // 0:EMERGENCY, 1:READY, 2:DRIVE

    float start_LF_omuni = 0;
    float current_LF_omuni = 0;
    float start_belt = 0;
    float current_belt = 0;
    bool kakuno_ok = false;
    bool prev_a_button_ = false;
    bool prev_b_button_ = false;
    bool prev_x_button_ = false;
    bool lift_extended_ = false;
    bool book_extended_ = false;
    bool pole_extended_ = false;
    float target_lift_position_ = 0.0f;
    float target_book_stretch_position_ = 0.0f;
    float target_pole_stretch_position_ = 0.0f;

    float estimate_lift_motion(float relative_target) const {
        const float target_lf = std::clamp(lift_offset[0] + relative_target, minpos, maxpos);
        const float target_lb = std::clamp(lift_offset[1] + relative_target, minpos, maxpos);
        const float target_rb = std::clamp(lift_offset[2] - relative_target, rb_rf_minpos, rb_rf_maxpos);
        const float target_rf = std::clamp(lift_offset[3] - relative_target, rb_rf_minpos, rb_rf_maxpos);

        return std::fabs(target_lf - current_motors_[MotorId::LIFT_LF - 1].angle) +
               std::fabs(target_lb - current_motors_[MotorId::LIFT_LB - 1].angle) +
               std::fabs(target_rb - current_motors_[MotorId::LIFT_RB - 1].angle) +
               std::fabs(target_rf - current_motors_[MotorId::LIFT_RF - 1].angle);
    }

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
            lift_extended_ = false;
            book_extended_ = false;
            pole_extended_ = false;
            target_lift_position_ = LIFT_RETRACT_POS;
            target_book_stretch_position_ = BOOK_RETRACT_POS;
            target_pole_stretch_position_ = POLE_RETRACT_POS;
        }

        if (latest_joy_.buttons.size() > Joy::X) {
            const bool a_pressed = latest_joy_.buttons[Joy::A];
            const bool b_pressed = latest_joy_.buttons[Joy::B];
            const bool x_pressed = latest_joy_.buttons[Joy::X];

            if (a_pressed && !prev_a_button_ && current_system_state_ == 2) {
                lift_extended_ = !lift_extended_;
                float next_target = lift_extended_ ? LIFT_EXTEND_POS : LIFT_RETRACT_POS;

                // クランプ後に無移動になる側を選んだ場合、反対側へフォールバックする。
                if (estimate_lift_motion(next_target) < 20.0f) {
                    lift_extended_ = !lift_extended_;
                    next_target = lift_extended_ ? LIFT_EXTEND_POS : LIFT_RETRACT_POS;
                }

                target_lift_position_ = next_target;
            }

            if (b_pressed && !prev_b_button_ && current_system_state_ == 2) {
                book_extended_ = !book_extended_;
                target_book_stretch_position_ = book_extended_ ? BOOK_EXTEND_POS : BOOK_RETRACT_POS;
            }

            if (x_pressed && !prev_x_button_ && current_system_state_ == 2) {
                pole_extended_ = !pole_extended_;
                target_pole_stretch_position_ = pole_extended_ ? POLE_EXTEND_POS : POLE_RETRACT_POS;
            }

            prev_a_button_ = a_pressed;
            prev_b_button_ = b_pressed;
            prev_x_button_ = x_pressed;
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

        set_pole_stretch(
            current_system_state_,
            target_pole_stretch_position_,
            current_motors_[MotorId::POLE_STRETCH - 1].angle,
            current_motors_[MotorId::POLE_STRETCH - 1].torque,
            packet);

        set_omni_velocity(
            latest_joy_.axes[Joy::L_STICK_X] * -1500, 
            latest_joy_.axes[Joy::L_STICK_Y] * 1500, 
            (latest_joy_.axes[Joy::LT] - latest_joy_.axes[Joy::RT]) * 500.0f, 
            packet);
        
        cmd_pub_->publish(packet);
    }



  sensor_msgs::msg::Joy latest_joy_;
  rclcpp::Publisher<robomas_interfaces::msg::RobomasPacket>::SharedPtr cmd_pub_;
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