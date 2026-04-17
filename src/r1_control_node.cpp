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

    float start_LF_omuni = 0;
    float current_LF_omuni = 0;
    float start_belt = 0;
    float current_belt = 0;
    bool kakuno_ok = false;
    bool prev_a_button_ = false;
    bool prev_b_button_ = false;
    bool prev_x_button_ = false;
    bool prev_y_button_ = false;
    float target_lift_position_ = 0.0f;
    float target_book_stretch_position_ = 0.0f;
    float target_pole_stretch_position_ = 0.0f;
    uint16_t target_book_stretch_angle = 0;
    uint16_t target_pole_stretch_angle = 0;

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
        }

        if (latest_joy_.buttons.size() > Joy::X) {
            const bool a_pressed = latest_joy_.buttons[Joy::A];
            const bool b_pressed = latest_joy_.buttons[Joy::B];
            const bool x_pressed = latest_joy_.buttons[Joy::X];
            const bool y_pressed = latest_joy_.buttons[Joy::Y];
            if (a_pressed && !prev_a_button_ &&
                lift_state[0] == SystemMode::DRIVE &&
                lift_state[1] == SystemMode::DRIVE &&
                lift_state[2] == SystemMode::DRIVE &&
                lift_state[3] == SystemMode::DRIVE) {
                target_lift_position_ = (std::fabs(target_lift_position_ - 20000.0f) < 1.0f) ? 0.0f : 20000.0f;
            }

            if (b_pressed && !prev_b_button_ && current_system_state_ == 2) {
                target_book_stretch_position_ =
                    (std::fabs(target_book_stretch_position_ - (-60000.0f)) < 1.0f) ? 0.0f : -60000.0f;
            }

            if (x_pressed && !prev_x_button_ && current_system_state_ == 2) {
                target_book_stretch_angle = (target_book_stretch_angle == 180U) ? 90U : 180U;
                target_pole_stretch_angle = (target_pole_stretch_angle == 180U) ? 90U : 180U;
            }

            if (y_pressed && !prev_y_button_ && current_system_state_ == 2) {
                target_book_catch_current = (target_book_catch_current == 0.0f) ? 0.25f : 0.0f;
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
        servo_book_stretch(target_book_stretch_angle, can_pub_);
        start_can_send_book(can_pub_);
        dc_book_catch(target_book_catch_current, can_pub_);

        set_pole_stretch(
            current_system_state_,
            target_pole_stretch_position_,
            current_motors_[MotorId::POLE_STRETCH - 1].angle,
            current_motors_[MotorId::POLE_STRETCH - 1].torque,
            packet);
        servo_pole_stretch(target_pole_stretch_angle, can_pub_);

        set_omni_velocity(
            latest_joy_.axes[Joy::L_STICK_X] * -1500, 
            latest_joy_.axes[Joy::L_STICK_Y] * 1500, 
            (latest_joy_.axes[Joy::LT] - latest_joy_.axes[Joy::RT]) * 500.0f, 
            packet);
        
        cmd_pub_->publish(packet);
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