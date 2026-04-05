#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <robomas_interfaces/msg/robomas_frame.hpp>
#include <robomas_interfaces/msg/robomas_packet.hpp>
#include <robomas_interfaces/msg/motor_command.hpp>
#include <cmath>

using namespace std::chrono_literals;

enum class SystemState { EMERGENCY = 0, READY = 1, DRIVE = 2 };
enum class HomingState { NOT_STARTED, HOMING, DONE };

class SingleMotorJoyNode : public rclcpp::Node {
public:
    SingleMotorJoyNode() : Node("single_motor_joy_node"), 
                           prev_sys_state_(SystemState::EMERGENCY), 
                           homing_state_(HomingState::NOT_STARTED),
                           offset_angle_(0.0f),
                           target_pos_(0.0f) {
        
        // パラメータ設定
        this->declare_parameter("motor_id", 1);
        this->declare_parameter("btn_forward", 1); // デフォルト: Aボタン (Joy-Con/ProCon)
        this->declare_parameter("btn_backward", 0); // デフォルト: Bボタン
        this->declare_parameter("move_speed", 2.0f); // 1周期(20ms)あたりの移動量[deg]

        motor_id_ = this->get_parameter("motor_id").as_int();
        btn_forward_ = this->get_parameter("btn_forward").as_int();
        btn_backward_ = this->get_parameter("btn_backward").as_int();
        move_speed_ = this->get_parameter("move_speed").as_double();

        // Publisher & Subscriber
        cmd_pub_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>("/robomas/cmd", 10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&SingleMotorJoyNode::joy_callback, this, std::placeholders::_1));
        fb_sub_ = this->create_subscription<robomas_interfaces::msg::RobomasFrame>(
            "/robomas/feedback", 10, std::bind(&SingleMotorJoyNode::feedback_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(20ms, std::bind(&SingleMotorJoyNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Single Motor Joy Node Started. Motor ID: %d", motor_id_);
    }

private:
    int motor_id_, btn_forward_, btn_backward_;
    float move_speed_, offset_angle_, target_pos_;
    SystemState prev_sys_state_;
    HomingState homing_state_;
    sensor_msgs::msg::Joy latest_joy_;
    robomas_interfaces::msg::RobomasFrame current_fb_;

    rclcpp::Publisher<robomas_interfaces::msg::RobomasPacket>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<robomas_interfaces::msg::RobomasFrame>::SharedPtr fb_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        latest_joy_ = *msg;
    }

    void feedback_callback(const robomas_interfaces::msg::RobomasFrame::SharedPtr msg) {
        current_fb_ = *msg;
        SystemState current_state = static_cast<SystemState>(msg->system_state);

        // DRIVEモード移行時にホーミング開始
        if (prev_sys_state_ != SystemState::DRIVE && current_state == SystemState::DRIVE) {
            if (homing_state_ != HomingState::HOMING) {
                homing_state_ = HomingState::HOMING;
                RCLCPP_INFO(this->get_logger(), "Starting Homing...");
            }
        }
        if (current_state == SystemState::EMERGENCY) homing_state_ = HomingState::NOT_STARTED;
        prev_sys_state_ = current_state;
    }

    void control_loop() {
        if (current_fb_.system_state != 2) return; 

        robomas_interfaces::msg::RobomasPacket cmd_msg;
        robomas_interfaces::msg::MotorCommand cmd;
        cmd.motor_id = motor_id_;

        if (homing_state_ == HomingState::HOMING) {
            // ホーミング中（電流閾値3000mAで判定）
            if (std::abs(current_fb_.current[motor_id_-1]) > 3000) {
                offset_angle_ = current_fb_.angle[motor_id_-1];
                target_pos_ = 0.0f;
                homing_state_ = HomingState::DONE;
                RCLCPP_INFO(this->get_logger(), "Homing Done.");
            } else {
                cmd.mode = 1; cmd.target = -300.0f; // ゆっくり下降
                cmd_msg.motors.push_back(cmd);
            }
        } else if (homing_state_ == HomingState::DONE) {
            // ジョイコンボタン判定
            if (latest_joy_.buttons.size() > (size_t)std::max(btn_forward_, btn_backward_)) {
                if (latest_joy_.buttons[btn_forward_]) {
                    target_pos_ += move_speed_;
                } else if (latest_joy_.buttons[btn_backward_]) {
                    target_pos_ -= move_speed_;
                }
            }

            cmd.mode = 2; // 位置制御
            cmd.target = target_pos_ + offset_angle_;
            cmd_msg.motors.push_back(cmd);
        }

        if (!cmd_msg.motors.empty()) cmd_pub_->publish(cmd_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SingleMotorJoyNode>());
    rclcpp::shutdown();
    return 0;
}