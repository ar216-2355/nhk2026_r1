#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <robomas_interfaces/msg/robomas_frame.hpp>
#include <robomas_interfaces/msg/robomas_packet.hpp>
#include <robomas_interfaces/msg/motor_command.hpp>
#include <algorithm>

using namespace std::chrono_literals;

class SimpleVelocityNode : public rclcpp::Node {
public:
    SimpleVelocityNode() : Node("simple_velocity_node") {
        
        // パラメータの設定
        this->declare_parameter("motor_id", 1);
        this->declare_parameter("btn_forward", 1); // Aボタン (正転)
        this->declare_parameter("btn_backward", 0); // Bボタン (逆転)
        this->declare_parameter("target_rpm", 300.0f); // 回転速度 [rpm]

        motor_id_ = this->get_parameter("motor_id").as_int();
        btn_forward_ = this->get_parameter("btn_forward").as_int();
        btn_backward_ = this->get_parameter("btn_backward").as_int();
        target_rpm_ = this->get_parameter("target_rpm").as_double();

        // Pub/Sub
        cmd_pub_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>("/robomas/cmd", 10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&SimpleVelocityNode::joy_callback, this, std::placeholders::_1));
        fb_sub_ = this->create_subscription<robomas_interfaces::msg::RobomasFrame>(
            "/robomas/feedback", 10, std::bind(&SimpleVelocityNode::feedback_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(20ms, std::bind(&SimpleVelocityNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Simple Velocity Node Started.");
        RCLCPP_INFO(this->get_logger(), "Motor: %d, Target: %.0f rpm", motor_id_, target_rpm_);
    }

private:
    int motor_id_, btn_forward_, btn_backward_;
    float target_rpm_;

    sensor_msgs::msg::Joy latest_joy_;
    robomas_interfaces::msg::RobomasFrame current_fb_;

    rclcpp::Publisher<robomas_interfaces::msg::RobomasPacket>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<robomas_interfaces::msg::RobomasFrame>::SharedPtr fb_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) { latest_joy_ = *msg; }
    void feedback_callback(const robomas_interfaces::msg::RobomasFrame::SharedPtr msg) { current_fb_ = *msg; }

    void control_loop() {
        if (current_fb_.system_state != 2) return; // DRIVEモードの時だけ動く

        float output_rpm = 0.0f; // 基本は0 (停止)

        // ジョイコンのボタン判定
        if (latest_joy_.buttons.size() > (size_t)std::max(btn_forward_, btn_backward_)) {
            if (latest_joy_.buttons[btn_forward_]) {
                output_rpm = target_rpm_;  // Aボタンで正転
            } else if (latest_joy_.buttons[btn_backward_]) {
                output_rpm = -target_rpm_; // Bボタンで逆転
            }
        }

        // コマンドの送信
        robomas_interfaces::msg::RobomasPacket cmd_msg;
        robomas_interfaces::msg::MotorCommand cmd;
        cmd.motor_id = motor_id_;
        cmd.mode = 1; // 速度制御モード
        cmd.target = output_rpm;
        cmd_msg.motors.push_back(cmd);
        
        cmd_pub_->publish(cmd_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleVelocityNode>());
    rclcpp::shutdown();
    return 0;
}