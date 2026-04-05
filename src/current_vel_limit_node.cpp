#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <robomas_interfaces/msg/robomas_frame.hpp>
#include <robomas_interfaces/msg/robomas_packet.hpp>
#include <robomas_interfaces/msg/motor_command.hpp>
#include <algorithm>

using namespace std::chrono_literals;

class CurrentVelLimitNode : public rclcpp::Node {
public:
    CurrentVelLimitNode() : Node("current_vel_limit_node") {
        
        // パラメータの設定
        this->declare_parameter("motor_id", 1);
        this->declare_parameter("btn_a", 1); // Aボタン (正転)
        this->declare_parameter("btn_b", 0); // Bボタン (逆転)
        
        // 制御パラメータ
        this->declare_parameter("target_current", 2000.0f); // 流したい基本の電流 [mA]
        this->declare_parameter("max_velocity", 1500.0f);   // 制限したい最大速度 [rpm]
        this->declare_parameter("kp", 10.0f);               // 制限速度に近づいた時のブレーキゲイン

        motor_id_ = this->get_parameter("motor_id").as_int();
        btn_a_ = this->get_parameter("btn_a").as_int();
        btn_b_ = this->get_parameter("btn_b").as_int();
        target_current_ = this->get_parameter("target_current").as_double();
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        kp_ = this->get_parameter("kp").as_double();

        // Pub/Sub
        cmd_pub_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>("/robomas/cmd", 10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&CurrentVelLimitNode::joy_callback, this, std::placeholders::_1));
        fb_sub_ = this->create_subscription<robomas_interfaces::msg::RobomasFrame>(
            "/robomas/feedback", 10, std::bind(&CurrentVelLimitNode::feedback_callback, this, std::placeholders::_1));

        // 50Hzで制御
        timer_ = this->create_wall_timer(20ms, std::bind(&CurrentVelLimitNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Current Velocity Limit Node Started.");
        RCLCPP_INFO(this->get_logger(), "Motor: %d, Current: %.0fmA, Max Vel: %.0frpm", 
                    motor_id_, target_current_, max_velocity_);
    }

private:
    int motor_id_, btn_a_, btn_b_;
    float target_current_, max_velocity_, kp_;

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

        float current_vel = current_fb_.velocity[motor_id_ - 1]; // 現在の速度を取得
        float output_current = 0.0f; // マイコンに送る最終的な電流指令値

        if (latest_joy_.buttons.size() > (size_t)std::max(btn_a_, btn_b_)) {
            if (latest_joy_.buttons[btn_a_]) {
                // --- Aボタン：正転方向の制御 ---
                float vel_error = max_velocity_ - current_vel;
                
                if (vel_error > 0) {
                    output_current = target_current_; // 基本は設定した電流をフルで流す
                    
                    // 速度上限に近づいたら電流を絞る処理（ソフトウェアリミッター）
                    float limit_current = vel_error * kp_;
                    if (output_current > limit_current) {
                        output_current = limit_current;
                    }
                } else {
                    output_current = 0.0f; // 制限速度を超えていたら電流を流さない
                }

            } else if (latest_joy_.buttons[btn_b_]) {
                // --- Bボタン：逆転方向の制御 ---
                float vel_error = -max_velocity_ - current_vel;
                
                if (vel_error < 0) {
                    output_current = -target_current_;
                    
                    float limit_current = vel_error * kp_;
                    if (output_current < limit_current) {
                        output_current = limit_current;
                    }
                } else {
                    output_current = 0.0f;
                }
            }
        }

        // コマンドの送信
        robomas_interfaces::msg::RobomasPacket cmd_msg;
        robomas_interfaces::msg::MotorCommand cmd;
        cmd.motor_id = motor_id_;
        cmd.mode = 0; // 電流制御モード
        cmd.target = output_current; // 安全に制限された電流値
        cmd_msg.motors.push_back(cmd);
        
        cmd_pub_->publish(cmd_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CurrentVelLimitNode>());
    rclcpp::shutdown();
    return 0;
}