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
                           offset_angle_(0.0f),
                           goal_pos_(0.0f),
                           current_traj_pos_(0.0f),
                           current_traj_vel_(0.0f),
                           prev_sys_state_(SystemState::EMERGENCY), 
                           homing_state_(HomingState::NOT_STARTED) {
        
        // --- パラメータ設定 ---
        this->declare_parameter("motor_id", 1);
        this->declare_parameter("btn_a", 1);
        this->declare_parameter("btn_b", 0);
        this->declare_parameter("pos_a", 0.0f);      // Aボタンの目標角度 [deg]
        this->declare_parameter("pos_b", 30000.0f);  // Bボタンの目標角度 [deg]

        // 軌道生成用のパラメータ
        this->declare_parameter("max_vel", 2000.0f); // 最高速度 [deg/s]
        this->declare_parameter("accel", 1000.0f);   // 加速度 [deg/s^2]
        this->declare_parameter("decel", 1000.0f);   // 減速度 [deg/s^2]

        motor_id_ = this->get_parameter("motor_id").as_int();
        btn_a_ = this->get_parameter("btn_a").as_int();
        btn_b_ = this->get_parameter("btn_b").as_int();
        pos_a_ = this->get_parameter("pos_a").as_double();
        pos_b_ = this->get_parameter("pos_b").as_double();
        max_vel_ = this->get_parameter("max_vel").as_double();
        accel_ = this->get_parameter("accel").as_double();
        decel_ = this->get_parameter("decel").as_double();

        cmd_pub_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>("/robomas/cmd", 10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&SingleMotorJoyNode::joy_callback, this, std::placeholders::_1));
        fb_sub_ = this->create_subscription<robomas_interfaces::msg::RobomasFrame>(
            "/robomas/feedback", 10, std::bind(&SingleMotorJoyNode::feedback_callback, this, std::placeholders::_1));

        // 制御周期: 20ms (50Hz)
        timer_ = this->create_wall_timer(20ms, std::bind(&SingleMotorJoyNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Smooth Trajectory Node Started. Motor ID: %d", motor_id_);
    }

private:
    int motor_id_, btn_a_, btn_b_;
    float pos_a_, pos_b_;
    float max_vel_, accel_, decel_;
    
    float offset_angle_;
    
    // 軌道生成用の状態変数
    float goal_pos_;          // 最終的な目標位置
    float current_traj_pos_;  // 現在PC側がロボマスに指示している「滑らかな」目標位置
    float current_traj_vel_;  // 現在PC側が想定している速度

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

        if (prev_sys_state_ != SystemState::DRIVE && current_state == SystemState::DRIVE) {
            if (homing_state_ != HomingState::HOMING) {
                homing_state_ = HomingState::HOMING;
                RCLCPP_INFO(this->get_logger(), "Starting Homing...");
            }
        }
        if (current_state == SystemState::EMERGENCY) homing_state_ = HomingState::NOT_STARTED;
        prev_sys_state_ = current_state;
    }

    // 軌道を1ステップ分（20ms）進める関数
    void update_trajectory() {
        float dt = 0.02f; // 20ms
        float error = goal_pos_ - current_traj_pos_;

        // ゴールに十分近く、速度もほぼ0なら完全に止める
        if (std::abs(error) < 0.1f && std::abs(current_traj_vel_) < (accel_ * dt)) {
            current_traj_pos_ = goal_pos_;
            current_traj_vel_ = 0.0f;
            return;
        }

        // ゴールでぴったり止まるための制限速度（V = sqrt(2 * a * d)）
        float safe_vel = std::sqrt(2.0f * decel_ * std::abs(error));
        if (safe_vel > max_vel_) safe_vel = max_vel_;
        
        // 方向を加味した目標速度
        float v_target = (error > 0) ? safe_vel : -safe_vel;

        // 加速度による速度の更新
        if (current_traj_vel_ < v_target) {
            current_traj_vel_ += accel_ * dt;
            if (current_traj_vel_ > v_target) current_traj_vel_ = v_target;
        } else if (current_traj_vel_ > v_target) {
            current_traj_vel_ -= accel_ * dt;
            if (current_traj_vel_ < v_target) current_traj_vel_ = v_target;
        }

        // 位置の更新（積分）
        current_traj_pos_ += current_traj_vel_ * dt;
    }

    void control_loop() {
        if (current_fb_.system_state != 2) return; 

        robomas_interfaces::msg::RobomasPacket cmd_msg;
        robomas_interfaces::msg::MotorCommand cmd;
        cmd.motor_id = motor_id_;

        if (homing_state_ == HomingState::HOMING) {
            if (std::abs(current_fb_.current[motor_id_-1]) > 3000) {
                offset_angle_ = current_fb_.angle[motor_id_-1];
                
                // ホーミング完了時、軌道生成の初期状態をセット
                goal_pos_ = pos_a_;
                current_traj_pos_ = pos_a_; 
                current_traj_vel_ = 0.0f;
                
                homing_state_ = HomingState::DONE;
                RCLCPP_INFO(this->get_logger(), "Homing Done. Offset: %.2f", offset_angle_);
            } else {
                cmd.mode = 1; cmd.target = -300.0f; 
                cmd_msg.motors.push_back(cmd);
            }
        } else if (homing_state_ == HomingState::DONE) {
            // ボタンが押されたら「最終的なゴール」を更新するだけ
            if (latest_joy_.buttons.size() > (size_t)std::max(btn_a_, btn_b_)) {
                if (latest_joy_.buttons[btn_a_]) {
                    goal_pos_ = pos_a_;
                } else if (latest_joy_.buttons[btn_b_]) {
                    goal_pos_ = pos_b_;
                }
            }

            // 軌道を1ステップ計算して滑らかな目標値を作成
            update_trajectory();

            cmd.mode = 2; // 位置制御
            cmd.target = current_traj_pos_ + offset_angle_;
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