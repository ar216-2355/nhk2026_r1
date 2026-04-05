#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <robomas_interfaces/msg/robomas_frame.hpp>
#include <robomas_interfaces/msg/robomas_packet.hpp>
#include <robomas_interfaces/msg/motor_command.hpp>
#include <cmath>

using namespace std::chrono_literals;

enum class SystemState {
    EMERGENCY = 0,
    READY = 1,
    DRIVE = 2
};

enum class HomingState {
    NOT_STARTED,
    HOMING,
    DONE
};


class SingleHomingNode : public rclcpp::Node {
public:
    SingleHomingNode() : Node("single_homing_node"),
                           offset_angle_(0.0f),
                           target_pos_(0.0f),
                           prev_sys_state_(SystemState::EMERGENCY), 
                           homing_state_(HomingState::NOT_STARTED) {
        
        // --- パラメータの宣言と取得 ---
        this->declare_parameter("motor_id", 1); // デフォルトはモーター1
        this->declare_parameter("homing_speed", -300.0f); // ホーミング時の速度[rpm]
        this->declare_parameter("homing_current_threshold", 3000); // 端点判定の電流閾値[mA]

        motor_id_ = this->get_parameter("motor_id").as_int();
        homing_speed_ = this->get_parameter("homing_speed").as_double();
        homing_threshold_ = this->get_parameter("homing_current_threshold").as_int();

        RCLCPP_INFO(this->get_logger(), "Single Homing Node Started. Motor ID: %d", motor_id_);

        // --- Publisher & Subscriber ---
        cmd_pub_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>("/robomas/cmd", 10);
        
        // 目標位置を受け取るサブスクライバー（例: /target_position トピック）
        target_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/target_position", 10, std::bind(&SingleHomingNode::target_callback, this, std::placeholders::_1));
            
        fb_sub_ = this->create_subscription<robomas_interfaces::msg::RobomasFrame>(
            "/robomas/feedback", 10, std::bind(&SingleHomingNode::feedback_callback, this, std::placeholders::_1));

        // --- タイマー (50Hz) ---
        timer_ = this->create_wall_timer(20ms, std::bind(&SingleHomingNode::control_loop, this));
    }

private:
    int motor_id_;
    float homing_speed_;
    int homing_threshold_;

    float offset_angle_;
    float target_pos_;

    SystemState prev_sys_state_;
    HomingState homing_state_;
    robomas_interfaces::msg::RobomasFrame current_fb_;

    rclcpp::Publisher<robomas_interfaces::msg::RobomasPacket>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_sub_;
    rclcpp::Subscription<robomas_interfaces::msg::RobomasFrame>::SharedPtr fb_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 目標位置の更新コールバック
    void target_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        if (homing_state_ == HomingState::DONE) {
            target_pos_ = msg->data;
            RCLCPP_INFO(this->get_logger(), "New Target Position: %.1f", target_pos_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Cannot set target. Homing is not done yet.");
        }
    }

    // フィードバック受信と状態遷移の判定
    void feedback_callback(const robomas_interfaces::msg::RobomasFrame::SharedPtr msg) {
        current_fb_ = *msg;
        SystemState current_state = static_cast<SystemState>(msg->system_state);

        bool just_changed_to_drive = (prev_sys_state_ != SystemState::DRIVE && current_state == SystemState::DRIVE);
        
        // DRIVEモードに移行したらホーミング開始
        if (just_changed_to_drive) {
            if (homing_state_ == HomingState::NOT_STARTED || homing_state_ == HomingState::DONE) {
                RCLCPP_INFO(this->get_logger(), "Starting Homing for Motor %d...", motor_id_);
                homing_state_ = HomingState::HOMING;
            }
        }

        // EMERGENCYになったら状態をリセット
        if (current_state == SystemState::EMERGENCY) {
            if (homing_state_ != HomingState::NOT_STARTED) {
                RCLCPP_WARN(this->get_logger(), "EMERGENCY detected. Resetting homing state.");
            }
            homing_state_ = HomingState::NOT_STARTED;
            target_pos_ = 0.0f; // 目標値も安全のためリセット
        }

        prev_sys_state_ = current_state;
    }

    // 制御ループ (定期実行)
    void control_loop() {
        if (current_fb_.system_state != 2) return; // DRIVE以外は何もしない

        int idx = motor_id_ - 1; // 配列インデックスは0から
        if (idx < 0 || idx >= 16) return;

        robomas_interfaces::msg::RobomasPacket cmd_msg;
        robomas_interfaces::msg::MotorCommand cmd;
        cmd.motor_id = motor_id_;

        if (homing_state_ == HomingState::HOMING) {
            // 端点（壁）に当たったか電流値で判定
            if (std::abs(current_fb_.current[idx]) > homing_threshold_) {
                offset_angle_ = current_fb_.angle[idx];
                cmd.mode = 1; // 速度制御で止める
                cmd.target = 0.0f;
                homing_state_ = HomingState::DONE;
                RCLCPP_INFO(this->get_logger(), "Homing Completed! Offset: %.2f", offset_angle_);
            } else {
                // 当たるまでは速度制御で移動
                cmd.mode = 1; // 速度制御
                cmd.target = homing_speed_;
            }
            cmd_msg.motors.push_back(cmd);
            cmd_pub_->publish(cmd_msg);

        } else if (homing_state_ == HomingState::DONE) {
            // ホーミング完了後は位置制御
            cmd.mode = 2; // 位置制御
            // ユーザーが指定した目標位置 + ホーミングで取得した原点オフセット
            cmd.target = target_pos_ + offset_angle_; 
            cmd_msg.motors.push_back(cmd);
            cmd_pub_->publish(cmd_msg);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SingleHomingNode>());
    rclcpp::shutdown();
    return 0;
}