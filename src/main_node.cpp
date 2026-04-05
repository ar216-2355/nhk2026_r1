#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <robomas_interfaces/msg/robomas_frame.hpp>
#include <robomas_interfaces/msg/robomas_packet.hpp>
#include <robomas_interfaces/msg/motor_command.hpp>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

// 状態遷移の定義
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

class Nhk2026R1Node : public rclcpp::Node {
public:
    Nhk2026R1Node() : Node("nhk2026_r1_node"), 
                      prev_sys_state_(SystemState::EMERGENCY), 
                      homing_state_(HomingState::NOT_STARTED) {
        
        // Publisher & Subscriber
        cmd_pub_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>("/robomas/cmd", 10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&Nhk2026R1Node::joy_callback, this, std::placeholders::_1));
        fb_sub_ = this->create_subscription<robomas_interfaces::msg::RobomasFrame>(
            "/robomas/feedback", 10, std::bind(&Nhk2026R1Node::feedback_callback, this, std::placeholders::_1));

        // 制御ループ (50Hz = 20ms)
        timer_ = this->create_wall_timer(20ms, std::bind(&Nhk2026R1Node::control_loop, this));

        // オフセットの初期化
        for (int i = 0; i < 4; i++) {
            lift_offset_angle_[i] = 0.0f;
            lift_target_pos_[i] = 0.0f;
        }

        RCLCPP_INFO(this->get_logger(), "NHK2026 R1 Node Started.");
    }

private:
    rclcpp::Publisher<robomas_interfaces::msg::RobomasPacket>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<robomas_interfaces::msg::RobomasFrame>::SharedPtr fb_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::Joy current_joy_;
    robomas_interfaces::msg::RobomasFrame current_fb_;

    SystemState prev_sys_state_;
    HomingState homing_state_;

    // 昇降機構用の変数
    float lift_offset_angle_[4];
    float lift_target_pos_[4];
    const float HOMING_SPEED = -300.0f; // 下降する方向の速度[rpm]
    const int HOMING_CURRENT_THRESHOLD = 3000; // 壁に当たったと判定する電流閾値[mA]

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        current_joy_ = *msg;
    }

    void feedback_callback(const robomas_interfaces::msg::RobomasFrame::SharedPtr msg) {
        current_fb_ = *msg;
        SystemState current_state = static_cast<SystemState>(msg->system_state);

        // ホーミング開始条件の判定
        bool just_changed_to_drive = (prev_sys_state_ != SystemState::DRIVE && current_state == SystemState::DRIVE);
        
        // ノード起動時またはDRIVEに移行した直後にホーミング未完了なら開始
        if (just_changed_to_drive) {
            if (homing_state_ == HomingState::NOT_STARTED || homing_state_ == HomingState::DONE) {
                RCLCPP_INFO(this->get_logger(), "Transitioned to DRIVE. Starting Homing...");
                homing_state_ = HomingState::HOMING;
            }
        }

        // EMERGENCYになったらホーミング状態をリセット
        if (current_state == SystemState::EMERGENCY) {
            homing_state_ = HomingState::NOT_STARTED;
        }

        prev_sys_state_ = current_state;
    }

    void control_loop() {
        if (current_fb_.system_state != 2) {
            // DRIVEモード以外は何もしない
            return;
        }

        robomas_interfaces::msg::RobomasPacket cmd_msg;

        if (homing_state_ == HomingState::HOMING) {
            // --- ホーミング制御 ---
            bool all_homed = true;

            for (int i = 0; i < 4; i++) {
                int motor_idx = 4 + i; // モーター5〜8 (index 4〜7)
                robomas_interfaces::msg::MotorCommand lift_cmd;
                lift_cmd.motor_id = motor_idx + 1;

                // 電流値が閾値を超えたかチェック
                if (std::abs(current_fb_.current[motor_idx]) > HOMING_CURRENT_THRESHOLD) {
                    // 当たった場合はそこを原点（オフセット）として記録し、モーターを止める
                    lift_offset_angle_[i] = current_fb_.angle[motor_idx];
                    lift_cmd.mode = 1; // 速度制御
                    lift_cmd.target = 0.0f;
                } else {
                    // まだ当たっていない場合はゆっくり下降
                    lift_cmd.mode = 1; // 速度制御
                    lift_cmd.target = HOMING_SPEED;
                    all_homed = false;
                }
                cmd_msg.motors.push_back(lift_cmd);
            }

            if (all_homed) {
                RCLCPP_INFO(this->get_logger(), "Homing Completed!");
                homing_state_ = HomingState::DONE;
                // 目標位置を0にリセット
                for (int i = 0; i < 4; i++) lift_target_pos_[i] = 0.0f; 
            }

        } else if (homing_state_ == HomingState::DONE) {
            // --- 通常制御（ジョイコン操作） ---
            if (current_joy_.axes.size() < 4) return; // ジョイコン未接続対策

            // 1. オムニ4輪制御 (モーター1〜4)
            float vx = current_joy_.axes[1]; // 左スティック上下
            float vy = current_joy_.axes[0]; // 左スティック左右
            float omega = current_joy_.axes[3]; // 右スティック左右

            float v1 = vx - vy + omega;
            float v2 = vx + vy - omega;
            float v3 = -vx + vy + omega;
            float v4 = -vx - vy - omega;

            const float MAX_RPM = 5000.0f;
            float wheel_targets[4] = {v1 * MAX_RPM, v2 * MAX_RPM, v3 * MAX_RPM, v4 * MAX_RPM};

            for (int i = 0; i < 4; i++) {
                robomas_interfaces::msg::MotorCommand wheel_cmd;
                wheel_cmd.motor_id = i + 1;
                wheel_cmd.mode = 1; // 速度制御
                wheel_cmd.target = wheel_targets[i];
                cmd_msg.motors.push_back(wheel_cmd);
            }

            // 2. 昇降機構制御 (モーター5〜8)
            // 十字キーの上下などで連続的に目標位置を更新
            float lift_input = current_joy_.axes[7]; // 十字キー上下を想定
            const float LIFT_SPEED_DEG_PER_TICK = 5.0f; // 1ループ(20ms)あたりの変化量

            for (int i = 0; i < 4; i++) {
                lift_target_pos_[i] += lift_input * LIFT_SPEED_DEG_PER_TICK;

                // 上下限のリミットを設けるのが安全です
                if (lift_target_pos_[i] < 0.0f) lift_target_pos_[i] = 0.0f;
                // if (lift_target_pos_[i] > MAX_LIFT_DEG) lift_target_pos_[i] = MAX_LIFT_DEG;

                robomas_interfaces::msg::MotorCommand lift_cmd;
                lift_cmd.motor_id = i + 5; // モーター5〜8
                lift_cmd.mode = 2; // 位置制御
                // 実機への指令値 = ジョイコンからの目標位置 + ホーミング時に記録した原点
                lift_cmd.target = lift_target_pos_[i] + lift_offset_angle_[i];
                cmd_msg.motors.push_back(lift_cmd);
            }
        }

        // コマンドの送信
        if (!cmd_msg.motors.empty()) {
            cmd_pub_->publish(cmd_msg);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nhk2026R1Node>());
    rclcpp::shutdown();
    return 0;
}