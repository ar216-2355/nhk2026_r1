#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <robomas_interfaces/msg/robomas_frame.hpp>
#include <robomas_interfaces/msg/robomas_packet.hpp>
#include <robomas_interfaces/msg/motor_command.hpp>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

class Omni4ControllerNode : public rclcpp::Node {
public:
    Omni4ControllerNode() : Node("omni4_controller_node") {
        
        // パラメータ設定
        // モーターID（ご提示の設定：fl=1, bl=2, br=9, fr=10）
        this->declare_parameter("motor_id_fl", 1);
        this->declare_parameter("motor_id_bl", 2);
        this->declare_parameter("motor_id_br", 9);
        this->declare_parameter("motor_id_fr", 10);
        
        // ジョイスティックの軸設定
        this->declare_parameter("axis_vx", 1); // L_stick_y (1): 前後
        this->declare_parameter("axis_vy", 0); // L_stick_x (0): 左右
        this->declare_parameter("axis_lt", 4); // LT (4): 旋回（左）
        this->declare_parameter("axis_rt", 5); // RT (5): 旋回（右）

        // 最大速度（rpm）
        this->declare_parameter("max_rpm", 3000.0f);

        motor_id_fl_ = this->get_parameter("motor_id_fl").as_int();
        motor_id_fr_ = this->get_parameter("motor_id_fr").as_int();
        motor_id_bl_ = this->get_parameter("motor_id_bl").as_int();
        motor_id_br_ = this->get_parameter("motor_id_br").as_int();

        axis_vx_ = this->get_parameter("axis_vx").as_int();
        axis_vy_ = this->get_parameter("axis_vy").as_int();
        axis_lt_ = this->get_parameter("axis_lt").as_int();
        axis_rt_ = this->get_parameter("axis_rt").as_int();
        max_rpm_ = this->get_parameter("max_rpm").as_double();

        cmd_pub_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>("/robomas/cmd", 10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&Omni4ControllerNode::joy_callback, this, std::placeholders::_1));
        fb_sub_ = this->create_subscription<robomas_interfaces::msg::RobomasFrame>(
            "/robomas/feedback", 10, std::bind(&Omni4ControllerNode::feedback_callback, this, std::placeholders::_1));

        // タイマー 20ms = 50Hz
        timer_ = this->create_wall_timer(20ms, std::bind(&Omni4ControllerNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Omni 4-Wheel Controller Node Started.");
    }

private:
    int motor_id_fl_, motor_id_fr_, motor_id_bl_, motor_id_br_;
    int axis_vx_, axis_vy_, axis_lt_, axis_rt_;
    double max_rpm_;

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
    }

    void control_loop() {
        // DRIVEモード (system_state == 2) 時のみ実行する
        if (current_fb_.system_state != 2) return; 

        if (latest_joy_.axes.empty()) return;

        double vx = 0.0;
        double vy = 0.0;
        double omega = 0.0;

        int max_axis = std::max({axis_vx_, axis_vy_, axis_lt_, axis_rt_});
        if (latest_joy_.axes.size() > (size_t)max_axis) {
            // 前後左右の速度（左スティック）
            vx = latest_joy_.axes[axis_vx_] * -1;
            vy = latest_joy_.axes[axis_vy_];
            
            // 例: LT(左トリガー)で左旋回(正)、RT(右トリガー)で右旋回(負)とする
            omega = latest_joy_.axes[axis_lt_] - latest_joy_.axes[axis_rt_];
        }

        // 4輪オムニホイールの運動学 (X-drive想定)
        // ※ ロボットのホイール配置・回転方向に応じて符合は適宜調整してください
        double v_fl = vx - vy - omega;
        double v_fr = vx + vy + omega;
        double v_bl = vx + vy - omega;
        double v_br = vx - vy + omega;

        // コマンドが1.0を超える場合、比率を保ったまま正規化する
        double max_val = std::max({1.0, std::abs(v_fl), std::abs(v_fr), std::abs(v_bl), std::abs(v_br)});
        v_fl /= max_val;
        v_fr /= max_val;
        v_bl /= max_val;
        v_br /= max_val;

        // 目標RPMにスケーリング
        v_fl *= max_rpm_;
        v_fr *= max_rpm_;
        v_bl *= max_rpm_;
        v_br *= max_rpm_;

        robomas_interfaces::msg::RobomasPacket cmd_msg;
        
        auto add_motor_cmd = [&](int id, float target) {
            robomas_interfaces::msg::MotorCommand cmd;
            cmd.motor_id = id;
            cmd.mode = 1; // 速度制御モード
            cmd.target = target;
            cmd_msg.motors.push_back(cmd);
        };

        add_motor_cmd(motor_id_fl_, v_fl);
        add_motor_cmd(motor_id_fr_, v_fr);
        add_motor_cmd(motor_id_bl_, v_bl);
        add_motor_cmd(motor_id_br_, v_br);

        // 送信
        if(!cmd_msg.motors.empty()){
            cmd_pub_->publish(cmd_msg);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Omni4ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
