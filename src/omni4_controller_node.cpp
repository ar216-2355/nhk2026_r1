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
        
        // 昇降用モーターID（ご提示の設定：fl=3, bl=4, br=11, fr=12）
        this->declare_parameter("motor_id_lift_fl", 3);
        this->declare_parameter("motor_id_lift_bl", 4);
        this->declare_parameter("motor_id_lift_br", 11);
        this->declare_parameter("motor_id_lift_fr", 12);
        
        // 押し出し・把持用モーターID
        this->declare_parameter("motor_id_extend", 5);
        this->declare_parameter("motor_id_grip", 6);
        
        // ジョイスティックの軸設定
        this->declare_parameter("axis_vx", 0); // L_stick_y (1): 前後
        this->declare_parameter("axis_vy", 1); // L_stick_x (0): 左右
        this->declare_parameter("axis_lt", 2); // LT (大抵2): 旋回（左）
        this->declare_parameter("axis_rt", 5); // RT (5): 旋回（右）
        this->declare_parameter("axis_lift", 4); // R_stick_y (通常4): 昇降

        // ボタン設定
        this->declare_parameter("btn_extend", 4); // LB (4) -> 伸ばす
        this->declare_parameter("btn_contract", 5); // RB (5) -> 縮む
        this->declare_parameter("btn_grip_open", 0); // A (0) -> 開く
        this->declare_parameter("btn_grip_close", 1); // B (1) -> 閉じる(掴む)

        // 最大速度（rpm）
        this->declare_parameter("max_rpm", 3000.0f);
        this->declare_parameter("lift_max_rpm", 3000.0f);
        this->declare_parameter("extend_max_rpm", 2000.0f);
        this->declare_parameter("grip_max_rpm", 1500.0f);

        motor_id_fl_ = this->get_parameter("motor_id_fl").as_int();
        motor_id_fr_ = this->get_parameter("motor_id_fr").as_int();
        motor_id_bl_ = this->get_parameter("motor_id_bl").as_int();
        motor_id_br_ = this->get_parameter("motor_id_br").as_int();

        motor_id_lift_fl_ = this->get_parameter("motor_id_lift_fl").as_int();
        motor_id_lift_fr_ = this->get_parameter("motor_id_lift_fr").as_int();
        motor_id_lift_bl_ = this->get_parameter("motor_id_lift_bl").as_int();
        motor_id_lift_br_ = this->get_parameter("motor_id_lift_br").as_int();

        motor_id_extend_ = this->get_parameter("motor_id_extend").as_int();
        motor_id_grip_ = this->get_parameter("motor_id_grip").as_int();

        axis_vx_ = this->get_parameter("axis_vx").as_int();
        axis_vy_ = this->get_parameter("axis_vy").as_int();
        axis_lt_ = this->get_parameter("axis_lt").as_int();
        axis_rt_ = this->get_parameter("axis_rt").as_int();
        axis_lift_ = this->get_parameter("axis_lift").as_int();

        btn_extend_ = this->get_parameter("btn_extend").as_int();
        btn_contract_ = this->get_parameter("btn_contract").as_int();
        btn_grip_open_ = this->get_parameter("btn_grip_open").as_int();
        btn_grip_close_ = this->get_parameter("btn_grip_close").as_int();

        max_rpm_ = this->get_parameter("max_rpm").as_double();
        lift_max_rpm_ = this->get_parameter("lift_max_rpm").as_double();
        extend_max_rpm_ = this->get_parameter("extend_max_rpm").as_double();
        grip_max_rpm_ = this->get_parameter("grip_max_rpm").as_double();

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
    int motor_id_lift_fl_, motor_id_lift_fr_, motor_id_lift_bl_, motor_id_lift_br_;
    int motor_id_extend_, motor_id_grip_;
    int axis_vx_, axis_vy_, axis_lt_, axis_rt_, axis_lift_;
    int btn_extend_, btn_contract_, btn_grip_open_, btn_grip_close_;
    double max_rpm_, lift_max_rpm_, extend_max_rpm_, grip_max_rpm_;

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
        double lift_v = 0.0;

        int max_axis = std::max({axis_vx_, axis_vy_, axis_lt_, axis_rt_, axis_lift_});
        if (latest_joy_.axes.size() > (size_t)max_axis) {
            // 前後左右の速度（左スティック）デッドゾーン(遊び)を設定して不要な回転を防ぐ
            double raw_vx = latest_joy_.axes[axis_vx_];
            double raw_vy = latest_joy_.axes[axis_vy_];
            
            vx = std::abs(raw_vx) < 0.05 ? 0.0 : raw_vx;
            vy = std::abs(raw_vy) < 0.05 ? 0.0 : raw_vy;
            
            // ROS 2のJoyでは、トリガーは「一度も触れていないと0.0、一度でも触れると1.0〜-1.0」という厄介な仕様があります。
            // 片方のトリガーだけを引いた後の離した状態（1.0と0.0）で引き算するとオムニが勝手に回転してしまうため、未初期化対策を行います。
            static bool lt_initialized = false;
            static bool rt_initialized = false;
            double lt_axis = latest_joy_.axes[axis_lt_];
            double rt_axis = latest_joy_.axes[axis_rt_];
            
            if (lt_axis != 0.0) lt_initialized = true;
            if (rt_axis != 0.0) rt_initialized = true;
            
            // 初期化済みの場合は0.0〜1.0の押し込み量に変換。未初期化の場合は0.0とする。
            double lt_val = lt_initialized ? (1.0 - lt_axis) / 2.0 : 0.0;
            double rt_val = rt_initialized ? (1.0 - rt_axis) / 2.0 : 0.0;
            
            // 旋回方向の反転: RTで右旋回、LTで左旋回になるように入れ替え
            double raw_omega = rt_val - lt_val;
            
            // 旋回にも微小な入力に対するデッドゾーンを設ける
            omega = std::abs(raw_omega) < 0.05 ? 0.0 : raw_omega;

            // 昇降軸の取得とデッドゾーン
            double raw_lift = latest_joy_.axes[axis_lift_];
            lift_v = std::abs(raw_lift) < 0.05 ? 0.0 : raw_lift;
        }

        // 4輪オムニホイールの運動学 (X-drive想定)
        // ※ ロボットのホイール配置・回転方向に応じて符合は適宜調整してください
        double v_fl = -vx + vy + omega;
        double v_fr = -vx - vy + omega;
        double v_bl = vx + vy + omega;
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

        // --- 昇降用モーターの指令 ---
        // 同じ速度で動かないと壊れるため、正規化はせず絶対の指令値を入れる
        // 右スティック上が正（+）で動かす＝上昇、下が負（-）で下降
        // 要件: fl・blが正転で下方向（下降）、br・frが正転で上方向（上昇）
        // 上昇 (lift_v > 0) の場合 -> fl,bl は逆転(-)で上方向。br,fr は正転(+)で上方向。
        // 下降 (lift_v < 0) の場合 -> fl,bl は正転(+)で下方向。br,fr は逆転(-)で下方向。
        // つまり、flとblには ` -lift_v * lift_max_rpm_ `
        // brとfrには ` lift_v * lift_max_rpm_ ` を与えれば完璧に同期します。
        
        double target_lift_fl = lift_v * lift_max_rpm_;
        double target_lift_bl = lift_v * lift_max_rpm_;
        double target_lift_br = -lift_v * lift_max_rpm_;
        double target_lift_fr = -lift_v * lift_max_rpm_;

        add_motor_cmd(motor_id_lift_fl_, target_lift_fl);
        add_motor_cmd(motor_id_lift_bl_, target_lift_bl);
        add_motor_cmd(motor_id_lift_br_, target_lift_br);
        add_motor_cmd(motor_id_lift_fr_, target_lift_fr);

        // --- 押し出し用モーターの指令 ---
        // ロボマス5番: 正回転で縮む、逆回転で伸びる
        // LB (btn_extend_) で伸ばす -> 逆転(-)
        // RB (btn_contract_) で縮む -> 正転(+)
        double target_extend = 0.0;
        int max_btn = std::max({btn_extend_, btn_contract_, btn_grip_open_, btn_grip_close_});
        if (latest_joy_.buttons.size() > (size_t)max_btn) {
            if (latest_joy_.buttons[btn_extend_]) {
                target_extend = -extend_max_rpm_; // 伸ばす（逆転）
            } else if (latest_joy_.buttons[btn_contract_]) {
                target_extend = extend_max_rpm_;  // 縮む（正転）
            }
        }
        add_motor_cmd(motor_id_extend_, target_extend);

        // --- 把持(グリップ)用モーターの指令 ---
        // ロボマス6番: 正転で開く、逆転で掴む（閉じる）
        // 掴む際は速度制御で泥流（電流）制限されることで保持します
        // Aボタン等 (btn_grip_open_) で開く -> 正転(+)
        // Bボタン等 (btn_grip_close_) で掴む -> 逆転(-)
        double target_grip = 0.0;
        if (latest_joy_.buttons.size() > (size_t)max_btn) {
            if (latest_joy_.buttons[btn_grip_open_]) {
                target_grip = grip_max_rpm_;  // 開く（正転）
            } else if (latest_joy_.buttons[btn_grip_close_]) {
                target_grip = -grip_max_rpm_; // 掴む（逆転）
            }
        }
        add_motor_cmd(motor_id_grip_, target_grip);

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
