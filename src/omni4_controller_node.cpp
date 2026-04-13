#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <robomas_interfaces/msg/robomas_frame.hpp>
#include <robomas_interfaces/msg/robomas_packet.hpp>
#include <robomas_interfaces/msg/motor_command.hpp>
#include <robomas_interfaces/msg/can_frame.hpp>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

enum class SystemMode {
    EMERGENCY,
    HOMING,
    HOMING_ASCEND,
    DRIVE
};

class Omni4ControllerNode : public rclcpp::Node {
public:
    Omni4ControllerNode() : Node("omni4_controller_node"),
                            sys_mode_(SystemMode::EMERGENCY),
                            target_lift_pos_fl_(0.0),
                            target_lift_pos_bl_(0.0),
                            target_lift_pos_br_(0.0),
                            target_lift_pos_fr_(0.0) {
        
        // --- モーターID（ご提示の設定：fl=1, bl=2, br=9, fr=10） ---
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
        
        // --- ジョイスティックの軸設定 ---
        this->declare_parameter("axis_vx", 0); // L_stick_y (1): 前後
        this->declare_parameter("axis_vy", 1); // L_stick_x (0): 左右
        this->declare_parameter("axis_lt", 2); // LT (大抵2): 旋回（左）
        this->declare_parameter("axis_rt", 5); // RT (5): 旋回（右）
        this->declare_parameter("axis_lift", 4); // R_stick_y (通常4): 昇降

        // --- ボタン設定 ---
        this->declare_parameter("btn_extend", 4); // LB (4) -> 伸ばす
        this->declare_parameter("btn_contract", 5); // RB (5) -> 縮む
        this->declare_parameter("btn_grip_open", 0); // A (0) -> 開く
        this->declare_parameter("btn_grip_close", 1); // B (1) -> 閉じる(掴む)
        this->declare_parameter("btn_can_x", 2); // X (2) -> CAN 送信 (0x301 ...)
        this->declare_parameter("btn_can_y", 3); // Y (3) -> CAN 送信 (0x301 ...)
        
        // 緊急停止とホーミング用ボタン
        this->declare_parameter("btn_start", 7); // START -> ホーミング開始
        this->declare_parameter("btn_back", 6);  // BACK -> EMERGENCY(電流ゼロ)

        // --- 速度などの設定値 ---
        this->declare_parameter("max_rpm", 3000.0f);
        this->declare_parameter("lift_max_rpm", 3000.0f);
        this->declare_parameter("extend_max_rpm", 2000.0f);
        this->declare_parameter("grip_max_rpm", 1500.0f);

        // ホーミング周りの設定
        this->declare_parameter("homing_rpm", 500.0f);       // ホーミング時の下降速度
        this->declare_parameter("homing_current", 1000.0f);  // ホーミング完了とする電流閾値 (mA) 1A=1000mA
        this->declare_parameter("homing_ascend_deg", 1000.0f); // ホーミング完了後に上昇する距離(度)

        // パラメータの取り込み
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
        btn_can_x_ = this->get_parameter("btn_can_x").as_int();
        btn_can_y_ = this->get_parameter("btn_can_y").as_int();
        btn_start_ = this->get_parameter("btn_start").as_int();
        btn_back_  = this->get_parameter("btn_back").as_int();

        max_rpm_ = this->get_parameter("max_rpm").as_double();
        lift_max_rpm_ = this->get_parameter("lift_max_rpm").as_double();
        extend_max_rpm_ = this->get_parameter("extend_max_rpm").as_double();
        grip_max_rpm_ = this->get_parameter("grip_max_rpm").as_double();
        
        homing_rpm_ = this->get_parameter("homing_rpm").as_double();
        homing_current_ = this->get_parameter("homing_current").as_double();
        homing_ascend_deg_ = this->get_parameter("homing_ascend_deg").as_double();

        cmd_pub_ = this->create_publisher<robomas_interfaces::msg::RobomasPacket>("/robomas/cmd", 10);
        can_pub_ = this->create_publisher<robomas_interfaces::msg::CanFrame>("/robomas/can_tx", 10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&Omni4ControllerNode::joy_callback, this, std::placeholders::_1));
        fb_sub_ = this->create_subscription<robomas_interfaces::msg::RobomasFrame>(
            "/robomas/feedback", 10, std::bind(&Omni4ControllerNode::feedback_callback, this, std::placeholders::_1));

        // タイマー 20ms = 50Hz
        timer_ = this->create_wall_timer(10ms, std::bind(&Omni4ControllerNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Omni 4-Wheel Controller Node Started.");
        RCLCPP_INFO(this->get_logger(), "Current Mode: EMERGENCY. Press START to Home.");
    }

private:
    int motor_id_fl_, motor_id_fr_, motor_id_bl_, motor_id_br_;
    int motor_id_lift_fl_, motor_id_lift_fr_, motor_id_lift_bl_, motor_id_lift_br_;
    int motor_id_extend_, motor_id_grip_;
    int axis_vx_, axis_vy_, axis_lt_, axis_rt_, axis_lift_;
    int btn_extend_, btn_contract_, btn_grip_open_, btn_grip_close_, btn_can_x_, btn_can_y_, btn_start_, btn_back_;
    double max_rpm_, lift_max_rpm_, extend_max_rpm_, grip_max_rpm_, homing_rpm_, homing_current_, homing_ascend_deg_;

    SystemMode sys_mode_;
    double target_lift_pos_fl_, target_lift_pos_bl_, target_lift_pos_br_, target_lift_pos_fr_;
    double origin_lift_pos_fl_;
    bool is_homed_fl_ = false;
    bool is_homed_bl_ = false;
    bool is_homed_br_ = false;
    bool is_homed_fr_ = false;

    sensor_msgs::msg::Joy latest_joy_;
    robomas_interfaces::msg::RobomasFrame current_fb_;

    rclcpp::Publisher<robomas_interfaces::msg::RobomasPacket>::SharedPtr cmd_pub_;
    rclcpp::Publisher<robomas_interfaces::msg::CanFrame>::SharedPtr can_pub_;
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
        if (latest_joy_.buttons.empty()) return;
        int max_btn = std::max({btn_start_, btn_back_, btn_can_x_, btn_can_y_, btn_extend_, btn_contract_, btn_grip_open_, btn_grip_close_});
        if (latest_joy_.buttons.size() <= (size_t)max_btn) return;
        if (latest_joy_.axes.empty()) return;

        robomas_interfaces::msg::RobomasPacket cmd_msg;
        auto add_motor_cmd = [&](int id, int mode, float target) {
            robomas_interfaces::msg::MotorCommand cmd;
            cmd.motor_id = id;
            cmd.mode = mode;
            cmd.target = target;
            cmd_msg.motors.push_back(cmd);
        };

        // --- EMERGENCY / START / BACK ボタン判定 ---
        static bool prev_start = false;
        bool current_start = latest_joy_.buttons[btn_start_];
        bool is_start_pressed = (current_start && !prev_start); // エッジ検出
        prev_start = current_start;

        bool current_back = latest_joy_.buttons[btn_back_];

        if (current_back) {
            if (sys_mode_ != SystemMode::EMERGENCY) {
                sys_mode_ = SystemMode::EMERGENCY;
                RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP (BACK Pressed). All motors currently disabled.");
            }
        } else if (is_start_pressed) {
            if (sys_mode_ == SystemMode::EMERGENCY || sys_mode_ == SystemMode::DRIVE) {
                sys_mode_ = SystemMode::HOMING;
                is_homed_fl_ = false;
                is_homed_bl_ = false;
                is_homed_br_ = false;
                is_homed_fr_ = false;
                RCLCPP_INFO(this->get_logger(), "HOMING STARTED. Waiting for all 4 motors to reach 1A.");
            }
        }

        // EMERGENCY状態なら0A司令を送る
        if (sys_mode_ == SystemMode::EMERGENCY) {
            // 全てのモーターの電流を0にする(mode=0, target=0)
            add_motor_cmd(motor_id_fl_, 0, 0.0f);
            add_motor_cmd(motor_id_bl_, 0, 0.0f);
            add_motor_cmd(motor_id_br_, 0, 0.0f);
            add_motor_cmd(motor_id_fr_, 0, 0.0f);
            add_motor_cmd(motor_id_lift_fl_, 0, 0.0f);
            add_motor_cmd(motor_id_lift_bl_, 0, 0.0f);
            add_motor_cmd(motor_id_lift_br_, 0, 0.0f);
            add_motor_cmd(motor_id_lift_fr_, 0, 0.0f);
            add_motor_cmd(motor_id_extend_, 0, 0.0f);
            add_motor_cmd(motor_id_grip_, 0, 0.0f);
            cmd_pub_->publish(cmd_msg);
            return; 
        }

        // --- CAN フレームの送信 (X / Y ボタン) ---
        static bool prev_x = false;
        static bool prev_y = false;
        bool current_x = latest_joy_.buttons[btn_can_x_];
        bool current_y = latest_joy_.buttons[btn_can_y_];

        if (current_x && !prev_x) {
            auto msg = robomas_interfaces::msg::CanFrame();
            msg.id = 0x301;
            msg.dlc = 8;
            msg.data = {0x03, 0xF0, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00};
            can_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Sent CAN Frame (X button)");
        }
        if (current_y && !prev_y) {
            auto msg = robomas_interfaces::msg::CanFrame();
            msg.id = 0x301;
            msg.dlc = 8;
            msg.data = {0x0A, 0xBC, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00};
            can_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Sent CAN Frame (Y button)");
        }
        prev_x = current_x;
        prev_y = current_y;

        // システムステータスがDRIVEモード(物理ボタン)でないなら足回り等は動かさない
        if (current_fb_.system_state != 2) return;

        // --- 足回りの計算 ---
        int max_axis = std::max({axis_vx_, axis_vy_, axis_lt_, axis_rt_, axis_lift_});
        if (latest_joy_.axes.size() <= (size_t)max_axis) return;
        
        double raw_vx = latest_joy_.axes[axis_vx_];
        double raw_vy = latest_joy_.axes[axis_vy_];
        double vx = std::abs(raw_vx) < 0.05 ? 0.0 : raw_vx;
        double vy = std::abs(raw_vy) < 0.05 ? 0.0 : raw_vy;

        static bool lt_initialized = false;
        static bool rt_initialized = false;
        double lt_axis = latest_joy_.axes[axis_lt_];
        double rt_axis = latest_joy_.axes[axis_rt_];
        if (lt_axis != 0.0) lt_initialized = true;
        if (rt_axis != 0.0) rt_initialized = true;
        double lt_val = lt_initialized ? (1.0 - lt_axis) / 2.0 : 0.0;
        double rt_val = rt_initialized ? (1.0 - rt_axis) / 2.0 : 0.0;
        double raw_omega = rt_val - lt_val;
        double omega = std::abs(raw_omega) < 0.05 ? 0.0 : raw_omega;

        double v_fl = -vx + vy + omega;
        double v_fr = -vx - vy + omega;
        double v_bl = vx + vy + omega;
        double v_br = vx - vy + omega;

        double max_val = std::max({1.0, std::abs(v_fl), std::abs(v_fr), std::abs(v_bl), std::abs(v_br)});
        v_fl = (v_fl / max_val) * max_rpm_;
        v_fr = (v_fr / max_val) * max_rpm_;
        v_bl = (v_bl / max_val) * max_rpm_;
        v_br = (v_br / max_val) * max_rpm_;

        add_motor_cmd(motor_id_fl_, 1, v_fl);
        add_motor_cmd(motor_id_fr_, 1, v_fr);
        add_motor_cmd(motor_id_bl_, 1, v_bl);
        add_motor_cmd(motor_id_br_, 1, v_br);

        // --- 押し出しとグリップ ---
        double target_extend = 0.0;
        if (latest_joy_.buttons[btn_extend_]) {
            target_extend = -extend_max_rpm_; 
        } else if (latest_joy_.buttons[btn_contract_]) {
            target_extend = extend_max_rpm_;  
        }
        add_motor_cmd(motor_id_extend_, 1, target_extend);

        double target_grip = 0.0;
        if (latest_joy_.buttons[btn_grip_open_]) {
            target_grip = grip_max_rpm_;  
        } else if (latest_joy_.buttons[btn_grip_close_]) {
            target_grip = -grip_max_rpm_; 
        }
        add_motor_cmd(motor_id_grip_, 1, target_grip);

        // --- 昇降の処理 ---
        if (sys_mode_ == SystemMode::HOMING) {
            // 電流値チェック
            double c_fl = std::abs(current_fb_.current[motor_id_lift_fl_ - 1]);
            double c_bl = std::abs(current_fb_.current[motor_id_lift_bl_ - 1]);
            double c_br = std::abs(current_fb_.current[motor_id_lift_br_ - 1]);
            double c_fr = std::abs(current_fb_.current[motor_id_lift_fr_ - 1]);

            // 各モーターが1A(1000mA)を超えたら個別に原点を記録して停止する
            if (!is_homed_fl_ && c_fl > homing_current_) {
                target_lift_pos_fl_ = current_fb_.angle[motor_id_lift_fl_ - 1];
                is_homed_fl_ = true;
                RCLCPP_INFO(this->get_logger(), "FL Homed.");
            }
            if (!is_homed_bl_ && c_bl > homing_current_) {
                target_lift_pos_bl_ = current_fb_.angle[motor_id_lift_bl_ - 1];
                is_homed_bl_ = true;
                RCLCPP_INFO(this->get_logger(), "BL Homed.");
            }
            if (!is_homed_br_ && c_br > homing_current_) {
                target_lift_pos_br_ = current_fb_.angle[motor_id_lift_br_ - 1];
                is_homed_br_ = true;
                RCLCPP_INFO(this->get_logger(), "BR Homed.");
            }
            if (!is_homed_fr_ && c_fr > homing_current_) {
                target_lift_pos_fr_ = current_fb_.angle[motor_id_lift_fr_ - 1];
                is_homed_fr_ = true;
                RCLCPP_INFO(this->get_logger(), "FR Homed.");
            }

            // ホーミングがまだ終わっていないモーターだけ速度指令を出す（終わったものは0を指示）
            // fl, blが逆転(-), br, frが正転(+)で下降する
            double cmd_fl = is_homed_fl_ ? 0.0 : -homing_rpm_;
            double cmd_bl = is_homed_bl_ ? 0.0 : -homing_rpm_;
            double cmd_br = is_homed_br_ ? 0.0 : homing_rpm_;
            double cmd_fr = is_homed_fr_ ? 0.0 : homing_rpm_;

            add_motor_cmd(motor_id_lift_fl_, 1, cmd_fl);
            add_motor_cmd(motor_id_lift_bl_, 1, cmd_bl);
            add_motor_cmd(motor_id_lift_br_, 1, cmd_br);
            add_motor_cmd(motor_id_lift_fr_, 1, cmd_fr);

            // 4つすべてのモーターがホーミング完了するまで待つ
            if (is_homed_fl_ && is_homed_bl_ && is_homed_br_ && is_homed_fr_) {
                RCLCPP_INFO(this->get_logger(), "HOMING COMPLETE! All 4 motors reached threshold.");
                
                // 急激に目標値を変えると過大電流が流れるため、ここからHOMING_ASCEND状態で連続的に目標値を送る
                origin_lift_pos_fl_ = target_lift_pos_fl_;
                sys_mode_ = SystemMode::HOMING_ASCEND;
            }
        } 
        else if (sys_mode_ == SystemMode::HOMING_ASCEND) {
            // ホーミング完了後、少しだけ上に滑らかに移動する（位置制御で連続的に上げる）
            // 速度は安全のため最大RPMの半分程度に設定
            double step = (lift_max_rpm_ * 0.1) * 6.0 * 0.02; // 1周期あたりの上昇角度
            
            // どれだけ上がったか（flの現在目標値 - flの原点）を計算
            // 上昇方向： fl, bl は(+), br, fr は(-) に目標値が進む
            double diff = (origin_lift_pos_fl_ + homing_ascend_deg_) - target_lift_pos_fl_;

            if (diff > 0.0) {
                double move_step = std::min(step, diff);
                target_lift_pos_fl_ += move_step;
                target_lift_pos_bl_ += move_step;
                target_lift_pos_br_ -= move_step;
                target_lift_pos_fr_ -= move_step;
            } else {
                RCLCPP_INFO(this->get_logger(), "ASCEND COMPLETE! Ready for DRIVE.");
                sys_mode_ = SystemMode::DRIVE;
            }

            add_motor_cmd(motor_id_lift_fl_, 2, target_lift_pos_fl_);
            add_motor_cmd(motor_id_lift_bl_, 2, target_lift_pos_bl_);
            add_motor_cmd(motor_id_lift_br_, 2, target_lift_pos_br_);
            add_motor_cmd(motor_id_lift_fr_, 2, target_lift_pos_fr_);
        }
        else if (sys_mode_ == SystemMode::DRIVE) {
            double raw_lift = latest_joy_.axes[axis_lift_];
            double lift_v = std::abs(raw_lift) < 0.05 ? 0.0 : raw_lift;

            // 位置制御 (mode = 2)
            // 目標角度の更新: dt = 0.02s
            // (RPM -> deg/s) = RPM * 360 / 60 = RPM * 6.0
            // degの変化量 = delta_rpm * 6.0 * 0.02 = delta_rpm * 0.12
            
            // 上昇させたい(lift_v > 0)の場合:
            // モーターの回転方向を反転（先ほどの逆）にします
            double delta_rpm_fl_bl = lift_v * lift_max_rpm_;
            double delta_rpm_br_fr = -lift_v * lift_max_rpm_;

            target_lift_pos_fl_ += delta_rpm_fl_bl * 6.0 * 0.02;
            target_lift_pos_bl_ += delta_rpm_fl_bl * 6.0 * 0.02;
            target_lift_pos_br_ += delta_rpm_br_fr * 6.0 * 0.02;
            target_lift_pos_fr_ += delta_rpm_br_fr * 6.0 * 0.02;

            // 原点(ホーミング時の角度)以下には下がれないようにする制限は設けていません。
            // ※必要に応じて、target_lift_pos_xx_ が原点を越えないように制約できます。

            add_motor_cmd(motor_id_lift_fl_, 2, target_lift_pos_fl_);
            add_motor_cmd(motor_id_lift_bl_, 2, target_lift_pos_bl_);
            add_motor_cmd(motor_id_lift_br_, 2, target_lift_pos_br_);
            add_motor_cmd(motor_id_lift_fr_, 2, target_lift_pos_fr_);
        }

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
