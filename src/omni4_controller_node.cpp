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

enum class ServoAngleState {
    DEG_45,
    DEG_135,
    DEG_225
};

class Omni4ControllerNode : public rclcpp::Node {
public:
    Omni4ControllerNode() : Node("omni4_controller_node"),
                            sys_mode_(SystemMode::EMERGENCY),
                            target_lift_pos_fl_(0.0),
                            target_lift_pos_bl_(0.0),
                            target_lift_pos_br_(0.0),
                            target_lift_pos_fr_(0.0),
                            target_motor5_home_pos_(0.0),
                            target_motor5_drive_pos_(0.0),
                            target_motor13_home_pos_(0.0),
                            target_motor13_drive_pos_(0.0),
                            servo_angle_state_(ServoAngleState::DEG_45) {
        
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

        // 13番モーター
        this->declare_parameter("motor_id_13", 13);
        
        // 押し出し・把持用モーターID
        this->declare_parameter("motor_id_extend", 5);
        this->declare_parameter("motor_id_grip", 6);
        
        // --- ジョイスティックの軸設定 ---
        this->declare_parameter("axis_vx", 3); // R_stick_y: 前後
        this->declare_parameter("axis_vy", 4); // R_stick_x: 左右
        this->declare_parameter("axis_lt", 2); // LT (大抵2): 旋回（左）
        this->declare_parameter("axis_rt", 5); // RT (5): 旋回（右）
        this->declare_parameter("axis_lift", 1); // L_stick_y: 昇降
        this->declare_parameter("axis_dpad_x", 6); // Dパッド横
        this->declare_parameter("axis_dpad_y", 7); // Dパッド縦

        // --- ボタン設定 ---
        this->declare_parameter("btn_extend", 4); // LB (4) -> 伸ばす
        this->declare_parameter("btn_contract", 5); // RB (5) -> 縮む
        this->declare_parameter("btn_grip_open", 0); // A (0) -> 開く
        this->declare_parameter("btn_grip_close", 1); // B (1) -> 閉じる(掴む)
        this->declare_parameter("btn_can_x", 2); // X (2) -> CAN 送信 (0x301 ...)
        this->declare_parameter("btn_can_y", 3); // Y (3) -> CAN 送信 (0x301 ...)
        
        // 緊急停止とホーミング、その他用ボタン
        this->declare_parameter("btn_start", 7); // START -> ホーミング開始
        this->declare_parameter("btn_back", 6);  // BACK -> EMERGENCY(電流ゼロ)
        this->declare_parameter("btn_home", 8);  // HOME -> CAN 送信 (0x111 ...)

        // --- 速度などの設定値 ---
        this->declare_parameter("max_rpm", 3000.0f);
        this->declare_parameter("lift_max_rpm", 3000.0f);
        this->declare_parameter("extend_max_rpm", 2000.0f);
        this->declare_parameter("grip_max_rpm", 1500.0f);
        this->declare_parameter("motor13_max_vel_deg_s", 50800.0f);   // 13番位置制御の最大速度[deg/s]
        this->declare_parameter("motor13_max_acc_deg_s2", 66000.0f);  // 13番位置制御の最大加速度[deg/s^2]

        // ホーミング周りの設定
        this->declare_parameter("homing_rpm", 500.0f);       // ホーミング時の下降速度
        this->declare_parameter("homing_current_lift_fl", 1000.0f);
        this->declare_parameter("homing_current_lift_bl", 1000.0f);
        this->declare_parameter("homing_current_lift_br", 1000.0f);
        this->declare_parameter("homing_current_lift_fr", 1000.0f);
        this->declare_parameter("homing_current_motor5", 2000.0f);   // 2A
        this->declare_parameter("homing_current_motor13", 2000.0f);  // 2A
        this->declare_parameter("homing_ascend_deg", 1000.0f); // ホーミング完了後に上昇する距離(度)
        this->declare_parameter("homing_timeout_sec", 8.0f); // ホーミング最大待機時間(秒)
        this->declare_parameter("drive_state_miss_limit", 20); // DRIVE不一致を許容する周期数(10ms周期)

        // パラメータの取り込み
        motor_id_fl_ = this->get_parameter("motor_id_fl").as_int();
        motor_id_fr_ = this->get_parameter("motor_id_fr").as_int();
        motor_id_bl_ = this->get_parameter("motor_id_bl").as_int();
        motor_id_br_ = this->get_parameter("motor_id_br").as_int();

        motor_id_lift_fl_ = this->get_parameter("motor_id_lift_fl").as_int();
        motor_id_lift_fr_ = this->get_parameter("motor_id_lift_fr").as_int();
        motor_id_lift_bl_ = this->get_parameter("motor_id_lift_bl").as_int();
        motor_id_lift_br_ = this->get_parameter("motor_id_lift_br").as_int();
        motor_id_13_ = this->get_parameter("motor_id_13").as_int();

        motor_id_extend_ = this->get_parameter("motor_id_extend").as_int();
        motor_id_grip_ = this->get_parameter("motor_id_grip").as_int();

        axis_vx_ = this->get_parameter("axis_vx").as_int();
        axis_vy_ = this->get_parameter("axis_vy").as_int();
        axis_lt_ = this->get_parameter("axis_lt").as_int();
        axis_rt_ = this->get_parameter("axis_rt").as_int();
        axis_lift_ = this->get_parameter("axis_lift").as_int();
        axis_dpad_x_ = this->get_parameter("axis_dpad_x").as_int();
        axis_dpad_y_ = this->get_parameter("axis_dpad_y").as_int();

        btn_extend_ = this->get_parameter("btn_extend").as_int();
        btn_contract_ = this->get_parameter("btn_contract").as_int();
        btn_grip_open_ = this->get_parameter("btn_grip_open").as_int();
        btn_grip_close_ = this->get_parameter("btn_grip_close").as_int();
        btn_can_x_ = this->get_parameter("btn_can_x").as_int();
        btn_can_y_ = this->get_parameter("btn_can_y").as_int();
        btn_start_ = this->get_parameter("btn_start").as_int();
        btn_back_  = this->get_parameter("btn_back").as_int();
        btn_home_  = this->get_parameter("btn_home").as_int();

        max_rpm_ = this->get_parameter("max_rpm").as_double();
        lift_max_rpm_ = this->get_parameter("lift_max_rpm").as_double();
        extend_max_rpm_ = this->get_parameter("extend_max_rpm").as_double();
        grip_max_rpm_ = this->get_parameter("grip_max_rpm").as_double();
        motor13_max_vel_deg_s_ = this->get_parameter("motor13_max_vel_deg_s").as_double();
        motor13_max_acc_deg_s2_ = this->get_parameter("motor13_max_acc_deg_s2").as_double();
        
        homing_rpm_ = this->get_parameter("homing_rpm").as_double();
        homing_current_lift_fl_ = this->get_parameter("homing_current_lift_fl").as_double();
        homing_current_lift_bl_ = this->get_parameter("homing_current_lift_bl").as_double();
        homing_current_lift_br_ = this->get_parameter("homing_current_lift_br").as_double();
        homing_current_lift_fr_ = this->get_parameter("homing_current_lift_fr").as_double();
        homing_current_motor5_ = this->get_parameter("homing_current_motor5").as_double();
        homing_current_motor13_ = this->get_parameter("homing_current_motor13").as_double();
        homing_ascend_deg_ = this->get_parameter("homing_ascend_deg").as_double();
        homing_timeout_sec_ = this->get_parameter("homing_timeout_sec").as_double();
        drive_state_miss_limit_ = this->get_parameter("drive_state_miss_limit").as_int();
        homing_start_time_ = this->now();

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
    int motor_id_13_;
    int motor_id_extend_, motor_id_grip_;
    int axis_vx_, axis_vy_, axis_lt_, axis_rt_, axis_lift_, axis_dpad_x_, axis_dpad_y_;
    int btn_extend_, btn_contract_, btn_grip_open_, btn_grip_close_, btn_can_x_, btn_can_y_, btn_start_, btn_back_, btn_home_;
    double max_rpm_, lift_max_rpm_, extend_max_rpm_, grip_max_rpm_, homing_rpm_, homing_ascend_deg_;
    double motor13_max_vel_deg_s_, motor13_max_acc_deg_s2_;
    double homing_current_lift_fl_, homing_current_lift_bl_, homing_current_lift_br_, homing_current_lift_fr_;
    double homing_current_motor5_, homing_current_motor13_;
    double homing_timeout_sec_;
    int drive_state_miss_limit_;
    int drive_state_miss_count_ = 0;

    SystemMode sys_mode_;
    double target_lift_pos_fl_, target_lift_pos_bl_, target_lift_pos_br_, target_lift_pos_fr_;
    double target_motor5_home_pos_;
    double target_motor5_drive_pos_;
    double target_motor13_home_pos_;
    double target_motor13_drive_pos_;
    double origin_lift_pos_fl_;
    double origin_lift_pos_bl_;
    double origin_lift_pos_br_;
    double origin_lift_pos_fr_;
    double origin_motor5_pos_;
    double origin_motor13_pos_;
    rclcpp::Time homing_start_time_;
    ServoAngleState servo_angle_state_;
    bool is_homed_fl_ = false;
    bool is_homed_bl_ = false;
    bool is_homed_br_ = false;
    bool is_homed_fr_ = false;
    bool is_homed_motor5_ = false;
    bool is_homed_motor13_ = false;
    bool motor5_drive_high_ = false;
    bool motor13_drive_high_ = false;
    double motor13_current_vel_deg_s_ = 0.0;

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
        int max_btn = std::max({btn_start_, btn_back_, btn_home_, btn_can_x_, btn_can_y_, btn_extend_, btn_contract_, btn_grip_open_, btn_grip_close_});
        if (latest_joy_.buttons.size() <= (size_t)max_btn) return;
        int max_axis = std::max({axis_vx_, axis_vy_, axis_lt_, axis_rt_, axis_lift_, axis_dpad_x_, axis_dpad_y_});
        if (latest_joy_.axes.size() <= (size_t)max_axis) return;

        robomas_interfaces::msg::RobomasPacket cmd_msg;
        auto add_motor_cmd = [&](int id, int mode, float target) {
            robomas_interfaces::msg::MotorCommand cmd;
            cmd.motor_id = id;
            cmd.mode = mode;
            cmd.target = target;
            cmd_msg.motors.push_back(cmd);
        };

        // DRIVE状態の不一致は瞬断を許容する（READY<->DRIVE遷移の揺れ対策）
        if (sys_mode_ == SystemMode::DRIVE) {
            if (current_fb_.system_state == 2) {
                drive_state_miss_count_ = 0;
            } else {
                drive_state_miss_count_++;
                if (drive_state_miss_count_ >= drive_state_miss_limit_) {
                    sys_mode_ = SystemMode::EMERGENCY;
                    drive_state_miss_count_ = 0;
                    RCLCPP_WARN(this->get_logger(),
                        "Hardware not in DRIVE for %d cycles. Forcing internal state to EMERGENCY.",
                        drive_state_miss_limit_);
                }
            }
        } else {
            drive_state_miss_count_ = 0;
        }

        // --- EMERGENCY / START / BACK ボタン判定 ---
        static bool prev_start = false;
        bool current_start = latest_joy_.buttons[btn_start_];
        bool is_start_pressed = (current_start && !prev_start); // エッジ検出
        prev_start = current_start;

        static bool prev_back = false;
        bool current_back = latest_joy_.buttons[btn_back_];
        bool is_back_pressed = (current_back && !prev_back);   // エッジ検出
        prev_back = current_back;

        if (is_start_pressed) {
            if (current_fb_.system_state != 2) {
                RCLCPP_WARN(this->get_logger(), "START pressed while system_state=%d (not DRIVE). Sending CAN init and starting HOMING anyway.", current_fb_.system_state);
            }
            // STARTを押した時にCANフレームも複数送る
            auto msg1 = robomas_interfaces::msg::CanFrame();
            msg1.id = 0x301; msg1.dlc = 8; msg1.data = {0x05, 0x53, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00};
            can_pub_->publish(msg1);

            auto msg2 = robomas_interfaces::msg::CanFrame();
            msg2.id = 0x302; msg2.dlc = 8; msg2.data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            can_pub_->publish(msg2);

            auto msg3 = robomas_interfaces::msg::CanFrame();
            msg3.id = 0x160; msg3.dlc = 1; msg3.data = {0x04};
            can_pub_->publish(msg3);

            if (sys_mode_ == SystemMode::EMERGENCY || sys_mode_ == SystemMode::DRIVE) {
                sys_mode_ = SystemMode::HOMING;
                homing_start_time_ = this->now();
                is_homed_fl_ = false;
                is_homed_bl_ = false;
                is_homed_br_ = false;
                is_homed_fr_ = false;
                is_homed_motor5_ = false;
                is_homed_motor13_ = false;
                motor5_drive_high_ = false;
                motor13_drive_high_ = false;
                target_motor5_drive_pos_ = 0.0;
                target_motor13_drive_pos_ = 0.0;
                motor13_current_vel_deg_s_ = 0.0;
                RCLCPP_INFO(this->get_logger(), "HOMING STARTED. Waiting for 4 lift motors, motor 5, and motor 13 to reach their thresholds.");
            }
        } else if (is_back_pressed) {
            if (sys_mode_ != SystemMode::EMERGENCY) {
                sys_mode_ = SystemMode::EMERGENCY;
                RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP (BACK Pressed). All motors currently disabled.");
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
            add_motor_cmd(motor_id_13_, 0, 0.0f);
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

        // --- DパッドとHOMEキーにおける拡張CAN送信 ---
        static bool prev_d_up = false;
        static bool prev_d_down = false;
        static bool prev_d_left = false;
        static bool prev_d_right = false;
        static bool prev_home = false;

        bool current_d_up = latest_joy_.axes[axis_dpad_y_] > 0.5;
        bool current_d_down = latest_joy_.axes[axis_dpad_y_] < -0.5;
        bool current_d_left = latest_joy_.axes[axis_dpad_x_] > 0.5;
        bool current_d_right = latest_joy_.axes[axis_dpad_x_] < -0.5;
        bool current_home = latest_joy_.buttons[btn_home_];

        auto send_servo_angle = [&](ServoAngleState state) {
            auto msg = robomas_interfaces::msg::CanFrame();
            msg.id = 0x301;
            msg.dlc = 8;
            switch (state) {
                case ServoAngleState::DEG_45:
                    msg.data = {0x03, 0x94, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00};
                    break;
                case ServoAngleState::DEG_135:
                    msg.data = {0x05, 0x5e, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00};
                    break;
                case ServoAngleState::DEG_225:
                    msg.data = {0x07, 0x28, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00};
                    break;
            }
            can_pub_->publish(msg);
        };

        if (current_d_up && !prev_d_up) {
            if (servo_angle_state_ == ServoAngleState::DEG_225) {
                servo_angle_state_ = ServoAngleState::DEG_135;
                send_servo_angle(ServoAngleState::DEG_135);
            } else if (servo_angle_state_ == ServoAngleState::DEG_135) {
                servo_angle_state_ = ServoAngleState::DEG_45;
                send_servo_angle(ServoAngleState::DEG_45);
            }
        }
        if (current_d_down && !prev_d_down) {
            if (servo_angle_state_ == ServoAngleState::DEG_45) {
                servo_angle_state_ = ServoAngleState::DEG_135;
                send_servo_angle(ServoAngleState::DEG_135);
            } else if (servo_angle_state_ == ServoAngleState::DEG_135) {
                servo_angle_state_ = ServoAngleState::DEG_225;
                send_servo_angle(ServoAngleState::DEG_225);
            }
        }
        if (current_d_right && !prev_d_right) {
            auto msg = robomas_interfaces::msg::CanFrame();
            msg.id = 0x161; msg.dlc = 4; msg.data = {0x3e, 0x99, 0x99, 0x99};
            can_pub_->publish(msg);
        }
        if (current_d_left && !prev_d_left) {
            auto msg = robomas_interfaces::msg::CanFrame();
            msg.id = 0x161; msg.dlc = 4; msg.data = {0xbe, 0x80, 0x00, 0x00};
            can_pub_->publish(msg);
        }
        if (current_home && !prev_home) {
            auto msg = robomas_interfaces::msg::CanFrame();
            msg.id = 0x161; msg.dlc = 4; msg.data = {0x00, 0x00, 0x00, 0x00};
            can_pub_->publish(msg);
        }

        prev_d_up = current_d_up;
        prev_d_down = current_d_down;
        prev_d_left = current_d_left;
        prev_d_right = current_d_right;
        prev_home = current_home;

        // システムステータスがDRIVE(2)でない時は足回り・機構のみ停止し、
        // 昇降のHOMING/HOMING_ASCEND処理は継続する
        bool hw_drive_enabled = (current_fb_.system_state == 2);

        // --- 足回りの計算 ---
        if (hw_drive_enabled) {
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
            if (latest_joy_.buttons[btn_grip_close_]) {
                target_grip = -grip_max_rpm_;
            }
            add_motor_cmd(motor_id_grip_, 1, target_grip);
        } else {
            add_motor_cmd(motor_id_fl_, 1, 0.0f);
            add_motor_cmd(motor_id_fr_, 1, 0.0f);
            add_motor_cmd(motor_id_bl_, 1, 0.0f);
            add_motor_cmd(motor_id_br_, 1, 0.0f);
            add_motor_cmd(motor_id_extend_, 1, 0.0f);
            add_motor_cmd(motor_id_grip_, 1, 0.0f);
        }

        // --- 昇降の処理 ---
        if (sys_mode_ == SystemMode::HOMING) {
            // 電流値チェック
            double c_fl = std::abs(current_fb_.current[motor_id_lift_fl_ - 1]);
            double c_bl = std::abs(current_fb_.current[motor_id_lift_bl_ - 1]);
            double c_br = std::abs(current_fb_.current[motor_id_lift_br_ - 1]);
            double c_fr = std::abs(current_fb_.current[motor_id_lift_fr_ - 1]);
            double c_m5 = std::abs(current_fb_.current[motor_id_extend_ - 1]);
            double c_m13 = std::abs(current_fb_.current[motor_id_13_ - 1]);

            double homing_elapsed = (this->now() - homing_start_time_).seconds();

            // 各モーターが1A(1000mA)を超えたら個別に原点を記録して停止する
            if (!is_homed_fl_ && c_fl > homing_current_lift_fl_) {
                target_lift_pos_fl_ = current_fb_.angle[motor_id_lift_fl_ - 1];
                is_homed_fl_ = true;
                RCLCPP_INFO(this->get_logger(), "FL Homed.");
            }
            if (!is_homed_bl_ && c_bl > homing_current_lift_bl_) {
                target_lift_pos_bl_ = current_fb_.angle[motor_id_lift_bl_ - 1];
                is_homed_bl_ = true;
                RCLCPP_INFO(this->get_logger(), "BL Homed.");
            }
            if (!is_homed_br_ && c_br > homing_current_lift_br_) {
                target_lift_pos_br_ = current_fb_.angle[motor_id_lift_br_ - 1];
                is_homed_br_ = true;
                RCLCPP_INFO(this->get_logger(), "BR Homed.");
            }
            if (!is_homed_fr_ && c_fr > homing_current_lift_fr_) {
                target_lift_pos_fr_ = current_fb_.angle[motor_id_lift_fr_ - 1];
                is_homed_fr_ = true;
                RCLCPP_INFO(this->get_logger(), "FR Homed.");
            }
            if (!is_homed_motor5_ && c_m5 > homing_current_motor5_) {
                target_motor5_home_pos_ = current_fb_.angle[motor_id_extend_ - 1];
                is_homed_motor5_ = true;
                RCLCPP_INFO(this->get_logger(), "M5 Homed.");
            }
            if (!is_homed_motor13_ && c_m13 > homing_current_motor13_) {
                target_motor13_home_pos_ = current_fb_.angle[motor_id_13_ - 1];
                is_homed_motor13_ = true;
                RCLCPP_INFO(this->get_logger(), "M13 Homed.");
            }

            if (homing_elapsed > homing_timeout_sec_) {
                if (!is_homed_fl_) {
                    target_lift_pos_fl_ = current_fb_.angle[motor_id_lift_fl_ - 1];
                    is_homed_fl_ = true;
                }
                if (!is_homed_bl_) {
                    target_lift_pos_bl_ = current_fb_.angle[motor_id_lift_bl_ - 1];
                    is_homed_bl_ = true;
                }
                if (!is_homed_br_) {
                    target_lift_pos_br_ = current_fb_.angle[motor_id_lift_br_ - 1];
                    is_homed_br_ = true;
                }
                if (!is_homed_fr_) {
                    target_lift_pos_fr_ = current_fb_.angle[motor_id_lift_fr_ - 1];
                    is_homed_fr_ = true;
                }
                if (!is_homed_motor5_) {
                    target_motor5_home_pos_ = current_fb_.angle[motor_id_extend_ - 1];
                    is_homed_motor5_ = true;
                }
                if (!is_homed_motor13_) {
                    target_motor13_home_pos_ = current_fb_.angle[motor_id_13_ - 1];
                    is_homed_motor13_ = true;
                }
                RCLCPP_WARN(this->get_logger(), "HOMING timeout reached (%.2fs). Forcing transition to ASCEND.", homing_elapsed);
            }

            // ホーミングがまだ終わっていないモーターだけ速度指令を出す（終わったものは0を指示）
            // fl, blが逆転(-), br, frが正転(+)で下降する
            double cmd_fl = is_homed_fl_ ? 0.0 : -homing_rpm_;
            double cmd_bl = is_homed_bl_ ? 0.0 : -homing_rpm_;
            double cmd_br = is_homed_br_ ? 0.0 : homing_rpm_;
            double cmd_fr = is_homed_fr_ ? 0.0 : homing_rpm_;
            double cmd_m5 = is_homed_motor5_ ? 0.0 : homing_rpm_;
            double cmd_m13 = is_homed_motor13_ ? 0.0 : homing_rpm_;

            add_motor_cmd(motor_id_lift_fl_, 1, cmd_fl);
            add_motor_cmd(motor_id_lift_bl_, 1, cmd_bl);
            add_motor_cmd(motor_id_lift_br_, 1, cmd_br);
            add_motor_cmd(motor_id_lift_fr_, 1, cmd_fr);
            add_motor_cmd(motor_id_extend_, 1, cmd_m5);
            add_motor_cmd(motor_id_13_, 1, cmd_m13);

            // 4つすべてのモーターがホーミング完了するまで待つ
            if (is_homed_fl_ && is_homed_bl_ && is_homed_br_ && is_homed_fr_ && is_homed_motor5_ && is_homed_motor13_) {
                RCLCPP_INFO(this->get_logger(), "HOMING COMPLETE! All lift motors, motor 5, and motor 13 reached threshold.");
                
                // 急激に目標値を変えると過大電流が流れるため、ここからHOMING_ASCEND状態で連続的に目標値を送る
                origin_lift_pos_fl_ = target_lift_pos_fl_;
                origin_lift_pos_bl_ = target_lift_pos_bl_;
                origin_lift_pos_br_ = target_lift_pos_br_;
                origin_lift_pos_fr_ = target_lift_pos_fr_;
                origin_motor5_pos_ = target_motor5_home_pos_;
                origin_motor13_pos_ = target_motor13_home_pos_;
                sys_mode_ = SystemMode::HOMING_ASCEND;
            }
        } 
        else if (sys_mode_ == SystemMode::HOMING_ASCEND) {
            // ホーミング完了後、少しだけ上に滑らかに移動する（位置制御で連続的に上げる）
            // 速度は安全のため最大RPMの半分程度に設定
            double step = (lift_max_rpm_ * 0.03) * 6.0 * 0.02; // 1周期あたりの上昇角度
            
            // 上昇方向： fl, bl は(+), br, fr は(-) に目標値が進む
            // 4輪の残量の最小値を使って同じステップで進める
            double rem_fl = (origin_lift_pos_fl_ + homing_ascend_deg_) - target_lift_pos_fl_;
            double rem_bl = (origin_lift_pos_bl_ + homing_ascend_deg_) - target_lift_pos_bl_;
            double rem_br = target_lift_pos_br_ - (origin_lift_pos_br_ - homing_ascend_deg_);
            double rem_fr = target_lift_pos_fr_ - (origin_lift_pos_fr_ - homing_ascend_deg_);
            double rem_m5 = target_motor5_home_pos_ - (origin_motor5_pos_ - homing_ascend_deg_);
            double rem_m13 = target_motor13_home_pos_ - (origin_motor13_pos_ - homing_ascend_deg_);
            double diff = std::min({rem_fl, rem_bl, rem_br, rem_fr, rem_m5, rem_m13});

            if (diff > 0.0) {
                double move_step = std::min(step, diff);
                target_lift_pos_fl_ += move_step;
                target_lift_pos_bl_ += move_step;
                target_lift_pos_br_ -= move_step;
                target_lift_pos_fr_ -= move_step;
                target_motor5_home_pos_ -= move_step;
                target_motor13_home_pos_ -= move_step;
            } else {
                RCLCPP_INFO(this->get_logger(), "ASCEND COMPLETE! Ready for DRIVE.");
                motor5_drive_high_ = false;
                target_motor5_drive_pos_ = target_motor5_home_pos_;
                motor13_drive_high_ = false;
                target_motor13_drive_pos_ = target_motor13_home_pos_;
                motor13_current_vel_deg_s_ = 0.0;
                drive_state_miss_count_ = 0;
                sys_mode_ = SystemMode::DRIVE;
            }

            add_motor_cmd(motor_id_lift_fl_, 2, target_lift_pos_fl_);
            add_motor_cmd(motor_id_lift_bl_, 2, target_lift_pos_bl_);
            add_motor_cmd(motor_id_lift_br_, 2, target_lift_pos_br_);
            add_motor_cmd(motor_id_lift_fr_, 2, target_lift_pos_fr_);
            add_motor_cmd(motor_id_extend_, 2, target_motor5_home_pos_);
            add_motor_cmd(motor_id_13_, 2, target_motor13_home_pos_);
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

            static bool prev_b_drive = false;
            static bool prev_a_drive = false;
            bool current_b_drive = latest_joy_.buttons[btn_grip_close_];
            bool current_a_drive = latest_joy_.buttons[btn_grip_open_];

            if (current_b_drive && !prev_b_drive) {
                motor5_drive_high_ = !motor5_drive_high_;
                RCLCPP_INFO(this->get_logger(), "Motor 5 target toggled to %s", motor5_drive_high_ ? "-7000" : "-1000");
            }
            if (current_a_drive && !prev_a_drive) {
                motor13_drive_high_ = !motor13_drive_high_;
                RCLCPP_INFO(this->get_logger(), "Motor 13 target toggled to %s", motor13_drive_high_ ? "-60000" : "-1000");
            }
            prev_b_drive = current_b_drive;
            prev_a_drive = current_a_drive;

            double motor5_goal_abs = origin_motor5_pos_ + (motor5_drive_high_ ? -7000.0 : -1000.0);
            double motor5_step = (lift_max_rpm_ * 0.03) * 6.0 * 0.02;
            double motor5_diff = motor5_goal_abs - target_motor5_drive_pos_;
            if (std::abs(motor5_diff) > motor5_step) {
                target_motor5_drive_pos_ += (motor5_diff > 0.0) ? motor5_step : -motor5_step;
            } else {
                target_motor5_drive_pos_ = motor5_goal_abs;
            }

            double motor13_goal_abs = origin_motor13_pos_ + (motor13_drive_high_ ? -60000.0 : -1000.0);
            const double dt = 0.01; // control_loop は 10ms 周期
            double motor13_diff = motor13_goal_abs - target_motor13_drive_pos_;
            if (motor13_max_vel_deg_s_ <= 0.0 || motor13_max_acc_deg_s2_ <= 0.0) {
                target_motor13_drive_pos_ = motor13_goal_abs;
                motor13_current_vel_deg_s_ = 0.0;
            } else {
                double dist_abs = std::abs(motor13_diff);
                if (dist_abs < 1e-6) {
                    target_motor13_drive_pos_ = motor13_goal_abs;
                    motor13_current_vel_deg_s_ = 0.0;
                } else {
                    double direction = (motor13_diff > 0.0) ? 1.0 : -1.0;
                    double v_abs = std::abs(motor13_current_vel_deg_s_);
                    double brake_dist = (v_abs * v_abs) / (2.0 * motor13_max_acc_deg_s2_);
                    double target_v_abs = (dist_abs > brake_dist)
                        ? motor13_max_vel_deg_s_
                        : std::sqrt(2.0 * motor13_max_acc_deg_s2_ * dist_abs);
                    double target_v = direction * target_v_abs;

                    double max_dv = motor13_max_acc_deg_s2_ * dt;
                    double dv = target_v - motor13_current_vel_deg_s_;
                    if (dv > max_dv) dv = max_dv;
                    if (dv < -max_dv) dv = -max_dv;
                    motor13_current_vel_deg_s_ += dv;

                    double step = motor13_current_vel_deg_s_ * dt;
                    if (std::abs(step) >= dist_abs) {
                        target_motor13_drive_pos_ = motor13_goal_abs;
                        motor13_current_vel_deg_s_ = 0.0;
                    } else {
                        target_motor13_drive_pos_ += step;
                    }
                }
            }

            add_motor_cmd(motor_id_extend_, 2, target_motor5_drive_pos_);
            add_motor_cmd(motor_id_13_, 2, target_motor13_drive_pos_);

            add_motor_cmd(motor_id_grip_, 1, 0.0f);
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
