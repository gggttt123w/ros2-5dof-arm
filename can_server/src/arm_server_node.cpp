#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include "can_init.h"
#include "Servo_ctrl.h"

using namespace std::chrono_literals;

class ArmServer : public rclcpp::Node {
public:
    ArmServer() : Node("arm_server") {
        can_iface_   = declare_parameter<std::string>("can_interface", "can0");
        poll_ms_     = declare_parameter<int>("poll_period_ms", 50);
        joint_names_ = declare_parameter<std::vector<std::string>>(
            "joint_names", {"joint1","joint2","joint3","joint4","joint5","gripper"});
        servo_ids_   = declare_parameter<std::vector<int64_t>>("servo_ids", {0,1,2,3,4,5});

        // 默认按 500~2500 ↔ URDF 角度范围做线性映射，之后按实机标定再改
        servo_min_ = declare_parameter<std::vector<double>>("servo_min", {500,500,500,500,500,0});
        servo_max_ = declare_parameter<std::vector<double>>("servo_max", {2500,2500,2500,2500,2500,2000});
        angle_min_ = declare_parameter<std::vector<double>>(
            "angle_min", {-1.5708,-1.5708,-1.5708,-1.5708,-3.1416,0.0});
        angle_max_ = declare_parameter<std::vector<double>>(
            "angle_max", { 1.5708, 1.5708, 1.5708, 1.5708, 3.1416, 0.5});
        dir_ = declare_parameter<std::vector<double>>("dir", {1.0, 1.0, -1.0, -1.0, 1.0, -1.0});
        RCLCPP_INFO(get_logger(),
            "dir = [%g %g %g %g %g %g]",
            dir_[0], dir_[1], dir_[2], dir_[3], dir_[4], dir_[5]);

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        std::vector<struct can_filter> filters = {{0x200, CAN_SFF_MASK}};

        can_ = std::make_unique<Cansocket_object>(addr, filters);
        if (!can_->Can_Socket_Init(can_iface_)) {
            RCLCPP_FATAL(get_logger(), "CAN 初始化失败: %s", can_->Can_geterr().c_str());
            throw std::runtime_error("CAN init failed");
        }
        servo_ = std::make_unique<ServoCtrl_object>(*can_);

        pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_commands", 10,
            std::bind(&ArmServer::on_command, this, std::placeholders::_1));

        poll_timer_ = create_wall_timer(
            std::chrono::milliseconds(poll_ms_), std::bind(&ArmServer::on_poll, this));
        pub_timer_  = create_wall_timer(20ms, std::bind(&ArmServer::on_publish, this));

        rx_thread_ = std::thread(&ArmServer::rx_loop, this);

        RCLCPP_INFO(get_logger(), "arm_server 启动, CAN=%s, 轮询=%dms",
                    can_iface_.c_str(), poll_ms_);
    }

    ~ArmServer() override {
        running_ = false;
        if (rx_thread_.joinable()) rx_thread_.join();
    }

private:
    void rx_loop() {
        while (running_) {
            if (servo_->Info_wait(100)) {  
                awaiting_.store(false);
                stall_ = 0;
            }
        }
    }

    void on_poll(){
        if (awaiting_.load()) {
            if (++stall_ < 30) return;           
            stall_ = 0;
            awaiting_.store(false);               
            RCLCPP_WARN(get_logger(), "STM32 无响应，重试");
        }
        servo_->Statue_get(next_id_);
        next_id_ = static_cast<uint8_t>((next_id_ + 1) % servo_ids_.size());
        awaiting_.store(true);
    }
    // 发布 /joint_states
    void on_publish() {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = now();

        for (size_t i = 0; i < joint_names_.size(); ++i) {
            ServoStatus s = servo_->Read_Info(static_cast<uint8_t>(servo_ids_[i]));
            msg.name.push_back(joint_names_[i]);
            msg.position.push_back(servo_to_angle(i, s.position));
        }
        pub_->publish(msg);
    }

    void on_command(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t k = 0; k < msg->name.size() && k < msg->position.size(); ++k) {
            int i = index_of(msg->name[k]);
            if (i < 0) continue;
            uint16_t pos = angle_to_servo(i, msg->position[k]);
            servo_->Position_set(static_cast<uint8_t>(servo_ids_[i]), pos, 200);
        }
    }

    int index_of(const std::string& name) const {
        for (size_t i = 0; i < joint_names_.size(); ++i)
            if (joint_names_[i] == name) return static_cast<int>(i);
        return -1;
    }

    double servo_to_angle(size_t i, uint16_t pos) const {
        double sp = servo_max_[i] - servo_min_[i];
        double ap = angle_max_[i] - angle_min_[i];
        if (sp == 0.0) return angle_min_[i];

        double t = (pos - servo_min_[i]) / sp;        // 舵机位置归一化 0~1
        if (dir_[i] < 0.0) t = 1.0 - t;               // ← 反射
        return angle_min_[i] + t * ap;
    }

    uint16_t angle_to_servo(size_t i, double a) const {
        double sp = servo_max_[i] - servo_min_[i];
        double ap = angle_max_[i] - angle_min_[i];
        if (ap == 0.0) return static_cast<uint16_t>(servo_min_[i]);

        double t = (a - angle_min_[i]) / ap;          // 角度归一化 0~1
        if (dir_[i] < 0.0) t = 1.0 - t;               // ← 反射
        double p = servo_min_[i] + t * sp;
        if (p < servo_min_[i]) p = servo_min_[i];
        if (p > servo_max_[i]) p = servo_max_[i];
        return static_cast<uint16_t>(p);
    }

    std::string can_iface_;
    int poll_ms_ = 200;

    std::vector<std::string> joint_names_;
    std::vector<int64_t>     servo_ids_;
    std::vector<double>      servo_min_, servo_max_, angle_min_, angle_max_;

    std::unique_ptr<Cansocket_object> can_;
    std::unique_ptr<ServoCtrl_object> servo_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr poll_timer_, pub_timer_;

    std::thread       rx_thread_;
    std::atomic<bool> running_{true};
    uint8_t           next_id_ = 0;
    std::atomic<bool> awaiting_{false};    // 有一条查询在等回复
    int               stall_ = 0;          // 连续没回复的轮数
    std::vector<double> dir_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmServer>());
    rclcpp::shutdown();
    return 0;
}