#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <chrono>
#include <cmath>
#include <Eigen/Dense>

using namespace std::chrono_literals;

#define TargeyHeight 1.5   // 追踪/悬停高度 (m)
#define TargeySpeed  0.5   // 起飞上升速度 (m/s)

// ============================================================================
// MPC 控制器（离散 LQR，等价于无限时域无约束 MPC）
// 每轴独立：状态 [offset(米), integral(米·秒)]
//   模型：offset' = offset - v*dt          （无人机速度 v 减小偏移）
//         integ'  = integ + offset*dt      （积分，消除匀速目标的残差）
//   输出：速度 v = -K_e*offset - K_i*integral
// ============================================================================
class MpcController {
public:
    MpcController(double dt, double px_per_m, double q_e, double q_i, double r, double vmax)
        : dt_(dt), px_per_m_(px_per_m), vmax_(vmax), integral_(0.0)
    {
        Eigen::Matrix2d A;
        A << 1.0, 0.0,
             dt, 1.0;
        Eigen::Vector2d B;
        B << -dt, 0.0;
        Eigen::Matrix2d Q;
        Q << q_e, 0.0,
             0.0, q_i;
        gain_ = lqr(A, B, Q, r);
    }

    // 输入像素误差 e_px，输出速度指令 v (m/s)
    double update(double e_px) {
        double offset = e_px / px_per_m_;          // 像素误差 -> 米
        integral_ += offset * dt_;                  // 积分（反残差）
        // 反积分饱和，防止 windup
        double imax = (std::abs(gain_(1)) > 1e-9) ? vmax_ / std::abs(gain_(1)) : 0.0;
        integral_ = std::clamp(integral_, -imax, imax);
        double v = -(gain_(0) * offset + gain_(1) * integral_);
        return std::clamp(v, -vmax_, vmax_);
    }

    void reset() { integral_ = 0.0; }

private:
    static Eigen::Vector2d lqr(const Eigen::Matrix2d& A, const Eigen::Vector2d& B,
                               const Eigen::Matrix2d& Q, double R) {
        Eigen::Matrix2d P = Q;
        Eigen::Vector2d K = Eigen::Vector2d::Zero();
        for (int i = 0; i < 2000; ++i) {
            double S = R + (B.transpose() * P * B)(0,0);
            if (S < 1e-12) break;
            K = (B.transpose() * P * A) / S;
            Eigen::Matrix2d Pn = Q + A.transpose() * P * A
                - (A.transpose() * P * B) * (B.transpose() * P * A) / S;
            if ((Pn - P).norm() < 1e-10) { P = Pn; break; }
            P = Pn;
        }
        return K;
    }

    double dt_, px_per_m_, vmax_;
    double integral_;
    Eigen::Vector2d gain_;
};

// ============================================================================
// MAVROS 状态/位姿订阅
// ============================================================================
class MavrosSubscriber : public rclcpp::Node {
public:
    MavrosSubscriber() : Node("mavros_subscriber") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        state_sub_ = create_subscription<mavros_msgs::msg::State>(
            "mavros/state", qos, [this](const mavros_msgs::msg::State::SharedPtr m){ current_state_ = *m; });
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos, [this](const geometry_msgs::msg::PoseStamped::SharedPtr m){ current_pose_ = *m; });
    }
    mavros_msgs::msg::State state() const { return current_state_; }
    geometry_msgs::msg::PoseStamped pose() const { return current_pose_; }

private:
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_pose_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

// ============================================================================
// 任务节点：状态机（起飞 -> 飞行到目标 -> MPC追踪 -> 悬停2s -> 降落）
// ============================================================================
enum class FlightState { TAKEOFF, FLYING, TRACKING, LANDING };

class Task : public rclcpp::Node {
public:
    Task(std::shared_ptr<MavrosSubscriber> mav)
        : Node("task"), mav_(mav), armed_(false), state_(FlightState::TAKEOFF),
          cur_state_(0), delay_started_(false), hight_ok_(false), pid_ok_(false)
    {
        last_req_ = now();

        offb_set_mode_ = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        offb_set_mode_->custom_mode = "OFFBOARD";
        arm_cmd_ = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        arm_cmd_->value = true;

        set_mode_cli_ = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        arming_cli_ = create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

        local_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
        set_raw_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 10);
        set_v_pub_ = create_publisher<geometry_msgs::msg::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

        // 二维码中心像素（gemini2 1280x720 -> 中心 640,360）
        qr_sub_ = create_subscription<geometry_msgs::msg::Point>(
            "/zzjnp", 10, [this](const geometry_msgs::msg::Point::SharedPtr m){ qr_ = *m; });

        // MPC 参数：像素/米 = fx/高度 ≈ 685.9/1.5 ≈ 457
        const double px_per_m = 685.9 / TargeyHeight;
        // 每轴 MPC：q_e=1(偏移权重) q_i=0.15(积分权重) r=0.08(速度权重) vmax=0.5
        mpc_x_ = std::make_shared<MpcController>(0.02, px_per_m, 0.9, 0.1, 0.65, 1.5);
        mpc_y_ = std::make_shared<MpcController>(0.02, px_per_m, 0.9, 0.1, 0.65, 1.5);

        timer_ = create_wall_timer(20ms, std::bind(&Task::run, this));
    }

private:
    void fly_pos(double x, double y, double z, double yaw) {
        auto t = mavros_msgs::msg::PositionTarget();
        t.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                      mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                      mavros_msgs::msg::PositionTarget::IGNORE_AFZ;
        t.coordinate_frame = 1;
        t.position.x = x; t.position.y = y; t.position.z = z; t.yaw = yaw;
        t.velocity.x = 0; t.velocity.y = 0; t.velocity.z = 0;
        set_raw_pub_->publish(t);
    }

    void fly_vel(double vx, double vy, double vz, double vyaw) {
        if (std::fabs(vx) < 1e-3 && std::fabs(vy) < 1e-3 && std::fabs(vz) < 1e-3 && std::fabs(vyaw) < 1e-3) return;
        auto t = geometry_msgs::msg::Twist();
        t.linear.x = vx; t.linear.y = vy; t.linear.z = vz;
        t.angular.z = vyaw; t.angular.x = 0; t.angular.y = 0;
        set_v_pub_->publish(t);
    }

    void run() {
        auto st = mav_->state();
        auto pose = mav_->pose();
        auto nowt = now();

        // 用真实 arm 状态
        armed_ = st.armed;

        // 保活（未 arm 时发位置 setpoint，维持 OFFBOARD）
        if (!armed_) {
            target_ = pose;
            target_.pose.position.x = 0;
            target_.pose.position.y = 0;
            target_.pose.position.z = TargeyHeight;
            local_pose_pub_->publish(target_);
        }

        // 不自动切 OFFBOARD；但一旦已处于 OFFBOARD 且未 arm，自动 arm
        if (st.mode == "OFFBOARD" && !st.armed) {
            if (nowt - last_req_ > rclcpp::Duration::from_seconds(2.0)) {
                arming_cli_->async_send_request(arm_cmd_);
                RCLCPP_INFO(get_logger(), "OFFBOARD 模式已检测到，自动 arm");
                last_req_ = nowt;
            }
        }

        if (!armed_) return;

        switch (state_) {
            case FlightState::TAKEOFF:  takeoff(pose); break;
            case FlightState::FLYING:   flying(pose);  break;
            case FlightState::TRACKING: tracking(pose); break;
            case FlightState::LANDING:  landing(pose); break;
        }
    }

    void takeoff(const geometry_msgs::msg::PoseStamped& pose) {
        double z = pose.pose.position.z;
        if (z < TargeyHeight - 0.05) {
            // 打点：飞向 (0,0,TargeyHeight)，由位置控制器负责爬升
            fly_pos(0.0, 0.0, TargeyHeight, 0.0);
        } else {
            if (!delay_started_) { delay_start_time_ = now(); delay_started_ = true; }
            if ((now() - delay_start_time_).seconds() >= 1.0) {
                delay_started_ = false; hight_ok_ = true;
                state_ = FlightState::FLYING;
                RCLCPP_INFO(get_logger(), "起飞完成");
            }
        }
    }

    void flying(const geometry_msgs::msg::PoseStamped& pose) {
        // 飞到目标点 (1,0)（小车位置）正上方
        fly_pos(1.0, 0.0, TargeyHeight, 0.0);
        double dx = pose.pose.position.x - 1.0;
        double dy = pose.pose.position.y - 0.0;
        if (std::fabs(dx) < 0.1 && std::fabs(dy) < 0.1) {
            state_ = FlightState::TRACKING;
            mpc_x_->reset(); mpc_y_->reset();
            RCLCPP_INFO(get_logger(), "飞行完成，进入追踪");
        }
    }

    void tracking(const geometry_msgs::msg::PoseStamped& pose) {
        // 像素误差：下视相机 gemini2，图像中心 (640,360)
        // 轴映射（与 fixed_point 一致，方向已反转）：无人机 X <- -(图像 Y 误差)，无人机 Y <- -(图像 X 误差)
        double e_x = 360.0 - qr_.y;   // -(图像 Y 误差) -> 无人机 X
        double e_y = 640.0 - qr_.x;   // -(图像 X 误差) -> 无人机 Y

        double vx = mpc_x_->update(e_x);
        double vy = mpc_y_->update(e_y);

        if (pose.pose.position.z < 0.8) {
            fly_pos(pose.pose.position.x, pose.pose.position.y, TargeyHeight, 0.0);
            return;
        }

        // 目标接近中心 → 悬停保持（暂不降落）
        bool centered = (std::fabs(e_x) < 8.0 && std::fabs(e_y) < 8.0);
        if (centered) {
            //fly_pos(pose.pose.position.x, pose.pose.position.y, TargeyHeight, 0.0);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "已对准，悬停保持");
            return;
        }

        fly_vel(vx, vy, 0.0, 0.0);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "MPC -> vx: %.2f, vy: %.2f", vx, vy);
    }

    void landing(const geometry_msgs::msg::PoseStamped& pose) {
        if (pose.pose.position.z > 0.05) {
            if (cur_state_ == 1) {
                target_ = pose;
                target_.pose.position.z = -0.2;
                cur_state_ = 0;
            }
            local_pose_pub_->publish(target_);
        } else {
            auto d = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            d->value = false;
            arming_cli_->async_send_request(d);
            RCLCPP_INFO(get_logger(), "Trying to disarm");
            armed_ = false; delay_started_ = false;
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<MavrosSubscriber> mav_;
    bool armed_;
    FlightState state_;
    int cur_state_;
    bool delay_started_;
    bool hight_ok_;
    bool pid_ok_;
    rclcpp::Time delay_start_time_;
    rclcpp::Time last_req_;
    geometry_msgs::msg::PoseStamped target_;
    geometry_msgs::msg::Point qr_;

    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_cli_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_cli_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_pub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr set_raw_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr set_v_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr qr_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<mavros_msgs::srv::SetMode::Request> offb_set_mode_;
    std::shared_ptr<mavros_msgs::srv::CommandBool::Request> arm_cmd_;

    std::shared_ptr<MpcController> mpc_x_;
    std::shared_ptr<MpcController> mpc_y_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto mav = std::make_shared<MavrosSubscriber>();
    auto task = std::make_shared<Task>(mav);

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(mav);
    exec.add_node(task);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
