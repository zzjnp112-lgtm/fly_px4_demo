#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>

#include <chrono>
#include <cmath>
#include <signal.h>

using namespace std::chrono_literals;

#define TargeyHeight 1.8

enum class FlightState {
    TAKEOFF = 1,
    FLYING,
    TRACKING,
    LANDING
};

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "Caught Ctrl+C, exiting..." << std::endl;
        rclcpp::shutdown();
        exit(0);
    }
}

class MavrosSubscriber : public rclcpp::Node {
public:
    MavrosSubscriber() : Node("mavros_subscriber") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        state_sub_ = create_subscription<mavros_msgs::msg::State>(
            "mavros/state", qos, std::bind(&MavrosSubscriber::state_callback, this, std::placeholders::_1));
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos, std::bind(&MavrosSubscriber::pose_callback, this, std::placeholders::_1));
    }

    mavros_msgs::msg::State getCurrentState() const { return current_state; }
    geometry_msgs::msg::PoseStamped getCurrentPose() const { return current_pose; }

private:
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
        current_state = *msg;
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose = *msg;
    }

    mavros_msgs::msg::State current_state;
    geometry_msgs::msg::PoseStamped current_pose;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

class Task : public rclcpp::Node {
public:
    Task(std::shared_ptr<MavrosSubscriber> mavros_sub)
    : Node("task"), mavros_subscriber_(mavros_sub), armed(false),
      flight_state(FlightState::TAKEOFF), cur_state(0), delay_started(false),
      hight_ok(false) {

        last_request_time = get_clock()->now();

        offb_set_mode = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        offb_set_mode->custom_mode = "OFFBOARD";

        arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        arm_cmd->value = true;

        set_mode_client = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        arming_client = create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

        local_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
        set_raw_pub = create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 10);

        timer_ = create_wall_timer(20ms, std::bind(&Task::runtask, this));
    }

private:
    std::shared_ptr<MavrosSubscriber> mavros_subscriber_;
    bool armed;
    FlightState flight_state;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_pub;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr set_raw_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    int cur_state;
    bool delay_started;
    rclcpp::Time delay_start_time;
    bool hight_ok;
    std::shared_ptr<mavros_msgs::srv::SetMode::Request> offb_set_mode;
    std::shared_ptr<mavros_msgs::srv::CommandBool::Request> arm_cmd;
    rclcpp::Time last_request_time;
    geometry_msgs::msg::PoseStamped target_position;
    int point_i = 0;

    double target_point[12][2] = {
        {1.00,0.00},
        {3.28,-4.00},
        {0.00,-4.00},
        {0.00,-3.20},
        {2.87,-3.20},
        {2.87,-2.40},
        {0.00,-2.40},
        {0.00,-1.60},
        {2.46,-1.60},
        {2.46,-0.80},
        {0.00,-0.80},
        {0.00,0.00}
    };

    void fly_dianvv_xy(double x, double y, double z, double yaw) {
        auto target = mavros_msgs::msg::PositionTarget();
        target.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                          mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                          mavros_msgs::msg::PositionTarget::IGNORE_AFZ;
        target.coordinate_frame = 1;
        target.position.x = x;
        target.position.y = y;
        target.position.z = z;
        target.yaw = yaw;
        set_raw_pub->publish(target);
    }

    void runtask() {
        auto current_state = mavros_subscriber_->getCurrentState();
        auto current_pose = mavros_subscriber_->getCurrentPose();

        if (current_state.mode != "OFFBOARD" &&
            (this->get_clock()->now() - last_request_time > rclcpp::Duration::from_seconds(5.0))) {
            set_mode_client->async_send_request(offb_set_mode);
            RCLCPP_INFO(this->get_logger(), "Trying to enable Offboard mode");
            last_request_time = this->get_clock()->now();
        } else {
            if (!current_state.armed &&
                (this->get_clock()->now() - last_request_time > rclcpp::Duration::from_seconds(5.0))) {
                arming_client->async_send_request(arm_cmd);
                RCLCPP_INFO(this->get_logger(), "Trying to arm vehicle");
                armed = true; // 实际应通过回调确认
                last_request_time = this->get_clock()->now();
            }
        }

        if (armed) {
            switch (flight_state) {
                case FlightState::TAKEOFF:
                    handle_takeoff(current_pose);
                    break;
                case FlightState::FLYING:
                    handle_flying(current_pose);
                    break;
                case FlightState::TRACKING:
                    handle_tracking(current_pose);
                    break;
                case FlightState::LANDING:
                    handle_landing(current_pose);
                    break;
            }
        }
    }

    void handle_takeoff(const geometry_msgs::msg::PoseStamped& current_pose) {
        if (current_pose.pose.position.z < (TargeyHeight - 0.05)) {
            if (cur_state == 0) {
                target_position = current_pose;
                target_position.pose.position.x = 0;
                target_position.pose.position.y = 0;
                target_position.pose.position.z = TargeyHeight;
                cur_state = 1;
            }
            RCLCPP_INFO(get_logger(), "高度: %lf", current_pose.pose.position.z);
            if (!hight_ok) {
                local_pose_pub->publish(target_position);
            }
        } else {
            if (!delay_started) {
                delay_start_time = get_clock()->now();
                delay_started = true;
            }
            auto elapsed = (get_clock()->now() - delay_start_time).seconds();
            if (elapsed >= 1.0) {
                flight_state = FlightState::FLYING;
                delay_started = false;
                hight_ok = true;
                target_position = current_pose;
                target_position.pose.position.x = 0;
                target_position.pose.position.y = 0;
                target_position.pose.position.z = TargeyHeight;
                RCLCPP_INFO(get_logger(), "起飞完成");
            }
        }
    }

    void handle_flying(const geometry_msgs::msg::PoseStamped& current_pose) {
        double x = target_point[point_i][0];
        double y = target_point[point_i][1];
        fly_dianvv_xy(x, y, TargeyHeight, 0);

        if (fabs(current_pose.pose.position.x - x) < 0.08 &&
            fabs(current_pose.pose.position.y - y) < 0.09) {
            RCLCPP_INFO(get_logger(), "飞行完成,已到达目标点(%lf,%lf)", x, y);
            delay_started = false;
            flight_state = FlightState::TRACKING;
        }
    }

    void handle_tracking(const geometry_msgs::msg::PoseStamped& current_pose) {
        if (!delay_started) {
            delay_start_time = get_clock()->now();
            delay_started = true;
        }
        auto elapsed = (get_clock()->now() - delay_start_time).seconds();
        if (elapsed >= 1.0) {
            delay_started = false;
            point_i = point_i + 1;
            RCLCPP_INFO(get_logger(), "point_i目前为 %d,当前坐标为(%lf,%lf)", point_i, current_pose.pose.position.x, current_pose.pose.position.y);
            if (point_i > 11) {
                flight_state = FlightState::LANDING;
            } else {
                flight_state = FlightState::FLYING;
            }
        }
    }

    void handle_landing(const geometry_msgs::msg::PoseStamped& current_pose) {
        if (current_pose.pose.position.z > 0.05) {
            if (cur_state == 1) {
                target_position = current_pose;
                target_position.pose.position.z = -0.2;
                cur_state = 0;
            }
            local_pose_pub->publish(target_position);
        } else {
            auto disarm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            disarm_cmd->value = false;
            arming_client->async_send_request(disarm_cmd);
            RCLCPP_INFO(get_logger(), "Trying to disarm vehicle");
            armed = false;
            delay_started = false;
            rclcpp::shutdown();
        }
    }
};

int main(int argc, char *argv[]) {
    std::signal(SIGINT, signal_handler);
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto MavrosSubscribernode = std::make_shared<MavrosSubscriber>();
    auto Tasknode = std::make_shared<Task>(MavrosSubscribernode);

    executor.add_node(MavrosSubscribernode);
    executor.add_node(Tasknode);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}