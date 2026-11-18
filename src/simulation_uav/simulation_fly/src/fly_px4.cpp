#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include "lib/Arduino.h"
#include "lib/fixed_point.hpp"
#include "lib/pidcontroller.h"

using namespace std::chrono_literals;


#define BUFFER_SIZE 1024
#define TargeyHeight 1.1


struct PidControl {
  double x;
  double y;
} pid;

struct YUAN{
    double c;
    double j;
}yuan;




std::mutex socket_data_mutex;
std::mutex pid_mutex;
std::mutex label_mutex;
std::mutex a_mutex;


class Subscriber : public rclcpp::Node {
public:
    Subscriber() : Node("subscriber") {
        create_subscription<std_msgs::msg::Float64>("pose_c", 10, [this](const std_msgs::msg::Float64 &msg) {
           yuan.c  = msg.data;
        });
        create_subscription<std_msgs::msg::Float64>("pose_j", 10, [this](const std_msgs::msg::Float64 &msg) {
          yuan.j   = msg.data;
        });

    }

private:

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscrition_class_stuas;
};

class PidRead : public rclcpp::Node { 
public:     
    PidRead() : Node("PidRead") {         
        pid_controller::PidParams pidparax{0.12, 0.0, 0.05, 10.0, 30, 0.0}; //静态0.1
        pid_controller::PidParams pidparay{0.12, 0.0, 0.05, 10.0, 31, 0.0}; //静态0.1
        
        fixed_point_black_ = std::make_shared<FixedPoint>(             
            std::static_pointer_cast<rclcpp::Node>(std::shared_ptr<PidRead>(this, [](PidRead*){})),
            "/zzjnp",             
            fixed_point::Point(640/2.0, 480/2.0),
            pidparax,             
            pidparay         
        );                  
        
        Serialtimer_ = create_wall_timer(100ms, std::bind(&PidRead::PDRead, this));     
    }  

private:     
    void PDRead() {                       
        double xx = (fixed_point_black_->getBoundedOutput().x);         
        double yy = (fixed_point_black_->getBoundedOutput().y);
        //RCLCPP_INFO(get_logger(), "PID -> X: %.2f, Y: %.2f", xx/100, yy/100);  
        {
            std::lock_guard<std::mutex> lock(pid_mutex);
            pid.x = xx / 100;
            pid.y = yy / 100;
        }
    }      
    
    rclcpp::TimerBase::SharedPtr Serialtimer_;     
    std::shared_ptr<FixedPoint> fixed_point_black_; 
};

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

class Task : public rclcpp::Node {
public:
    Task(std::shared_ptr<MavrosSubscriber> mavros_sub)
    : Node("task"), mavros_subscriber_(mavros_sub), armed(false), 
      flight_state(FlightState::TAKEOFF), cur_state(0), delay_started(false),
      hight_ok(false), pid_ok(false), fly_dian(true) {
        
        last_request_time = get_clock()->now();
        
        offb_set_mode = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        offb_set_mode->custom_mode = "OFFBOARD";
        
        arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        arm_cmd->value = true;

        set_mode_client = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        arming_client = create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

        local_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
        set_raw_pub = create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 10);
        set_v_pub = create_publisher<geometry_msgs::msg::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

        Serialtimer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&Task::runtask, this));
    }

private:    
    std::shared_ptr<MavrosSubscriber> mavros_subscriber_;
    bool armed;
    FlightState flight_state;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_pub;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr set_raw_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr set_v_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr a_ok_pub;
    rclcpp::TimerBase::SharedPtr Serialtimer_;
    int cur_state;
    bool delay_started;
    rclcpp::Time delay_start_time;
    bool hight_ok;
    bool pid_ok;
    bool fly_dian;
    std::shared_ptr<mavros_msgs::srv::SetMode::Request> offb_set_mode;
    std::shared_ptr<mavros_msgs::srv::CommandBool::Request> arm_cmd;
    rclcpp::Time last_request_time;
    geometry_msgs::msg::PoseStamped target_position;

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
        target.velocity.x = 0;
        target.velocity.y = 0;
        target.velocity.z = 0;
        set_raw_pub->publish(target);
    }

    void fly_v_xy(double vx, double vy, double vz, double vyaw) {
        if (fabs(vx) < 0.001 && fabs(vy) < 0.001 && fabs(vz) < 0.001 && fabs(vyaw) < 0.001) {
            return;
        }
        
        auto target = geometry_msgs::msg::Twist();
        target.linear.x = vx;
        target.linear.y = vy;
        target.linear.z = vz;
        target.angular.z = vyaw;
        target.angular.x = 0;
        target.angular.y = 0;
        set_v_pub->publish(target);
    }

    void runtask() {
        auto current_state = mavros_subscriber_->getCurrentState();
        auto current_pose = mavros_subscriber_->getCurrentPose();
        
        if (current_state.mode == "OFFBOARD") {
            if (!current_state.armed && 
                (get_clock()->now() - last_request_time > rclcpp::Duration::from_seconds(5.0))) {
                arming_client->async_send_request(arm_cmd);
                RCLCPP_INFO(get_logger(), "OFFBOARD mode detected - Arming vehicle");
                armed = true;
                last_request_time = get_clock()->now();
            }
        } else {
            Arduino::led_1s();
            sleep(1);
        }

        if (armed) {
            switch(flight_state) {
                case FlightState::TAKEOFF:
                    handle_takeoff(current_pose);
                    break;
                case FlightState::FLYING:
                    handle_flying(current_pose,1,0);
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
                flight_state = FlightState::TRACKING;
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

    void handle_flying(const geometry_msgs::msg::PoseStamped& current_pose, double x, double y) {
            fly_dianvv_xy(x, y, TargeyHeight, 0);
            if((current_pose.pose.position.x - x) <0.1&&(current_pose.pose.position.x - x )>-0.1){
            RCLCPP_INFO(get_logger(), "飞行完成");
             flight_state = FlightState::TRACKING;
    }
}

    void handle_tracking(const geometry_msgs::msg::PoseStamped& current_pose){
         // if(fabs(pid.x)==0.4 && fabs(pid.y)==0.41){
           // RCLCPP_INFO(get_logger(), "数据异常");
        // return;
       //}
         if((pid.x+pid.y) ==0){
            RCLCPP_INFO(get_logger(), "无目标，保持当前位置");
          fly_dianvv_xy(current_pose.pose.position.x,current_pose.pose.position.y, TargeyHeight, 0);
       }
        fly_v_xy(pid.x, pid.y, 0, 0);
        RCLCPP_INFO(get_logger(), "PID -> X: %.2f, Y: %.2f", pid.x, pid.y);
       if(current_pose.pose.position.z < 0.8){
         RCLCPP_INFO(get_logger(), "掉高");
           fly_dianvv_xy(current_pose.pose.position.x, current_pose.pose.position.y, TargeyHeight, 0);
       }
       if((fabs(pid.x)+fabs(pid.y))<0.02  && pid.x!=0 && pid.y!=0){
           if(!pid_ok){
               pid_ok = true;
                RCLCPP_INFO(get_logger(), "位置调整完成");
                flight_state = FlightState::LANDING;
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
    Arduino::serinit();
    
    rclcpp::executors::MultiThreadedExecutor executor;

    auto MavrosSubscribernode = std::make_shared<MavrosSubscriber>();
    auto Subscribernode = std::make_shared<Subscriber>();
    auto Tasknode = std::make_shared<Task>(MavrosSubscribernode);
    auto PidReadnode = std::make_shared<PidRead>();
    

    executor.add_node(MavrosSubscribernode);
    executor.add_node(Subscribernode);
    executor.add_node(Tasknode);
    executor.add_node(PidReadnode);
    
    executor.spin();
    rclcpp::shutdown(); 
    return 0;
}
