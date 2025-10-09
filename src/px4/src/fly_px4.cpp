#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
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
using json = nlohmann::json;

#define BUFFER_SIZE 1024
#define TargeyHeight 0.9

struct SocketData {
  double x;
  double y;
  bool socket = true;
} socket_data;

struct PidControl {
  double x;
  double y;
} pid;

struct Label {
  double count;
  double hou;
  double hu;
  double kongque;
  double lang;
  double xiang;
} label;

struct WaypointData {
  double x;
  double y;
  double active;
  double repeat;
  double x_erro;
  double y_erro;
} a;

std::mutex socket_data_mutex;
std::mutex pid_mutex;
std::mutex label_mutex;
std::mutex a_mutex;

class Socket : public rclcpp::Node {
public:
    Socket() : Node("socket"), send_fd(-1) {
        send_server_addr.sin_family = AF_INET;
        send_server_addr.sin_port = htons(9999);
        send_server_addr.sin_addr.s_addr = inet_addr("10.42.0.16");
        
        coord_map = {
            {"A1B1", {0, 4}}, {"A4B1", {0, 2.5}}, {"A7B1", {0, 1}},
            {"A1B2", {0.5, 4}}, {"A4B2", {0.5, 2.5}}, {"A7B2", {0.5, 1}},
            {"A1B3", {1, 4}}, {"A4B3", {1, 2.5}}, {"A7B3", {1, 1}},
            {"A1B4", {1.5, 4}}, {"A4B4", {1.5, 2.5}}, {"A7B4", {1.5, 1}},
            {"A1B5", {2, 4}}, {"A4B5", {2, 2.5}}, {"A7B5", {2, 1}},
            {"A1B6", {2.5, 4}}, {"A4B6", {2.5, 2.5}}, {"A7B6", {2.5, 1}},
            {"A1B7", {3, 4}}, {"A4B7", {3, 2.5}}, {"A7B7", {3, 1}},
            {"A2B1", {0, 3.5}}, {"A5B1", {0, 2}}, {"A8B1", {0, 0.5}},
            {"A2B2", {0.5, 3.5}}, {"A5B2", {0.5, 2}}, {"A8B2", {0.5, 0.5}},
            {"A2B3", {1, 3.5}}, {"A5B3", {1, 2}}, {"A8B3", {1, 0.5}},
            {"A2B4", {1.5, 3.5}}, {"A5B4", {1.5, 2}}, {"A8B4", {1.5, 0.5}},
            {"A2B5", {2, 3.5}}, {"A5B5", {2, 2}}, {"A8B5", {2, 0.5}},
            {"A2B6", {2.5, 3.5}}, {"A5B6", {2.5, 2}}, {"A8B6", {2.5, 0.5}},
            {"A2B7", {3, 3.5}}, {"A5B7", {3, 2}}, {"A8B7", {3, 0.5}},
            {"A3B1", {0, 3}}, {"A6B1", {0, 1.5}}, {"A9B1", {0, 0}},
            {"A3B2", {0.5, 3}}, {"A6B2", {0.5, 1.5}}, {"A9B2", {0.5, 0}},
            {"A3B3", {1, 3}}, {"A6B3", {1, 1.5}}, {"A9B3", {1, 0}},
            {"A3B4", {1.5, 3}}, {"A6B4", {1.5, 1.5}}, {"A9B4", {1.5, 0}},
            {"A3B5", {2, 3}}, {"A6B5", {2, 1.5}}, {"A9B5", {2, 0}},
            {"A3B6", {2.5, 3}}, {"A6B6", {2.5, 1.5}}, {"A9B6", {2.5, 0}},
            {"A3B7", {3, 3}}, {"A6B7", {3, 1.5}}, {"A9B7", {3, 0}}
        };
        
        RCLCPP_INFO(this->get_logger(), "Socket节点启动，发送到: 10.42.0.16:9999");
    }

    ~Socket() {
        if (send_fd >= 0) {
            close(send_fd);
        }
    }

    void send_data(double a, double shijue) {       
        if (send_fd < 0) {
            setup_send_connection();
            return;
        }

        int item_index = a;
        float x = socket_data.x;
        float y = socket_data.y;
        int counts = int(shijue);

        std::string area = find_area(x, y);
        
        char send_buffer[256];
        int len = snprintf(send_buffer, sizeof(send_buffer), "(%d,%s,%d)",
                          item_index, area.c_str(), counts);
        
        if (socket_data.socket && label.count != 0) {
            if (send(send_fd, send_buffer, len, 0) < 0) {
                RCLCPP_ERROR(this->get_logger(), "发送数据失败: %s", strerror(errno));
                close(send_fd);
                send_fd = -1;
            } else {
                RCLCPP_INFO(this->get_logger(), "发送物品 %d: 区域 %s", item_index, area.c_str());
            }
        }   
    }

private:
    int send_fd;
    struct sockaddr_in send_server_addr;
    std::map<std::string, std::pair<float, float>> coord_map;
    
    std::string find_area(float x, float y) {
        const float tolerance = 0.1f;
        
        for (const auto& [area, coords] : coord_map) {
            float dx = std::abs(x - coords.first);
            float dy = std::abs(y - coords.second);
            
            if (dx <= tolerance && dy <= tolerance) {
                return area;
            }
        }
        
        return "Unknown";
    }

    void setup_send_connection() {
        send_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (send_fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "发送Socket创建失败: %s", strerror(errno));
            return;
        }

        if (connect(send_fd, (struct sockaddr*)&send_server_addr, sizeof(send_server_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "连接10.42.0.16:9999失败: %s", strerror(errno));
            close(send_fd);
            send_fd = -1;
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "已连接到发送服务器 10.42.0.16:9999");
    }
};

class Subscriber : public rclcpp::Node {
public:
    Subscriber(std::shared_ptr<Socket> socket_node) : Node("subscriber"), socket_node_(socket_node) {
        create_subscription<std_msgs::msg::Float64>("/pose_x_A", 10, [this](const std_msgs::msg::Float64 &msg) {
            a.x = msg.data;
        });
        create_subscription<std_msgs::msg::Float64>("/pose_y_A", 10, [this](const std_msgs::msg::Float64 &msg) {
            a.y = msg.data;
        });
        create_subscription<std_msgs::msg::Float64>("/ok", 10, [this](const std_msgs::msg::Float64 &msg) {
            a.active = msg.data;
        });
        create_subscription<std_msgs::msg::Float64>("pose_count", 10, [this](const std_msgs::msg::Float64 &msg) {
            label.count = msg.data;
        });
        create_subscription<std_msgs::msg::Float64>("/report", 10, [this](const std_msgs::msg::Float64 &msg) {
            a.repeat = msg.data;
        });
        create_subscription<std_msgs::msg::Float64>("pose_x_A_prev", 10, [this](const std_msgs::msg::Float64 &msg) {
            a.x_erro = msg.data;
        });
        create_subscription<std_msgs::msg::Float64>("pose_y_A_prev", 10, [this](const std_msgs::msg::Float64 &msg) {
            a.y_erro = msg.data;
        });

        subscrition_class_stuas = create_subscription<std_msgs::msg::String>(
            "class_statistics", 10, 
            std::bind(&Subscriber::class_stats_callback, this, std::placeholders::_1));
    }

private:
    void class_stats_callback(const std_msgs::msg::String::SharedPtr msg) const {
        try {
            json stats_data = json::parse(msg->data);
            json class_counts = stats_data["class_counts"];
            
            if (!class_counts.empty()) {
                for (auto& [label_id_str, class_info] : class_counts.items()) {
                    int label_id = std::stoi(label_id_str);
                    std::string label_name = class_info.value("name", "unknown");
                    int count = class_info.value("count", 0);
                    
                    RCLCPP_INFO(this->get_logger(), "标签号: %d, 名称: %s, 数量: %d", 
                                label_id, label_name.c_str(), count);
                    
                    process_single_class(label_id, label_name, count);
                }
            }
        } catch (const json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON解析错误: %s", e.what());
        }
    }

    void process_single_class(int label_id, const std::string& label_name, int count) const {
        switch (label_id) {
            case 0:
                label.hou = count;
                for (int i = 0; i < count; i++) {
                    socket_node_->send_data(0, label.hou);
                    RCLCPP_INFO(this->get_logger(), "发送猴子数据 [%d/%d]", i+1, count);
                }
                break;
            case 1:
                label.hu = count;
                for (int i = 0; i < count; i++) {
                    socket_node_->send_data(1, label.hu);
                    RCLCPP_INFO(this->get_logger(), "发送虎子数据 [%d/%d]", i+1, count);
                }
                break;
            case 2:
                label.kongque = count;
                for (int i = 0; i < count; i++) {
                    socket_node_->send_data(2, label.kongque);
                    RCLCPP_INFO(this->get_logger(), "发送孔雀数据 [%d/%d]", i+1, count);
                }
                break;
            case 3:
                label.lang = count;
                for (int i = 0; i < count; i++) {
                    socket_node_->send_data(3, label.lang);
                    RCLCPP_INFO(this->get_logger(), "发送浪数据 [%d/%d]", i+1, count);
                }
                break;
            case 4:
                label.xiang = count;
                for (int i = 0; i < count; i++) {
                    socket_node_->send_data(4, label.xiang);
                    RCLCPP_INFO(this->get_logger(), "发送象数据 [%d/%d]", i+1, count);
                }
                break;
        }
    }

    std::shared_ptr<Socket> socket_node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscrition_class_stuas;
};

class PidRead : public rclcpp::Node { 
public:     
    PidRead() : Node("PidRead") {         
        pid_controller::PidParams pidpara{0.1, 0.0, 0.05, 10.0, 30, 0.0};
        
        fixed_point_black_ = std::make_shared<FixedPoint>(             
            std::static_pointer_cast<rclcpp::Node>(std::shared_ptr<PidRead>(this, [](PidRead*){})),
            "/zzjnp",             
            fixed_point::Point(640/2.0, 360/2.0),
            pidpara,             
            pidpara         
        );                  
        
        Serialtimer_ = create_wall_timer(100ms, std::bind(&PidRead::PDRead, this));     
    }  

private:     
    void PDRead() {                       
        double xx = (fixed_point_black_->getBoundedOutput().x);         
        double yy = (fixed_point_black_->getBoundedOutput().y);
        
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
    WAIT,
    LANDING_HIGHT,
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
        a_ok_pub = create_publisher<std_msgs::msg::Float64>("/a_ok", 10);

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
                    handle_flying(current_pose);
                    break;
                case FlightState::WAIT:
                    handle_wait(current_pose);
                    break;
                case FlightState::LANDING_HIGHT:
                    handle_landing_height(current_pose);
                    break;
                case FlightState::LANDING:
                    handle_landing(current_pose);
                    break;
            }
        }
    }

    void handle_takeoff(const geometry_msgs::msg::PoseStamped& current_pose) {
        if (current_pose.pose.position.z < TargeyHeight) {
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
        if (fly_dian) {
            fly_dianvv_xy(a.x, a.y, TargeyHeight, 0);
        }
        
        if (fabs(a.x - current_pose.pose.position.x) < 0.06 && 
            fabs(a.y - current_pose.pose.position.y) < 0.06) {
            flight_state = FlightState::WAIT;
            delay_started = false;
            socket_data.socket = false;
            RCLCPP_INFO(get_logger(), "到达航点，进入停留状态");
        } else {
            double ok = 0;
            std_msgs::msg::Float64 msg;
            msg.data = ok;
            a_ok_pub->publish(msg);
        }
    }

    void handle_wait(const geometry_msgs::msg::PoseStamped& current_pose) {
        target_position = current_pose;
        target_position.pose.position.z = TargeyHeight;
        local_pose_pub->publish(target_position);
        
        if (!delay_started) {
            delay_start_time = get_clock()->now();
            delay_started = true;
            socket_data.socket = true;
            RCLCPP_INFO(get_logger(), "到达目标点，开启socket发送并停留0.5秒...");
        }
        
        auto current_time = get_clock()->now();
        auto elapsed = (current_time - delay_start_time).seconds();
        
        if (elapsed < 0.5) {
            socket_data.socket = true;
        } else {
            double pid_x, pid_y;
            {
                std::lock_guard<std::mutex> lock(pid_mutex);
                pid_x = pid.x;
                pid_y = pid.y;
            }
            
            if (a.repeat != 1 && (fabs(pid_x) + fabs(pid_y)) > 0.001) {
                if (!pid_ok) {
                    fly_v_xy(pid_x, pid_y, 0, 0);
                    fly_dian = false;
                    socket_data.socket = false;
                    
                    if (label.count > 1) {
                        if ((current_time - delay_start_time).seconds() >= 4.5) {
                            pid_ok = true;
                            socket_data.socket = false;
                            fly_dian = true;
                            publish_waypoint_complete();
                            RCLCPP_INFO(get_logger(), "检测到%d个目标，数据发送+PID调整完成", (int)label.count);
                        }
                    } else {
                        if (fabs(pid_x) < 0.02 && fabs(pid_y) < 0.02) {
                            pid_ok = true;
                            socket_data.socket = false;
                            fly_dian = true;
                            publish_waypoint_complete();
                            RCLCPP_INFO(get_logger(), "PID到达阈值");
                        }
                    }
                } else {
                    socket_data.socket = false;
                    publish_waypoint_complete();
                }
            } else {
                socket_data.socket = false;
                fly_dian = true;
                pid_ok = true;
                publish_waypoint_complete();
            }
            
            if (a.active == 1 && pid_ok) {
                socket_data.x = a.x_erro;
                socket_data.y = a.y_erro;
                RCLCPP_INFO(get_logger(), "接收到新航点 -> X: %.2f, Y: %.2f", a.x, a.y);
                flight_state = FlightState::FLYING;
                pid_ok = false;
                delay_started = false;
            } else if (a.active == 0 && pid_ok) {
                flight_state = FlightState::LANDING_HIGHT;
                delay_started = false;
                RCLCPP_INFO(get_logger(), "任务完成，准备降落");
            }
        }
    }

    void handle_landing_height(const geometry_msgs::msg::PoseStamped& current_pose) {
        if (!delay_started) {
            delay_start_time = get_clock()->now();
            delay_started = true;
            target_position = current_pose;
            target_position.pose.position.z = 0.7;
        }
        
        local_pose_pub->publish(target_position);
        auto elapsed = (get_clock()->now() - delay_start_time).seconds();
        
        if (elapsed >= 6.0) {
            Arduino::bigbomb_3();
            flight_state = FlightState::LANDING;
        }
    }

    void handle_landing(const geometry_msgs::msg::PoseStamped& current_pose) {
        if (current_pose.pose.position.z > 0.05) {
            if (cur_state == 1) {
                target_position = current_pose;
                target_position.pose.position.x = 0;
                target_position.pose.position.y = 0;
                target_position.pose.position.z = -0.1;
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

    void publish_waypoint_complete() {
        double ok = 1;
        std_msgs::msg::Float64 msg;
        msg.data = ok;
        a_ok_pub->publish(msg);
    }
};

int main(int argc, char *argv[]) {
    std::signal(SIGINT, signal_handler);
    rclcpp::init(argc, argv);
    Arduino::serinit();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    
    auto node = std::make_shared<Socket>();
    auto MavrosSubscribernode = std::make_shared<MavrosSubscriber>();
    auto Subscribernode = std::make_shared<Subscriber>(node);
    auto Tasknode = std::make_shared<Task>(MavrosSubscribernode);
    auto PidReadnode = std::make_shared<PidRead>();
    
    executor.add_node(node);
    executor.add_node(MavrosSubscribernode);
    executor.add_node(Subscribernode);
    executor.add_node(Tasknode);
    executor.add_node(PidReadnode);
    
    executor.spin();
    rclcpp::shutdown(); 
    return 0;
}