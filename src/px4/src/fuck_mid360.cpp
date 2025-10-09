/**
 * @file improved_mid360.cpp
 * @brief 改进的ROS2 Mid-360 lidar数据处理节点
 * @version 2.1
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "nav_msgs/msg/odometry.hpp"
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <mutex>
#include <memory>

#include "lib/Arduino.h"

using namespace std::chrono;

class ImprovedMid360Node : public rclcpp::Node
{
public:
    ImprovedMid360Node() : Node("improved_mid360")
    {
        // 参数声明
        declare_parameter("publish_rate", 30.0);  // 降低到50Hz
        declare_parameter("height_fusion_enable", true);
        declare_parameter("max_data_age_ms", 100.0);  // 最大数据年龄100ms
        declare_parameter("height_filter_alpha", 0.8);  // 低通滤波系数
        
        publish_rate_ = get_parameter("publish_rate").as_double();
   
        max_data_age_ms_ = get_parameter("max_data_age_ms").as_double();
        height_filter_alpha_ = get_parameter("height_filter_alpha").as_double();
        
        // Subscribers
        lidar_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, 
            std::bind(&ImprovedMid360Node::lidar_callback, this, std::placeholders::_1));
            
      
            
        // Publishers
        lidar_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/vision_pose/pose", 10);

        pose_pub_x = create_publisher<std_msgs::msg::Float64>("/pose_mid_x", 10);
        pose_pub_y = create_publisher<std_msgs::msg::Float64>("/pose_mid_y", 10);
        pose_pub_z = create_publisher<std_msgs::msg::Float64>("/pose_mid_z", 10);
        pose_pub_yaw = create_publisher<std_msgs::msg::Float64>("pose_mid_yaw", 10);

        // Timer - 使用可配置频率
        auto timer_period = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_));
        timer_ = create_wall_timer(timer_period, 
            std::bind(&ImprovedMid360Node::timer_callback, this));
            
        RCLCPP_INFO(get_logger(), "ImprovedMid360Node initialized at %.1f Hz", publish_rate_);
    }

private:
    // ROS2 communication
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lidar_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pose_pub_x, pose_pub_y, pose_pub_z, pose_pub_yaw;
    rclcpp::TimerBase::SharedPtr timer_;

    // 参数
    double publish_rate_;
    double max_data_age_ms_;
    double height_filter_alpha_;

    // Data storage with timestamps
    struct TimestampedOdom {
        nav_msgs::msg::Odometry data;
        rclcpp::Time timestamp;
        bool valid = false;
    };
    
    TimestampedOdom latest_odom_;
    std::mutex data_mutex_;

    // Processing state
    bool first_measurement_ = true;
    double filtered_height_ = 0.0;
    double last_valid_height_ = 0.0;
    
    // 数据质量统计
    int valid_measurements_ = 0;
    int total_measurements_ = 0;

    /**
     * @brief Lidar数据回调
     */
    void lidar_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_odom_.data = *msg;
        latest_odom_.timestamp = this->now();
        latest_odom_.valid = validate_odom_data(*msg);
        
        if (!latest_odom_.valid) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                "Invalid lidar odometry data received");
                 Arduino::loud();
        }
    }
    


    /**
     * @brief 验证里程计数据有效性
     */
    bool validate_odom_data(const nav_msgs::msg::Odometry& odom)
    {
        // 检查位置数据
        if (!std::isfinite(odom.pose.pose.position.x) ||
            !std::isfinite(odom.pose.pose.position.y) ||
            !std::isfinite(odom.pose.pose.position.z)) {
            return false;
        }
        
        // 检查四元数
        auto& q = odom.pose.pose.orientation;
        if (!std::isfinite(q.x) || !std::isfinite(q.w) ||
            !std::isfinite(q.y) || !std::isfinite(q.z)) {
            return false;
        }
        
        // 检查四元数模长
        double norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
        if (abs(norm - 1.0) > 0.1) {  // 四元数应该是单位长度
            return false;
        }
        
        return true;
    }
    

    
    /**
     * @brief 检查数据是否过时
     */
    bool is_data_fresh(const rclcpp::Time& data_time)
    {
        auto age = (this->now() - data_time).seconds() * 1000.0;  // ms
        return age <= max_data_age_ms_;
    }

    /**
     * @brief 高度数据滤波
     */
    double filter_height(double lidar_height)
    {
        if (first_measurement_) {
            filtered_height_ = lidar_height;
            first_measurement_ = false;
            return lidar_height;
        }
        
        // 低通滤波
        filtered_height_ = height_filter_alpha_ * filtered_height_ + 
                          (1.0 - height_filter_alpha_) * lidar_height;
        
        return filtered_height_;
    }

    /**
     * @brief Timer callback for processing and publishing data
     */
    void timer_callback()
    {
        TimestampedOdom odom;
        
        // Copy shared data under lock
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            odom = latest_odom_;
        }
        
        total_measurements_++;
        
        // 检查数据有效性和新鲜度
        bool odom_ok = odom.valid && is_data_fresh(odom.timestamp);
        
        if (!odom_ok) {
             Arduino::loud();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "No valid/fresh odometry data available");
                
                 

            return;
        }
        
        valid_measurements_++;
        
        // Calculate yaw from orientation
        tf2::Quaternion q;
        tf2::fromMsg(odom.data.pose.pose.orientation, q);
        double yaw = tf2::getYaw(q);
        
        // 高度滤波处理
        double final_height = filter_height(odom.data.pose.pose.position.z);
        
        // Prepare output message
        geometry_msgs::msg::PoseStamped send_data;
        send_data.header.stamp = this->now();
 
      
        send_data.pose.position.x = odom.data.pose.pose.position.x;
        send_data.pose.position.y = odom.data.pose.pose.position.y;
        send_data.pose.position.z = final_height;
        send_data.pose.orientation = odom.data.pose.pose.orientation;
  
        // Publish pose
        lidar_pub_->publish(send_data);

        // Publish debug info
        publish_debug_info(send_data, yaw);
        
        // 定期输出统计信息
        if (total_measurements_ % (int(publish_rate_) * 5) == 0) {  // 每5秒
            double success_rate = 100.0 * valid_measurements_ / total_measurements_;
            RCLCPP_INFO(get_logger(), 
                "Data quality: %.1f%% (%d/%d)", 
                success_rate, valid_measurements_, total_measurements_);
        }
    }
    
    /**
     * @brief 发布调试信息
     */
    void publish_debug_info(const geometry_msgs::msg::PoseStamped& pose, double yaw)
    {
        std_msgs::msg::Float64 msg_x, msg_y, msg_z, msg_yaw;
        msg_x.data = pose.pose.position.x;
        msg_y.data = pose.pose.position.y;
        msg_z.data = pose.pose.position.z;
        msg_yaw.data = yaw;
        
        pose_pub_x->publish(msg_x);
        pose_pub_y->publish(msg_y);
        pose_pub_z->publish(msg_z);
        pose_pub_yaw->publish(msg_yaw);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImprovedMid360Node>();
    Arduino::serinit();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}