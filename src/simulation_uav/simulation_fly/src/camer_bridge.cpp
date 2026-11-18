#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include <gz/msgs/clock.pb.h>
#include <cmath>
#include <limits>
#include <cstring>
#include <csignal>
#include <memory>
#include <string>

/**
 * gz-sim → ROS2 桥接节点（适配 x500_mid360 模型）。
 *
 * gz-sim 传感器话题带作用域，完整格式为：
 *   /world/<world_name>/model/<model_name>/link/<link>/sensor/<sensor>/<topic>
 *
 * 对应 x500_mid360 模型的传感器：
 *   - 下视 gemini2 相机: link=camera_link, sensor=gemini2_rgb, topic=gemini2/rgb/image
 *   - 前视相机:          link=front_camera_link, sensor=front_camera_rgb, topic=front_cam/rgb/image
 *   - MID360 雷达:       link=lidar_sensor_link, sensor=mid360_lidar, topic=mid360/scan
 *   - MID360 IMU:        link=lidar_sensor_link, sensor=mid360_imu,   topic=mid360/imu
 *
 * 下游消费（FAST-LIO mid360.yaml）：
 *   - 点云 PointCloud2 -> /drone_0_/lidar/points
 *   - IMU              -> lidar/imu/data
 *
 * 注意：若 gz-sim 版本对话题作用域处理不同，用 `gz topic -l` 查看实际话题名后，
 * 改 world_name/model_name 即可（link/sensor/topic 已按模型写死）。
 */
class GzToRos2Bridge : public rclcpp::Node
{
public:
    GzToRos2Bridge()
        : Node("gz_to_ros2_bridge")
    {
        // ---------------- world / model 名 ----------------
        this->declare_parameter<std::string>("world_name", "custom_world");
        this->declare_parameter<std::string>("model_name", "x500_mid360");
        world_name_ = this->get_parameter("world_name").as_string();
        model_name_ = this->get_parameter("model_name").as_string();

        // ---------------- ROS2 输出话题 ----------------
        this->declare_parameter<std::string>("ros_rgb_topic", "/drone_0_/camera/image_raw");
        this->declare_parameter<std::string>("ros_front_rgb_topic", "/drone_0_/front_cam/image_raw");
        this->declare_parameter<std::string>("ros_lidar_points_topic", "/drone_0_/lidar/points");
        this->declare_parameter<std::string>("lidar_imu_topic", "lidar/imu/data");

        ros_rgb_topic_          = this->get_parameter("ros_rgb_topic").as_string();
        ros_front_rgb_topic_    = this->get_parameter("ros_front_rgb_topic").as_string();
        ros_lidar_points_topic_ = this->get_parameter("ros_lidar_points_topic").as_string();
        lidar_imu_topic_        = this->get_parameter("lidar_imu_topic").as_string();

        // ---------------- 发布器 ----------------
        rgb_pub_         = this->create_publisher<sensor_msgs::msg::Image>(ros_rgb_topic_, 100);
        front_rgb_pub_   = this->create_publisher<sensor_msgs::msg::Image>(ros_front_rgb_topic_, 100);
        lidar_imu_pub_   = this->create_publisher<sensor_msgs::msg::Imu>(lidar_imu_topic_, 100);
        lidar_points_pub_= this->create_publisher<sensor_msgs::msg::PointCloud2>(ros_lidar_points_topic_, 100);
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // ---------------- CameraInfo 发布器 ----------------
        auto info_qos = rclcpp::QoS(rclcpp::KeepLast(10));
        info_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        rgb_info_pub_       = this->create_publisher<sensor_msgs::msg::CameraInfo>(ros_rgb_topic_ + "_camera_info", info_qos);
        front_rgb_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(ros_front_rgb_topic_ + "_camera_info", info_qos);

        // 相机内参：1280x720, hfov=1.5016 -> fx≈685.9, cx=640, cy=360（与模型一致）
        initializeCameraInfo();

        // ---------------- 构造 gz-sim 作用域话题 ----------------
        const std::string gz_rgb_topic       = scopedTopic("camera_link",       "gemini2_rgb",     "gemini2/rgb/image");
        const std::string gz_front_rgb_topic = scopedTopic("front_camera_link", "front_camera_rgb","front_cam/rgb/image");
        const std::string gz_lidar_scan      = scopedTopic("lidar_sensor_link", "mid360_lidar",    "mid360/scan");
        const std::string gz_lidar_imu       = scopedTopic("lidar_sensor_link", "mid360_imu",      "mid360/imu");

        // ---------------- Gazebo 订阅 ----------------
        if (!gz_node_.Subscribe(gz_rgb_topic, &GzToRos2Bridge::rgbCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅 RGB 失败: %s", gz_rgb_topic.c_str());
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅 RGB: %s", gz_rgb_topic.c_str());

        if (!gz_node_.Subscribe(gz_front_rgb_topic, &GzToRos2Bridge::frontRgbCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅前视 RGB 失败: %s", gz_front_rgb_topic.c_str());
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅前视 RGB: %s", gz_front_rgb_topic.c_str());

        if (!gz_node_.Subscribe("/clock", &GzToRos2Bridge::gzClockCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅 Gazebo Clock 失败: /clock");
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅 Gazebo Clock: /clock");

        if (!gz_node_.Subscribe(gz_lidar_scan, &GzToRos2Bridge::lidarScanCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅激光雷达扫描失败: %s", gz_lidar_scan.c_str());
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅激光雷达扫描: %s", gz_lidar_scan.c_str());

        if (!gz_node_.Subscribe(gz_lidar_imu, &GzToRos2Bridge::gzLidarImuCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅激光雷达 IMU 失败: %s", gz_lidar_imu.c_str());
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅激光雷达 IMU: %s", gz_lidar_imu.c_str());

        RCLCPP_INFO(this->get_logger(), "gz-sim → ROS2 桥启动");
        RCLCPP_INFO(this->get_logger(), "下视 RGB -> %s", ros_rgb_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "前视 RGB -> %s", ros_front_rgb_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "雷达点云 -> %s", ros_lidar_points_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "雷达 IMU -> %s", lidar_imu_topic_.c_str());

        std::signal(SIGINT, [](int){ rclcpp::shutdown(); });
    }

private:
    // 构造 gz-sim 作用域话题
    std::string scopedTopic(const std::string& link, const std::string& sensor, const std::string& topic)
    {
        return "/world/" + world_name_ + "/model/" + model_name_ +
               "/link/" + link + "/sensor/" + sensor + "/" + topic;
    }

    void initializeCameraInfo()
    {
        // 1280x720, hfov=1.5016 -> fx≈685.9
        const double fx = 685.9, fy = 685.9, cx = 640.0, cy = 360.0;
        auto fill = [&](sensor_msgs::msg::CameraInfo& ci, const std::string& frame_id) {
            ci.header.frame_id = frame_id;
            ci.height = 720;
            ci.width  = 1280;
            ci.distortion_model = "plumb_bob";
            ci.d = {0.0, 0.0, 0.0, 0.0, 0.0};
            ci.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
            ci.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
            ci.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
        };
        fill(rgb_camera_info_, "camera_link");
        fill(front_rgb_camera_info_, "front_camera_link");
    }

    // ---------------- RGB 回调 ----------------
    void rgbCallback(const gz::msgs::Image &gz_msg)
    {
        auto ros_msg = convertToRosImage(gz_msg, "camera_link");
        rgb_camera_info_.header.stamp = ros_msg.header.stamp;
        rgb_pub_->publish(ros_msg);
        rgb_info_pub_->publish(rgb_camera_info_);
    }

    void frontRgbCallback(const gz::msgs::Image &gz_msg)
    {
        auto ros_msg = convertToRosImage(gz_msg, "front_camera_link");
        front_rgb_camera_info_.header.stamp = ros_msg.header.stamp;
        front_rgb_pub_->publish(ros_msg);
        front_rgb_info_pub_->publish(front_rgb_camera_info_);
    }

    void gzClockCallback(const gz::msgs::Clock &_msg)
    {
        rosgraph_msgs::msg::Clock ros_clock_msg;
        ros_clock_msg.clock.sec = _msg.sim().sec();
        ros_clock_msg.clock.nanosec = _msg.sim().nsec();
        clock_pub_->publish(ros_clock_msg);
    }

    // ---------------- 雷达 IMU 回调 ----------------
    void gzLidarImuCallback(const gz::msgs::IMU &gz_msg)
    {
        sensor_msgs::msg::Imu ros_msg;
        auto s = gz_msg.header().stamp();
        ros_msg.header.stamp.sec = s.sec();
        ros_msg.header.stamp.nanosec = s.nsec();
        ros_msg.header.frame_id = "lidar_sensor_link";

        ros_msg.linear_acceleration.x = gz_msg.linear_acceleration().x();
        ros_msg.linear_acceleration.y = gz_msg.linear_acceleration().y();
        ros_msg.linear_acceleration.z = gz_msg.linear_acceleration().z();

        ros_msg.angular_velocity.x = gz_msg.angular_velocity().x();
        ros_msg.angular_velocity.y = gz_msg.angular_velocity().y();
        ros_msg.angular_velocity.z = gz_msg.angular_velocity().z();

        ros_msg.orientation.x = gz_msg.orientation().x();
        ros_msg.orientation.y = gz_msg.orientation().y();
        ros_msg.orientation.z = gz_msg.orientation().z();
        ros_msg.orientation.w = gz_msg.orientation().w();

        for (int i = 0; i < 9; ++i) {
            ros_msg.orientation_covariance[i] = 0.0;
            ros_msg.angular_velocity_covariance[i] = 0.0;
            ros_msg.linear_acceleration_covariance[i] = 0.0;
        }
        lidar_imu_pub_->publish(ros_msg);
    }

    // ---------------- 激光雷达扫描 -> PointCloud2 ----------------
    // gz gpu_lidar 输出的是三维 LaserScan（水平 360 x 垂直多线），
    // FAST-LIO 需要 PointCloud2，这里把 (水平角, 垂直角, 距离) 转成三维点云。
    void lidarScanCallback(const gz::msgs::LaserScan &gz_msg)
    {
        const int h_count = static_cast<int>(gz_msg.count());        // 水平采样数
        const int v_count = static_cast<int>(gz_msg.vertical_count()); // 垂直采样数
        const int total = h_count * v_count;
        if (total <= 0 || gz_msg.ranges_size() < total) return;

        sensor_msgs::msg::PointCloud2 cloud;
        auto s = gz_msg.header().stamp();
        cloud.header.stamp.sec = s.sec();
        cloud.header.stamp.nanosec = s.nsec();
        cloud.header.frame_id = "lidar_sensor_link";
        cloud.height = 1;
        cloud.width = total;
        cloud.is_dense = false;
        cloud.is_bigendian = false;

        cloud.fields.resize(4);
        cloud.fields[0].name = "x";         cloud.fields[0].offset = 0;  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields[0].count = 1;
        cloud.fields[1].name = "y";         cloud.fields[1].offset = 4;  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields[1].count = 1;
        cloud.fields[2].name = "z";         cloud.fields[2].offset = 8;  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields[2].count = 1;
        cloud.fields[3].name = "intensity"; cloud.fields[3].offset = 12; cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields[3].count = 1;
        cloud.point_step = 16;
        cloud.row_step = 16 * total;
        cloud.data.resize(static_cast<size_t>(16) * total);

        float* p = reinterpret_cast<float*>(cloud.data.data());
        for (int j = 0; j < v_count; ++j) {
            const double va = gz_msg.vertical_angle_min() + j * gz_msg.vertical_angle_step();
            const double cos_va = std::cos(va), sin_va = std::sin(va);
            for (int i = 0; i < h_count; ++i) {
                const double ha = gz_msg.angle_min() + i * gz_msg.angle_step();
                const double cos_ha = std::cos(ha), sin_ha = std::sin(ha);
                const double r = gz_msg.ranges(j * h_count + i);
                float* pt = p + (j * h_count + i) * 4;
                if (r >= gz_msg.range_min() && r <= gz_msg.range_max() && std::isfinite(r)) {
                    pt[0] = static_cast<float>(r * cos_va * cos_ha);
                    pt[1] = static_cast<float>(r * cos_va * sin_ha);
                    pt[2] = static_cast<float>(r * sin_va);
                    pt[3] = (gz_msg.intensities_size() > j * h_count + i)
                                ? static_cast<float>(gz_msg.intensities(j * h_count + i)) : 0.0f;
                } else {
                    pt[0] = std::numeric_limits<float>::quiet_NaN();
                    pt[1] = std::numeric_limits<float>::quiet_NaN();
                    pt[2] = std::numeric_limits<float>::quiet_NaN();
                    pt[3] = 0.0f;
                }
            }
        }
        lidar_points_pub_->publish(cloud);
    }

    // ---------------- Gazebo Image → ROS Image ----------------
    sensor_msgs::msg::Image convertToRosImage(const gz::msgs::Image &gz_msg,
                                              const std::string &frame_id)
    {
        sensor_msgs::msg::Image ros_msg;
        if (gz_msg.has_header()) {
            auto s = gz_msg.header().stamp();
            ros_msg.header.stamp.sec = s.sec();
            ros_msg.header.stamp.nanosec = s.nsec();
        } else {
            ros_msg.header.stamp = this->now();
        }
        ros_msg.header.frame_id = frame_id;
        ros_msg.height = gz_msg.height();
        ros_msg.width = gz_msg.width();
        ros_msg.is_bigendian = false;

        switch (gz_msg.pixel_format_type()) {
            case gz::msgs::PixelFormatType::RGB_INT8:  ros_msg.encoding = "rgb8";    ros_msg.step = ros_msg.width*3; break;
            case gz::msgs::PixelFormatType::RGBA_INT8: ros_msg.encoding = "rgba8";   ros_msg.step = ros_msg.width*4; break;
            case gz::msgs::PixelFormatType::L_INT8:    ros_msg.encoding = "mono8";   ros_msg.step = ros_msg.width;   break;
            case gz::msgs::PixelFormatType::L_INT16:   ros_msg.encoding = "mono16";  ros_msg.step = ros_msg.width*2; break;
            case gz::msgs::PixelFormatType::BGR_INT8:  ros_msg.encoding = "bgr8";    ros_msg.step = ros_msg.width*3; break;
            case gz::msgs::PixelFormatType::R_FLOAT32: ros_msg.encoding = "32FC1";   ros_msg.step = ros_msg.width*4; break;
            default: ros_msg.encoding = "rgb8"; ros_msg.step = ros_msg.width*3; break;
        }
        ros_msg.data.resize(gz_msg.data().size());
        std::memcpy(ros_msg.data.data(), gz_msg.data().data(), gz_msg.data().size());
        return ros_msg;
    }

private:
    gz::transport::Node gz_node_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_, front_rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr lidar_imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_pub_, front_rgb_info_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

    std::string world_name_, model_name_;
    std::string ros_rgb_topic_, ros_front_rgb_topic_;
    std::string ros_lidar_points_topic_;
    std::string lidar_imu_topic_;

    sensor_msgs::msg::CameraInfo rgb_camera_info_, front_rgb_camera_info_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GzToRos2Bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
