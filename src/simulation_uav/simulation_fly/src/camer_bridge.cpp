#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include <gz/msgs/clock.pb.h>
#include <cstring>
#include <csignal>
#include <memory>
#include <string>

/**
 * gz-sim → ROS2 桥接节点（适配 x500_mid360 模型）。
 *
 * 注意：x500_mid360 的自定义传感器都显式写了 <topic>，gz-sim 会把这些
 * topic 发布为【全局 topic】（不带 /world/... 作用域）。实测 `gz topic -l`：
 *   - 下视 gemini2 相机: /gemini2/rgb/image        (gz.msgs.Image)
 *   - 前视相机:          /front_cam/rgb/image      (gz.msgs.Image)
 *   - MID360 雷达点云:   /mid360/scan/points       (gz.msgs.PointCloudPacked)
 *   - MID360 IMU:        /mid360/imu               (gz.msgs.IMU)
 *
 * 只有 x500_base 里【没写 <topic>】的传感器才走
 * /world/<world>/model/<model>/link/<link>/sensor/<sensor>/<type> 作用域。
 *
 * 下游消费（FAST-LIO mid360.yaml）：
 *   - 点云 PointCloud2 -> /drone_0_/lidar/points
 *   - IMU              -> lidar/imu/data
 */
class GzToRos2Bridge : public rclcpp::Node
{
public:
    GzToRos2Bridge()
        : Node("gz_to_ros2_bridge")
    {
        // ---------------- gz 全局 topic ----------------
        this->declare_parameter<std::string>("gz_rgb_topic", "/gemini2/rgb/image");
        this->declare_parameter<std::string>("gz_front_rgb_topic", "/front_cam/rgb/image");
        this->declare_parameter<std::string>("gz_lidar_points_topic", "/mid360/scan/points");
        this->declare_parameter<std::string>("gz_lidar_imu_topic", "/mid360/imu");
        gz_rgb_topic_         = this->get_parameter("gz_rgb_topic").as_string();
        gz_front_rgb_topic_   = this->get_parameter("gz_front_rgb_topic").as_string();
        gz_lidar_points_topic_= this->get_parameter("gz_lidar_points_topic").as_string();
        gz_lidar_imu_topic_   = this->get_parameter("gz_lidar_imu_topic").as_string();

        // ---------------- ROS2 输出 topic ----------------
        this->declare_parameter<std::string>("ros_rgb_topic", "/drone_0_/camera/image_raw");
        this->declare_parameter<std::string>("ros_front_rgb_topic", "/drone_0_/front_cam/image_raw");
        this->declare_parameter<std::string>("ros_lidar_points_topic", "/drone_0_/lidar/points");
        this->declare_parameter<std::string>("lidar_imu_topic", "lidar/imu/data");
        ros_rgb_topic_          = this->get_parameter("ros_rgb_topic").as_string();
        ros_front_rgb_topic_    = this->get_parameter("ros_front_rgb_topic").as_string();
        ros_lidar_points_topic_ = this->get_parameter("ros_lidar_points_topic").as_string();
        lidar_imu_topic_        = this->get_parameter("lidar_imu_topic").as_string();

        // ---------------- 发布器 ----------------
        rgb_pub_          = this->create_publisher<sensor_msgs::msg::Image>(ros_rgb_topic_, 100);
        front_rgb_pub_    = this->create_publisher<sensor_msgs::msg::Image>(ros_front_rgb_topic_, 100);
        lidar_imu_pub_    = this->create_publisher<sensor_msgs::msg::Imu>(lidar_imu_topic_, 100);
        lidar_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(ros_lidar_points_topic_, 100);
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // ---------------- CameraInfo 发布器 ----------------
        auto info_qos = rclcpp::QoS(rclcpp::KeepLast(10));
        info_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        rgb_info_pub_       = this->create_publisher<sensor_msgs::msg::CameraInfo>(ros_rgb_topic_ + "_camera_info", info_qos);
        front_rgb_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(ros_front_rgb_topic_ + "_camera_info", info_qos);

        initializeCameraInfo();

        // ---------------- Gazebo 订阅 ----------------
        if (!gz_node_.Subscribe(gz_rgb_topic_, &GzToRos2Bridge::rgbCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅 RGB 失败: %s", gz_rgb_topic_.c_str());
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅 RGB: %s", gz_rgb_topic_.c_str());

        if (!gz_node_.Subscribe(gz_front_rgb_topic_, &GzToRos2Bridge::frontRgbCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅前视 RGB 失败: %s", gz_front_rgb_topic_.c_str());
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅前视 RGB: %s", gz_front_rgb_topic_.c_str());

        if (!gz_node_.Subscribe("/clock", &GzToRos2Bridge::gzClockCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅 Gazebo Clock 失败: /clock");
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅 Gazebo Clock: /clock");

        if (!gz_node_.Subscribe(gz_lidar_points_topic_, &GzToRos2Bridge::lidarPointsCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅雷达点云失败: %s", gz_lidar_points_topic_.c_str());
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅雷达点云: %s", gz_lidar_points_topic_.c_str());

        if (!gz_node_.Subscribe(gz_lidar_imu_topic_, &GzToRos2Bridge::gzLidarImuCallback, this))
            RCLCPP_ERROR(this->get_logger(), "订阅雷达 IMU 失败: %s", gz_lidar_imu_topic_.c_str());
        else
            RCLCPP_INFO(this->get_logger(), "成功订阅雷达 IMU: %s", gz_lidar_imu_topic_.c_str());

        RCLCPP_INFO(this->get_logger(), "gz-sim → ROS2 桥启动");
        RCLCPP_INFO(this->get_logger(), "下视 RGB -> %s", ros_rgb_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "前视 RGB -> %s", ros_front_rgb_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "雷达点云 -> %s", ros_lidar_points_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "雷达 IMU -> %s", lidar_imu_topic_.c_str());

        std::signal(SIGINT, [](int){ rclcpp::shutdown(); });
    }

private:
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

    // ---------------- 雷达点云回调（gz PointCloudPacked -> ROS2 PointCloud2） ----------------
    void lidarPointsCallback(const gz::msgs::PointCloudPacked &gz_msg)
    {
        sensor_msgs::msg::PointCloud2 ros_msg;
        auto s = gz_msg.header().stamp();
        ros_msg.header.stamp.sec = s.sec();
        ros_msg.header.stamp.nanosec = s.nsec();
        ros_msg.header.frame_id = "lidar_sensor_link";

        ros_msg.height = 1;
        ros_msg.width = gz_msg.data().size() / gz_msg.point_step();
        ros_msg.is_bigendian = false;
        ros_msg.point_step = gz_msg.point_step();
        ros_msg.row_step = ros_msg.width * ros_msg.point_step;
        ros_msg.is_dense = true;

        ros_msg.fields.clear();
        for (const auto &field : gz_msg.field()) {
            sensor_msgs::msg::PointField pf;
            pf.name = field.name();
            pf.offset = field.offset();
            switch (field.datatype()) {
                case gz::msgs::PointCloudPacked::Field::INT8:    pf.datatype = sensor_msgs::msg::PointField::INT8; break;
                case gz::msgs::PointCloudPacked::Field::UINT8:   pf.datatype = sensor_msgs::msg::PointField::UINT8; break;
                case gz::msgs::PointCloudPacked::Field::INT16:   pf.datatype = sensor_msgs::msg::PointField::INT16; break;
                case gz::msgs::PointCloudPacked::Field::UINT16:  pf.datatype = sensor_msgs::msg::PointField::UINT16; break;
                case gz::msgs::PointCloudPacked::Field::INT32:   pf.datatype = sensor_msgs::msg::PointField::INT32; break;
                case gz::msgs::PointCloudPacked::Field::UINT32:  pf.datatype = sensor_msgs::msg::PointField::UINT32; break;
                case gz::msgs::PointCloudPacked::Field::FLOAT32: pf.datatype = sensor_msgs::msg::PointField::FLOAT32; break;
                case gz::msgs::PointCloudPacked::Field::FLOAT64: pf.datatype = sensor_msgs::msg::PointField::FLOAT64; break;
                default: pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
            }
            pf.count = 1;
            ros_msg.fields.push_back(pf);
        }

        ros_msg.data.resize(gz_msg.data().size());
        std::memcpy(ros_msg.data.data(), gz_msg.data().data(), gz_msg.data().size());

        lidar_points_pub_->publish(ros_msg);
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

    std::string gz_rgb_topic_, gz_front_rgb_topic_;
    std::string gz_lidar_points_topic_, gz_lidar_imu_topic_;
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
