#ifndef FIXED_POINT_HPP
#define FIXED_POINT_HPP

#include <geometry_msgs/msg/point.hpp>
#include "pidcontroller.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
//#include "new_ac_link/msg/target_object.hpp"  // 注意命名风格
#include <ac_link/bound.hpp>  // 将 .cpp 改为 .hpp

namespace fixed_point
{
    // Size模板类定义
    template <typename T>
    class Size_
    {
    public:
        Size_() : width(0), height(0) {}
        Size_(T w, T h) : width(w), height(h) {}

        T width;
        T height;
    };

    // Point模板类定义
    template <typename T>
    class Point_
    {
    public:
        Point_() : x(0), y(0) {}
        Point_(T x, T y) : x(x), y(y) {}

        T x;
        T y;
    };

    typedef Size_<int> Size2i;
    typedef Size2i Size;

    typedef Point_<double> Point2d;
    typedef Point2d Point;
}

class FixedPoint
{
public:
    FixedPoint(rclcpp::Node::SharedPtr node, const std::string &topic, fixed_point::Point frame_center, const pid_controller::PidParams &params_x, const pid_controller::PidParams &params_y);

    auto calcXYOutput(fixed_point::Point offset) -> fixed_point::Point;
    auto calcXYOutput() -> fixed_point::Point;
    auto getBoundedOutput(fixed_point::Point offset) -> fixed_point::Point;
    auto getBoundedOutput() -> fixed_point::Point;
    rclcpp::Time getTimeNow();
    rclcpp::Logger get_logger();

    void clear();

    ~FixedPoint() {};

protected:
    void subPointCallback(const std::shared_ptr<const geometry_msgs::msg::Point> msg);
    fixed_point::Point last_output_;
    rclcpp::Node::SharedPtr node_;                                         // 节点智能指针
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point_sub_; // 订阅器
    geometry_msgs::msg::Point point_;                                      // 目标点数据

public:
    fixed_point::Size frame_size_;
    fixed_point::Point frame_center_;
    fixed_point::Point output_bound_;

    PIDController pid_controller_x; // X轴PID控制器
    PIDController pid_controller_y; // Y轴PID控制器
};

#endif // FIXED_POINT_HPP