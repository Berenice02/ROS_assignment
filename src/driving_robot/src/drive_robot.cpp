#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_msgs/msg/closest_obstacle.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include "driving_robot/scan_sectors.hpp"

using std::placeholders::_1;

class DriveRobot : public rclcpp::Node
{
public:
    DriveRobot() : Node("drive_robot")
    {
        // User choice implemented through node parameters
        this->declare_parameter("linear_speed", 0.0);
        this->declare_parameter("angular_speed", 0.0);

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&DriveRobot::scan_callback, this, _1));
        obstacle_subscription_ = this->create_subscription<custom_msgs::msg::ClosestObstacle>("/closest_obstacle", 10, std::bind(&DriveRobot::obstacle_callback, this, _1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&DriveRobot::timer_callback, this));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_ = msg;
    }

    void obstacle_callback(const custom_msgs::msg::ClosestObstacle::SharedPtr msg)
    {
        // the safety distance comes from obstacle_monitor
        safety_distance_ = msg->threshold;
    }

    double min_distance(const geometry_msgs::msg::Twist &velocity)
    {
        // select the sector according to linear/angular velocities
        if (velocity.linear.x > 0.0)
        {
            if (velocity.angular.z > 0.0)
            {
                return min_scan_in_sector(scan_, FRONT_LEFT);
            }
            if (velocity.angular.z < 0.0)
            {
                return min_scan_in_sector(scan_, FRONT_RIGHT);
            }
            return std::min(min_scan_in_sector(scan_, FRONT_RIGHT), min_scan_in_sector(scan_, FRONT_LEFT));
        }

        if (velocity.linear.x < 0.0)
        {
            if (velocity.angular.z > 0.0)
            {
                return min_scan_in_sector(scan_, REAR_RIGHT);
            }
            if (velocity.angular.z < 0.0)
            {
                return min_scan_in_sector(scan_, REAR_LEFT);
            }
            // the rear sector is split between the two ends of ranges
            return std::min(min_scan_in_sector(scan_, REAR_RIGHT), min_scan_in_sector(scan_, REAR_LEFT));
        }

        return std::numeric_limits<double>::infinity();
    }

    void timer_callback()
    {
        if (!scan_ || !std::isfinite(safety_distance_))
        {
            return;
        }

        if (recovering_)
        {
            if (min_distance(blocked_) < safety_distance_)
            {
                message.linear.x = -blocked_.linear.x;
                message.angular.z = -blocked_.angular.z;
            }
            else
            {
                message.linear.x = 0.0;
                message.angular.z = 0.0;
                recovering_ = false;
                this->set_parameter(rclcpp::Parameter("linear_speed", 0.0));
                this->set_parameter(rclcpp::Parameter("angular_speed", 0.0));
                RCLCPP_WARN(this->get_logger(), "Safe area reached, stop");
            }
        }
        else
        {
            geometry_msgs::msg::Twist user_input;
            user_input.linear.x = this->get_parameter("linear_speed").as_double();
            user_input.angular.z = this->get_parameter("angular_speed").as_double();

            if (min_distance(user_input) < safety_distance_)
            {
                recovering_ = true;
                blocked_ = user_input;
                message.linear.x = -user_input.linear.x;
                message.angular.z = -user_input.angular.z;
                RCLCPP_WARN(this->get_logger(), "Too close to an obstacle, moving back");
            }
            else
            {
                message = user_input;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Publishing: '%f', '%f'", message.linear.x, message.angular.z);
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;

    rclcpp::Subscription<custom_msgs::msg::ClosestObstacle>::SharedPtr obstacle_subscription_;

    geometry_msgs::msg::Twist message;
    geometry_msgs::msg::Twist blocked_;

    bool recovering_ = false;

    double safety_distance_ = std::numeric_limits<double>::infinity();
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveRobot>());
    rclcpp::shutdown();
    return 0;
}
