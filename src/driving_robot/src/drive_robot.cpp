#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

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
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&DriveRobot::timer_callback, this));
    }

private:
    struct Sector
    {
        int first;
        int last;
    };

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_ = msg;
    }

    double min_scan_in_sector(Sector sector)
    {
        double minimum = std::numeric_limits<double>::infinity();

        // find the minimum over the beams of the sector
        for (int i = sector.first; i <= sector.last; i++)
        {
            double current = scan_->ranges[i];
            if (std::isfinite(current) && current < minimum)
            {
                minimum = current;
            }
        }
        return minimum;
    }

    double min_distance(const geometry_msgs::msg::Twist &velocity)
    {
        // select the sector according to linear/angular velocities
        if (velocity.linear.x > 0.0)
        {
            if (velocity.angular.z > 0.0)
            {
                return min_scan_in_sector(front_left_);
            }
            if (velocity.angular.z < 0.0)
            {
                return min_scan_in_sector(front_right_);
            }
            return std::min(min_scan_in_sector(front_right_), min_scan_in_sector(front_left_));
        }

        if (velocity.linear.x < 0.0)
        {
            if (velocity.angular.z > 0.0)
            {
                return min_scan_in_sector(rear_right_);
            }
            if (velocity.angular.z < 0.0)
            {
                return min_scan_in_sector(rear_left_);
            }
            // the rear sector is split between the two ends of ranges
            return std::min(min_scan_in_sector(rear_right_), min_scan_in_sector(rear_left_));
        }

        return std::numeric_limits<double>::infinity();
    }

    void timer_callback()
    {
        if (!scan_)
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

    geometry_msgs::msg::Twist message;
    geometry_msgs::msg::Twist blocked_;

    bool recovering_ = false;

    double safety_distance_ = 0.4;

    // the lidar publishes 720 beams over 360 degrees, starting from the rear
    Sector rear_right_ = {0, 180};
    Sector front_right_ = {180, 360};
    Sector front_left_ = {360, 540};
    Sector rear_left_ = {540, 719};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveRobot>());
    rclcpp::shutdown();
    return 0;
}
