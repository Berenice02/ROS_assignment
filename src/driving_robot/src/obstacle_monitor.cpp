#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/closest_obstacle.hpp"
#include "custom_msgs/srv/safety_distance.hpp"
#include "driving_robot/scan_sectors.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <limits>

using std::placeholders::_1;
using std::placeholders::_2;

class ObstacleMonitor : public rclcpp::Node
{
public:
    ObstacleMonitor() : Node("obstacle_monitor")
    {
        publisher_ = this->create_publisher<custom_msgs::msg::ClosestObstacle>("/closest_obstacle", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&ObstacleMonitor::scan_callback, this, _1));
        service_ = this->create_service<custom_msgs::srv::SafetyDistance>("set_safety_distance", std::bind(&ObstacleMonitor::safety_callback, this, _1, _2));
    }

private:
    void safety_callback(const std::shared_ptr<custom_msgs::srv::SafetyDistance::Request> request,
                         std::shared_ptr<custom_msgs::srv::SafetyDistance::Response> response)
    {
        if (request->distance <= 0.0)
        {
            response->success = false;
            response->message = "the safety distance must be positive";
            RCLCPP_WARN(this->get_logger(), "Refused safety distance: '%f'", request->distance);
            return;
        }

        safety_distance_ = request->distance;
        response->success = true;
        response->message = "safety distance set";
        RCLCPP_INFO(this->get_logger(), "Safety distance set to: '%f'", safety_distance_);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        custom_msgs::msg::ClosestObstacle message;
        message.distance = std::numeric_limits<double>::infinity();
        message.direction = "none";
        message.threshold = safety_distance_;

        // keep the closest of the four sectors
        for (const Sector &sector : SECTORS)
        {
            double minimum = min_scan_in_sector(msg, sector);
            if (minimum < message.distance)
            {
                message.distance = minimum;
                message.direction = sector.name;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Closest obstacle: '%f' on the '%s' sector, threshold: '%f'", message.distance, message.direction.c_str(), message.threshold);
        publisher_->publish(message);
    }

    rclcpp::Publisher<custom_msgs::msg::ClosestObstacle>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Service<custom_msgs::srv::SafetyDistance>::SharedPtr service_;

    double safety_distance_ = 0.4;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleMonitor>());
    rclcpp::shutdown();
    return 0;
}
