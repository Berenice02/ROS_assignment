#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/closest_obstacle.hpp"
#include "custom_msgs/msg/obstacles.hpp"
#include "custom_msgs/srv/safety_distance.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <limits>

using std::placeholders::_1;
using std::placeholders::_2;

struct Sector
{
    int first;
    int last;
    const char *name;
};

// the lidar publishes 720 beams over 360 degrees, starting from the rear
// same order as the indices of custom_msgs/msg/Obstacles
constexpr Sector SECTORS[4] = {
    {0, 179, "rear right"},
    {180, 359, "front right"},
    {360, 539, "front left"},
    {540, 719, "rear left"}};

class ObstacleMonitor : public rclcpp::Node
{
public:
    ObstacleMonitor() : Node("obstacle_monitor")
    {
        closest_publisher_ = this->create_publisher<custom_msgs::msg::ClosestObstacle>("/closest_obstacle", 10);
        obstacles_publisher_ = this->create_publisher<custom_msgs::msg::Obstacles>("/obstacles", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&ObstacleMonitor::scan_callback, this, _1));
        service_ = this->create_service<custom_msgs::srv::SafetyDistance>("set_safety_distance", std::bind(&ObstacleMonitor::safety_callback, this, _1, _2));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                         std::bind(&ObstacleMonitor::timer_callback, this));
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

    void timer_callback()
    {
        if (!scan_)
        {
            return;
        }

        custom_msgs::msg::Obstacles message_obstacles;
        custom_msgs::msg::ClosestObstacle closest_obs_message;
        closest_obs_message.distance = std::numeric_limits<double>::infinity();
        closest_obs_message.direction = "none";
        closest_obs_message.threshold = safety_distance_;

        // one entry per sector, and keep the closest of the four
        for (size_t i = 0; i < message_obstacles.sectors.size(); i++)
        {
            message_obstacles.sectors[i].distance = min_scan_in_sector(SECTORS[i]);
            message_obstacles.sectors[i].direction = SECTORS[i].name;
            message_obstacles.sectors[i].threshold = safety_distance_;

            if (message_obstacles.sectors[i].distance < closest_obs_message.distance)
            {
                closest_obs_message = message_obstacles.sectors[i];
            }
        }

        obstacles_publisher_->publish(message_obstacles);
        closest_publisher_->publish(closest_obs_message);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;

    rclcpp::Publisher<custom_msgs::msg::Obstacles>::SharedPtr obstacles_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_msgs::msg::ClosestObstacle>::SharedPtr closest_publisher_;

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
