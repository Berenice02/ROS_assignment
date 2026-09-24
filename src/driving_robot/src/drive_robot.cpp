#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "custom_msgs/msg/obstacles.hpp"
#include <algorithm>
#include <limits>

using std::placeholders::_1;
using custom_msgs::msg::Obstacles;

class DriveRobot : public rclcpp::Node
{
public:
    DriveRobot() : Node("drive_robot")
    {
        // User choice implemented through node parameters
        this->declare_parameter("linear_speed", 0.0);
        this->declare_parameter("angular_speed", 0.0);

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        input_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/user_input", 10);
        subscription_ = this->create_subscription<Obstacles>("/obstacles", 10, std::bind(&DriveRobot::obstacles_callback, this, _1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                         std::bind(&DriveRobot::timer_callback, this));
    }

private:
    void obstacles_callback(const Obstacles::SharedPtr msg)
    {
        obstacles_ = msg;
    }

    double sector_distance(uint8_t sector)
    {
        return obstacles_->sectors[sector].distance;
    }

    double min_distance(const geometry_msgs::msg::Twist &velocity)
    {
        // select the sector according to linear/angular velocities
        if (velocity.linear.x > 0.0)
        {
            if (velocity.angular.z > 0.0)
            {
                return sector_distance(Obstacles::FRONT_LEFT);
            }
            if (velocity.angular.z < 0.0)
            {
                return sector_distance(Obstacles::FRONT_RIGHT);
            }
            return std::min(sector_distance(Obstacles::FRONT_RIGHT), sector_distance(Obstacles::FRONT_LEFT));
        }

        if (velocity.linear.x < 0.0)
        {
            if (velocity.angular.z > 0.0)
            {
                return sector_distance(Obstacles::REAR_RIGHT);
            }
            if (velocity.angular.z < 0.0)
            {
                return sector_distance(Obstacles::REAR_LEFT);
            }
            return std::min(sector_distance(Obstacles::REAR_RIGHT), sector_distance(Obstacles::REAR_LEFT));
        }

        return std::numeric_limits<double>::infinity();
    }

    void timer_callback()
    {
        if (!obstacles_)
        {
            return;
        }

        double safety_distance = obstacles_->sectors[0].threshold;

        if (recovering_)
        {
            // Coming back until safe
            if (min_distance(blocked_) < safety_distance)
            {
                message_.linear.x = -blocked_.linear.x;
                message_.angular.z = -blocked_.angular.z;
            }
            // And then stop
            else
            {
                message_.linear.x = 0.0;
                message_.angular.z = 0.0;
                recovering_ = false;
                this->set_parameter(rclcpp::Parameter("linear_speed", 0.0));
                this->set_parameter(rclcpp::Parameter("angular_speed", 0.0));
                last_user_input_ = geometry_msgs::msg::Twist();
                RCLCPP_WARN(this->get_logger(), "Safe area reached, stop");
            }
        }
        else
        {
            geometry_msgs::msg::Twist velocity_input;
            velocity_input.linear.x = this->get_parameter("linear_speed").as_double();
            velocity_input.angular.z = this->get_parameter("angular_speed").as_double();

            // If it's a new user input
            if (velocity_input.linear.x != last_user_input_.linear.x
                || velocity_input.angular.z != last_user_input_.angular.z)
            {
                last_user_input_ = velocity_input;
                input_publisher_->publish(velocity_input);
                RCLCPP_INFO(this->get_logger(), "New user input: '%f', '%f'", velocity_input.linear.x, velocity_input.angular.z);
            }

            if (min_distance(velocity_input) < safety_distance)
            {
                recovering_ = true;
                blocked_ = velocity_input;
                message_.linear.x = -velocity_input.linear.x;
                message_.angular.z = -velocity_input.angular.z;
                RCLCPP_WARN(this->get_logger(), "Too close to an obstacle, moving back");
            }
            else
            {
                message_ = velocity_input;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Publishing: '%f', '%f'", message_.linear.x, message_.angular.z);
        publisher_->publish(message_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr input_publisher_;

    rclcpp::Subscription<Obstacles>::SharedPtr subscription_;
    Obstacles::SharedPtr obstacles_;

    geometry_msgs::msg::Twist message_;
    geometry_msgs::msg::Twist blocked_;
    geometry_msgs::msg::Twist last_user_input_;

    bool recovering_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveRobot>());
    rclcpp::shutdown();
    return 0;
}
