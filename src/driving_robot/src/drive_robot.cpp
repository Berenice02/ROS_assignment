#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class DriveRobot : public rclcpp::Node
{
public:
    DriveRobot() : Node("drive_robot")
    {
        // User choice implemented through node parameters
        this->declare_parameter("linear_speed", 0.0);
        this->declare_parameter("angular_speed", 0.0);

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&DriveRobot::timer_callback, this));
    }

private:
    void timer_callback()
    {
        message.linear.x = this->get_parameter("linear_speed").as_double();
        message.angular.z = this->get_parameter("angular_speed").as_double();
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f', '%f'", message.linear.x, message.angular.z);
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist message;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveRobot>());
    rclcpp::shutdown();
    return 0;
}
