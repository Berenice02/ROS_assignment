#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/srv/average_velocity.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <deque>

using std::placeholders::_1;
using std::placeholders::_2;

// the average is made on the most recent inputs of the user
constexpr size_t WINDOW = 5;

class VelocityMonitor : public rclcpp::Node
{
public:
    VelocityMonitor() : Node("velocity_monitor")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/user_input", 10, std::bind(&VelocityMonitor::user_input_callback, this, _1));
        service_ = this->create_service<custom_msgs::srv::AverageVelocity>("get_average_velocity", std::bind(&VelocityMonitor::average_callback, this, _1, _2));
    }

private:
    void user_input_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Add the new message and remove older ones
        user_inputs_.push_back(*msg);
        while (user_inputs_.size() > WINDOW)
        {
            user_inputs_.pop_front();
        }

        RCLCPP_INFO(this->get_logger(), "Received command: '%f', '%f', stored: '%zu'", msg->linear.x, msg->angular.z, user_inputs_.size());
    }

    void average_callback(const std::shared_ptr<custom_msgs::srv::AverageVelocity::Request> request,
                          std::shared_ptr<custom_msgs::srv::AverageVelocity::Response> response)
    {
        (void)request;

        response->linear = 0.0;
        response->angular = 0.0;

        if (user_inputs_.size() < WINDOW)
        {
            response->success = false;
            response->message = "Not enough inputs for the average yet";
            RCLCPP_WARN(this->get_logger(), "Only '%zu' inputs received, no average", user_inputs_.size());
            return;
        }

        for (const geometry_msgs::msg::Twist &user_input : user_inputs_)
        {
            response->linear += user_input.linear.x;
            response->angular += user_input.angular.z;
        }
        response->linear /= user_inputs_.size();
        response->angular /= user_inputs_.size();
        response->success = true;
        response->message = "average of the last inputs";

        RCLCPP_INFO(this->get_logger(), "Average of the last '%zu' inputs: '%f', '%f'", user_inputs_.size(), response->linear, response->angular);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Service<custom_msgs::srv::AverageVelocity>::SharedPtr service_;

    std::deque<geometry_msgs::msg::Twist> user_inputs_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityMonitor>());
    rclcpp::shutdown();
    return 0;
}
