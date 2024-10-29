#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "example_interfaces/srv/trigger.hpp"
#include <memory>
#include <vector>

class NavigationHandler : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigationHandler() : Node("navigation_handler") {
        // Create the action client for navigation
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this,
            "navigate_to_pose"
        );

        // Declare and set the pauseNav parameter initially as false
        this->declare_parameter("pauseNav", false);

        // Create the service
        diversion_service_ = this->create_service<example_interfaces::srv::Trigger>(
            "request_diversion",
            std::bind(&NavigationHandler::handle_diversion_request, this,
                     std::placeholders::_1, std::placeholders::_2));

        estop_service = this->create_service<example_interfaces::srv::Trigger>(
            "e_stop_service",
            std::bind(&NavigationHandler::handle_diversion_request, this,
                     std::placeholders::_1, std::placeholders::_2));             

        // Store the current pose from odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&NavigationHandler::odom_callback, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr diversion_service_;
    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr estop_service;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    nav_msgs::msg::Odometry::SharedPtr current_pose_;
    geometry_msgs::msg::PoseStamped original_goal_;
    bool is_diversion_active_ = false;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg;
    }

    void handle_diversion_request(
    const std::shared_ptr<example_interfaces::srv::Trigger::Request>,
    std::shared_ptr<example_interfaces::srv::Trigger::Response> response)
{
    if (is_diversion_active_) {
        response->success = false;
        response->message = "Diversion already in progress";
        return;
    }

    is_diversion_active_ = true;

    // Notify GoalCycleNode to pause
    this->set_parameter(rclcpp::Parameter("pauseNav", true));

    // Save current goal if needed
    original_goal_ = create_original_goal();

    // Create diversion goal
    geometry_msgs::msg::PoseStamped diversion_goal;
    diversion_goal.header.frame_id = "map";
    diversion_goal.header.stamp = this->now();
    diversion_goal.pose.position.x = -2.0;  // Example coordinates
    diversion_goal.pose.position.y = -2.0;
    diversion_goal.pose.orientation = current_pose_->pose.pose.orientation;

    // Send robot to diversion point
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = diversion_goal;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NavigationHandler::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NavigationHandler::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&NavigationHandler::result_callback, this, std::placeholders::_1);

    nav_client_->async_send_goal(goal_msg, send_goal_options);

    response->success = true;
    response->message = "Diversion initiated";
}

    geometry_msgs::msg::PoseStamped create_original_goal() {
        // Save current pose as the original goal
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = current_pose_->pose.pose.position.x;
        pose.pose.position.y = current_pose_->pose.pose.position.y;
        pose.pose.orientation = current_pose_->pose.pose.orientation;
        return pose;
    }

    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
    }

    void feedback_callback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Distance to goal: %.2f", feedback->distance_remaining);
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result) {
        if (is_diversion_active_) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Diversion complete, returning to original path");
                    return_to_path();
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Navigation failed");
                    is_diversion_active_ = false;
                    // Set pauseNav to false after failure
                    //this->set_parameter(rclcpp::Parameter("pauseNav", false));
                    break;
            }
        }
    }

    void return_to_path() {
        // Create goal to return to original path
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = original_goal_;  // Use stored original goal

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&NavigationHandler::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&NavigationHandler::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&NavigationHandler::result_callback, this, std::placeholders::_1);

        nav_client_->async_send_goal(goal_msg, send_goal_options);

        // Reset pauseNav to indicate normal operation after diversion
        this->set_parameter(rclcpp::Parameter("pauseNav", false));
        is_diversion_active_ = false;
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationHandler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
