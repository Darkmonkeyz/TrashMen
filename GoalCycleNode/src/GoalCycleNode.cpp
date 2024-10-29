#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class GoalCycleNode : public rclcpp::Node {
public:
    GoalCycleNode()
        : Node("goal_cycle_node"),
          goal_index_(0), paused_(false) {
        // Initialize the action client
        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

        // Initialize goals
        goals_ = {
            create_pose_stamped(4.0, -4.0, 0.0),
            create_pose_stamped(4.0, 4.0, 0.0),
            create_pose_stamped(-4.0, 4.0, 0.0),
            create_pose_stamped(-4.0, -4.0, 0.0)
        };
        
        // Declare and monitor the `pauseNav` parameter
        this->declare_parameter("pauseNav", false);
        parameter_event_sub_ = this->add_on_set_parameters_callback(
            std::bind(&GoalCycleNode::parameter_callback, this, std::placeholders::_1)
        );

        wait_for_action_server();
    }

private:
    std::vector<geometry_msgs::msg::PoseStamped> goals_;
    size_t goal_index_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    bool paused_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_event_sub_;

    // Create a PoseStamped message for each goal
    geometry_msgs::msg::PoseStamped create_pose_stamped(double x, double y, double yaw) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.z = sin(yaw / 2.0);
        pose.pose.orientation.w = cos(yaw / 2.0);
        return pose;
    }

    // Wait for the action server to become available
    void wait_for_action_server() {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        } else {
            if (paused_) {
            RCLCPP_INFO(this->get_logger(), "Navigation paused.");
            return;
        }
            send_next_goal();
        }
    }

    // Send the current goal from the list if not paused
    void send_next_goal() {
        if (paused_) {
            RCLCPP_INFO(this->get_logger(), "Navigation paused.");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goals_[goal_index_];

        RCLCPP_INFO(this->get_logger(), "Sending goal %zu", goal_index_);

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
        options.goal_response_callback = std::bind(
            &GoalCycleNode::goal_response_callback, this, std::placeholders::_1);
        options.result_callback = std::bind(
            &GoalCycleNode::result_callback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, options);
    }

    // Callback to handle parameter updates
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters) {
        for (const auto &param : parameters) {
            if (param.get_name() == "pauseNav") {
                paused_ = param.as_bool();
                RCLCPP_INFO(this->get_logger(), "Pause state changed to: %s", paused_ ? "True" : "False");

                // If paused was set to false, resume navigation
                if (!paused_) {
                    send_next_goal();
                }
            }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    // Handle the goal response
    void goal_response_callback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted");
        }
    }

    // Handle the result of the goal
    void result_callback(const GoalHandleNavigateToPose::WrappedResult &result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Goal %zu reached successfully", goal_index_);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Goal %zu failed", goal_index_);
        }

        // Move to the next goal in the list
        goal_index_ = (goal_index_ + 1) % goals_.size();
        send_next_goal();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalCycleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
