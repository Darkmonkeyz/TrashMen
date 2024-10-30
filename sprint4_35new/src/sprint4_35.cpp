#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <cmath>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

/**
 * @class CylinderIdentifier
 * @brief A ROS node for identifying cylindrical objects from LaserScan data.
 */
class CylinderIdentifier : public rclcpp::Node
{
public:
    /**
     * @brief Constructor initializes the node and subscribes to topics.
     */
    CylinderIdentifier()
    : Node("cylinder_identifier")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&CylinderIdentifier::scanCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&CylinderIdentifier::odomCallback,this,std::placeholders::_1));
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/plan", 10, std::bind(&CylinderIdentifier::pathCallback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10); 
        nav2_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    nav2_msgs::action::NavigateToPose::Goal current_goal_; // Store the original goal
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_; // To publish twist commands
rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_action_client_;
rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::GoalHandle::SharedPtr current_goal_handle_; // Store the active goal handle
    
    nav_msgs::msg::Odometry odom_;
    std::vector<std::pair<float, float>> cylinders_;

    /**
     * @brief Callback function to process LaserScan messages.
     * @param msg The LaserScan message.
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<std::pair<float, float>> points = extractPoints(msg);
        cv::Mat image = cv::imread("/home/student/ros2_ws/src/sprint4_35/src/map3.pgm");
        if (image.empty()) {
            std::cerr << "Error: Could not load image." << std::endl;
        }
        cv::Mat resizedImage;
        cv::resize(image, resizedImage, cv::Size(image.cols / 4, image.rows / 4));

        std::vector<std::vector<std::pair<float, float>>> segments = segmentPoints(points);

        for (const auto &segment : segments) {
            if (isCylindrical(segment)) {
                RCLCPP_INFO(this->get_logger(), "Cylinder detected!");
                
            }
           
            
        }
        for (auto cylinder : cylinders_) {
                    cv::Point center = cv::Point(cylinder.first*56 + 400, 800 - (cylinder.second*56 + 400));
                    cv::Scalar color(0, 255, 0);
                    int radius = 10; 
                    int thickness = 2;  
                    cv::circle(resizedImage, center, radius, color, thickness);
                }
        cv::imshow("Image with Circles", resizedImage);
            cv::waitKey(1);
    }

    /**
     * @brief Callback function to process Odometry messages.
     * @param msg The Odometry message.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_ = *msg;
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        cv::Mat image = cv::imread("/home/student/ros2_ws/src/sprint4_35/src/map3.pgm");
        if (image.empty()) {
            std::cerr << "Error: Could not load image." << std::endl;
        }
        cv::Mat resizedImage;
        cv::resize(image, resizedImage, cv::Size(image.cols / 4, image.rows / 4));

        //geometry_msgs::msg::PoseStamped last_pose = msg->poses.back();
        //current_goal_.pose = last_pose;
        
        for (const auto &pose : msg->poses) {
            cv::Point center = cv::Point(pose.pose.position.x*56 + 400, 800 - (pose.pose.position.y*56 + 400));
                    cv::Scalar color(255, 0, 0);
                    int radius = 2; 
                    int thickness = 2;  
                    cv::circle(resizedImage, center, radius, color, thickness);
            for (const auto &cylinder : cylinders_) {
                if (isIntersecting(pose.pose.position.x, pose.pose.position.y, cylinder.first, cylinder.second)) {
                    RCLCPP_WARN(this->get_logger(), "Predicted path intersects with a cylinder! Interrupting navigation.");
                    interruptNavigation(); // Method to interrupt navigation
                    return; // Exit if an intersection is found
                }
            }
        }
        cv::imshow("Image with path", resizedImage);
            cv::waitKey(1);
    }
    bool isIntersecting(float path_x, float path_y, float cylinder_x, float cylinder_y) {
        const float threshold = 0.5; // Threshold distance to consider as intersecting
        return std::hypot(path_x - cylinder_x, path_y - cylinder_y) < threshold;
    }

    // Method to interrupt navigation 
    void interruptNavigation() {
        RCLCPP_INFO(this->get_logger(), "Interrupting navigation due to detected cylinder.");

    

    // Cancel the current goal in `nav2`
    cancelCurrentGoal();

    // Step 2: Drive to the cylinder
    driveToCylinder();

    // Step 3: Drive around the cylinder
    circumnavigateCylinder();

    // Step 4: Resume the original goal
    resumeNavigation();
    }


    void cancelCurrentGoal()
{
    // Check if the action server is available
    if (!nav2_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available for canceling!");
        return;
    }

    // Check if there is an active goal handle to cancel
    if (current_goal_handle_) {  // Ensure current_goal_handle_ is a member variable storing the goal handle
        // Cancel the current goal
        auto cancel_result = nav2_action_client_->async_cancel_goal(current_goal_handle_);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), cancel_result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Navigation paused by canceling the current goal.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to cancel the navigation goal.");
        }

        // Optionally reset the current goal handle after cancellation
        current_goal_handle_ = nullptr;
    } else {
        RCLCPP_WARN(this->get_logger(), "No active goal to cancel.");
    }
}

    void driveToCylinder()
{
    RCLCPP_INFO(this->get_logger(), "Driving to cylinder location...");

    geometry_msgs::msg::Twist twist_msg;
    float target_x = cylinders_.back().first;
    float target_y = cylinders_.back().second;

    // Simple approach to move towards the target (implementing a basic proportional controller)
    while (rclcpp::ok()) {
        // Calculate distance and angle to target
        float dx = target_x - odom_.pose.pose.position.x;
        float dy = target_y - odom_.pose.pose.position.y;
        float distance = std::hypot(dx, dy);
        float target_angle = std::atan2(dy, dx);

        // Proportional control for rotation and forward movement
        twist_msg.linear.x = std::min(0.2f, distance);
        twist_msg.angular.z = std::clamp(2.0 * (target_angle - getRobotYaw()), -1.0, 1.0);

        cmd_pub_->publish(twist_msg);

        if (distance < 0.2) break;  // Stop when close to the cylinder

        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

// Helper function to drive around the cylinder
void circumnavigateCylinder()
{
    RCLCPP_INFO(this->get_logger(), "Driving around the cylinder...");

    geometry_msgs::msg::Twist twist_msg;

    // Move around the cylinder in an arc
    twist_msg.linear.x = 0.1;
    twist_msg.angular.z = 0.5;  // Adjust to control the turning speed around the cylinder

    for (int i = 0; i < 20; ++i) {
        cmd_pub_->publish(twist_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop after the arc
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    cmd_pub_->publish(twist_msg);
}

double getRobotYaw() const
{
    return std::atan2(2.0 * (odom_.pose.pose.orientation.w * odom_.pose.pose.orientation.z +
                             odom_.pose.pose.orientation.x * odom_.pose.pose.orientation.y),
                      1.0 - 2.0 * (odom_.pose.pose.orientation.y * odom_.pose.pose.orientation.y +
                                   odom_.pose.pose.orientation.z * odom_.pose.pose.orientation.z));
}

void resumeNavigation()
{
    if (current_goal_handle_ != nullptr) {
        RCLCPP_INFO(this->get_logger(), "Resending the stored goal.");

        // Send the previously interrupted goal
        auto goal_handle_future = nav2_action_client_->async_send_goal(current_goal_);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            current_goal_handle_ = goal_handle_future.get(); // Store the new goal handle
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to resend the goal.");
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "No interrupted goal stored to resend.");
    }
}

    /**
     * @brief Converts LaserScan data from polar to Cartesian coordinates.
     * @param msg The LaserScan message.
     * @return Vector of (x, y) points.
     */
    std::vector<std::pair<float, float>> extractPoints(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<std::pair<float, float>> points;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (std::isfinite(msg->ranges[i])) {
                float angle = msg->angle_min + i * msg->angle_increment;
                float x = msg->ranges[i] * cos(angle);
                float y = msg->ranges[i] * sin(angle);
                points.push_back({x, y});
            }
        }
        return points;
    }

    /**
     * @brief Segments LaserScan points into continuous regions.
     * @param points Vector of (x, y) points.
     * @return Vector of segmented point groups.
     */
    std::vector<std::vector<std::pair<float, float>>> segmentPoints(const std::vector<std::pair<float, float>> &points)
    {
        std::vector<std::vector<std::pair<float, float>>> segments;
        std::vector<std::pair<float, float>> current_segment;
        const float distance_threshold = 0.05;

        for (size_t i = 1; i < points.size(); ++i) {
            float dist = std::hypot(points[i].first - points[i-1].first, points[i].second - points[i-1].second);
            if (dist < distance_threshold) {
                current_segment.push_back(points[i]);
            } else {
                if (!current_segment.empty()) {
                    segments.push_back(current_segment);
                    current_segment.clear();
                }
            }
        }

        if (!current_segment.empty()) {
            segments.push_back(current_segment);
        }

        return segments;
    }

    /**
     * @brief Normalizes a 2D vector.
     * @param x The x component.
     * @param y The y component.
     * @return Normalized (x, y) vector.
     */
    std::pair<float, float> normalizeVector(float x, float y)
    {
        float magnitude = std::sqrt(x*x + y*y);
        return std::pair(x/magnitude, y/magnitude);
    }

    /**
     * @brief Determines if a segment forms a cylindrical shape.
     * @param segment The segment of points.
     * @return True if cylindrical, false otherwise.
     */
    bool isCylindrical(const std::vector<std::pair<float, float>> &segment)
    {
        float segmentCenterX;
        float segmentCenterY;
        if (segment.size() > 5) {
            if (segment.size() % 2) {
                std::pair<float, float> magnitude = normalizeVector(segment.at(std::round(segment.size()/2)).first, segment.at(std::round(segment.size()/2)).second);
                segmentCenterX = segment.at(std::round(segment.size()/2)).first + magnitude.first * 0.15;
                segmentCenterY = segment.at(std::round(segment.size()/2)).second + magnitude.second * 0.15;
            } else {
                std::pair<float,float> middleSeg = std::pair<float,float>((segment.at(segment.size()/2).first + segment.at((segment.size()/2)+1).first)/2, 
                    (segment.at(segment.size()/2).second + segment.at((segment.size()/2)+1).second) /2); 
                std::pair<float, float> magnitude = normalizeVector(middleSeg.first, middleSeg.second);
                segmentCenterX = middleSeg.first + magnitude.first * 0.145;
                segmentCenterY = middleSeg.second + magnitude.second * 0.145;
            }
        } else {
            return false;
        }

        float rAvgSum = 0;
        for (auto point : segment) {
            float distanceFromCentre = std::hypot(point.first - segmentCenterX, point.second - segmentCenterY);
            rAvgSum += distanceFromCentre;
            if (std::abs(distanceFromCentre - 0.15) > 0.015) {
                //RCLCPP_INFO(this->get_logger(), "Cylinder not detected! at %f, %f", segmentCenterX, segmentCenterY);
                return false;
            }
        }

        rAvgSum /= segment.size();
        RCLCPP_INFO(this->get_logger(), "Cylinder radius! at %f", rAvgSum);
        addCylinderLocation(segmentCenterX, segmentCenterY);
        return true;
    }

    /**
     * @brief Adds the detected cylinder's location based on robot's odometry.
     * @param x The x coordinate of the cylinder.
     * @param y The y coordinate of the cylinder.
     * @return True if added, false if already exists.
     */
    bool addCylinderLocation(float x, float y)
    {
        float roboX = odom_.pose.pose.position.x;
        float roboY = odom_.pose.pose.position.y;
        double robot_yaw = std::atan2(2.0 * (odom_.pose.pose.orientation.w * odom_.pose.pose.orientation.z + odom_.pose.pose.orientation.x * odom_.pose.pose.orientation.y), 
                                      1.0 - 2.0 * (odom_.pose.pose.orientation.y * odom_.pose.pose.orientation.y + odom_.pose.pose.orientation.z * odom_.pose.pose.orientation.z));
        std::pair<float, float> cylinder = std::make_pair(roboX + (std::cos(robot_yaw) * x - std::sin(robot_yaw) * y), 
                                                          roboY + (std::sin(robot_yaw) * x + std::cos(robot_yaw) * y));
        for (auto point : cylinders_) {
            if (std::hypot(point.first - cylinder.first, point.second - cylinder.second) < 0.5) {
                return false;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Cylinder added! at %f, %f", cylinder.first, cylinder.second);
        cylinders_.push_back(cylinder);
        return true;
    }
};

/**
 * @brief Main function to run the CylinderIdentifier node.
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CylinderIdentifier>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

