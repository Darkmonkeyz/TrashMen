#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <cmath>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

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
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 100, std::bind(&CylinderIdentifier::odomCallback,this,std::placeholders::_1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    nav_msgs::msg::Odometry odom_;
    std::vector<std::pair<float, float>> cylinders_;

    /**
     * @brief Callback function to process LaserScan messages.
     * @param msg The LaserScan message.
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<std::pair<float, float>> points = extractPoints(msg);
        cv::Mat image = cv::imread("/home/student/ros2_ws/src/sprint3_36/src/map2.pgm");
        if (image.empty()) {
            std::cerr << "Error: Could not load image." << std::endl;
        }
        cv::Mat resizedImage;
        cv::resize(image, resizedImage, cv::Size(image.cols / 2, image.rows / 2));

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
        const float distance_threshold = 0.1;

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
            if (std::abs(distanceFromCentre - 0.15) > 0.035) {
                //RCLCPP_INFO(this->get_logger(), "Cylinder not detected! at %f, %f", segmentCenterX, segmentCenterY);
                return false;
            }
        }

        rAvgSum /= segment.size();
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