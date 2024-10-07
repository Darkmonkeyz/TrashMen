#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <cmath>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

class CylinderIdentifier : public rclcpp::Node
{
public:
    CylinderIdentifier()
    : Node("cylinder_identifier")
    {
        // Subscribe to the LaserScan topic
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&CylinderIdentifier::scanCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 100, std::bind(&CylinderIdentifier::odomCallback,this,std::placeholders::_1));
        
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    nav_msgs::msg::Odometry odom_;
    std::vector<std::pair<float, float>> cylinders_;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<std::pair<float, float>> points = extractPoints(msg);

        

        cv::Mat image = cv::imread("/home/student/ros2_ws/src/sprint3_36/src/map.pgm");
        if (image.empty()) {
        std::cerr << "Error: Could not load image." << std::endl;
        }
        

        // Segment laser scan points into continuous regions
        std::vector<std::vector<std::pair<float, float>>> segments = segmentPoints(points);

        // Check each segment for cylindrical shape
        for (const auto &segment : segments) {
            if (isCylindrical(segment)) {
                RCLCPP_INFO(this->get_logger(), "Cylinder detected!");
                // TODO: Add your code here to transform the cylinder's position into map coordinates
                for (auto cylinder : cylinders_){
                    cv::Point center = cv::Point(cylinder.first*100, cylinder.second*100);
                    cv::Scalar color(0, 255, 0);
                    int radius = 20; 
                    int thickness = 2;  
                    cv::circle(image, center, radius, color, thickness);
                    cv::imshow("Image with Circles", image);
                    cv::waitKey(1);
                }
            }
            else{
                for (auto cylinder : cylinders_){
                    cv::Point center = cv::Point(cylinder.first*100 + 488, 900 - (cylinder.second*100 + 450));
                    cv::Scalar color(0, 255, 0);
                    int radius = 20; 
                    int thickness = 2;  
                    cv::circle(image, center, radius, color, thickness);
                    cv::imshow("Image with Circles", image);
                    cv::waitKey(1);
                }
            }
        }

    }
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_ = *msg;
        
    }

    // Extract points from the LaserScan message (polar to Cartesian conversion)
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

    // Segment the points into continuous regions
    std::vector<std::vector<std::pair<float, float>>> segmentPoints(const std::vector<std::pair<float, float>> &points)
    {
        std::vector<std::vector<std::pair<float, float>>> segments;
        std::vector<std::pair<float, float>> current_segment;

        const float distance_threshold = 0.1;  // Adjust the distance threshold based on your requirements

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

        // Add the last segment
        if (!current_segment.empty()) {
            segments.push_back(current_segment);
        }

        return segments;
    }

    std::pair<float, float> normalizeVector(float x, float y){
        float magnitude = std::sqrt(x*x + y*y);
        return std::pair(x/magnitude, y/magnitude);
    }

    // Check if a segment forms a cylindrical shape (circle fitting)
    bool isCylindrical(const std::vector<std::pair<float, float>> &segment)
    {
        float segmentCenterX;
        float segmentCenterY;
        if(segment.size() > 4){
        if (segment.size() % 2){

            std::pair<float, float> magnitude = normalizeVector(segment.at(std::round(segment.size()/2)).first, segment.at(std::round(segment.size()/2)).second);
            segmentCenterX = (segment.at(std::round(segment.size()/2)).first + magnitude.first *0.15);
            segmentCenterY = (segment.at(std::round(segment.size()/2)).second + magnitude.second *0.15);
        }
        else{
            std::pair<float,float> middleSeg = std::pair<float,float>((segment.at(segment.size()/2).first + segment.at((segment.size()/2)+1).first)/2 , (segment.at(segment.size()/2).second + segment.at((segment.size()/2)+1).second) /2); 
            std::pair<float, float> magnitude = normalizeVector(middleSeg.first, middleSeg.second);
            segmentCenterX = (middleSeg.first + magnitude.first *0.15);
            segmentCenterY = (middleSeg.second + magnitude.second *0.15);
        }
        }
        else{
            return false;
        }
        

        float rAvgSum = 0;
        for (auto point : segment){
            float distanceFromCentre = std::hypot(point.first - segmentCenterX, point.second - segmentCenterY);
            rAvgSum = rAvgSum + distanceFromCentre;
            if (std::abs(distanceFromCentre - 0.15) > 0.035){
                RCLCPP_INFO(this->get_logger(), "Cylinder not detected! at %f, %f", segmentCenterX, segmentCenterY);
                return false;
                
                
            }
        }
        rAvgSum = rAvgSum/segment.size();

        addCylinderLocation(segmentCenterX, segmentCenterY);
        return true;

        




        
    }

    bool addCylinderLocation(float x, float y){
        float roboX = odom_.pose.pose.position.x;
        float roboY = odom_.pose.pose.position.y;
        double robot_yaw = std::atan2(2.0 * (odom_.pose.pose.orientation.w * odom_.pose.pose.orientation.z + odom_.pose.pose.orientation.x * odom_.pose.pose.orientation.y), 1.0 - 2.0 * (odom_.pose.pose.orientation.y * odom_.pose.pose.orientation.y + odom_.pose.pose.orientation.z * odom_.pose.pose.orientation.z));
        std::pair<float, float> cylinder = std::make_pair(roboX + (std::cos(robot_yaw) * x - std::sin(robot_yaw) * y), roboY + (std::sin(robot_yaw) * x + std::cos(robot_yaw) * y));
        for (auto point : cylinders_){
            if (std::hypot(point.first -cylinder.first, point.second, cylinder.second) < 0.5){
                return false;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Cylinder added! at %f, %f", cylinder.first, cylinder.second);
        cylinders_.push_back(cylinder);


        return true;
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CylinderIdentifier>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}