#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <cmath>

class dataProcessor : public rclcpp::Node
{
public:
    dataProcessor() : Node("data_processor")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&dataProcessor::scanCallback, this, std::placeholders::_1));

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/sensor_msgs/msg/LasScanMod", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received a LaserScan message with %zu ranges.", msg->ranges.size());

    float startAngle = -M_PI_4;  // -45 degrees
    float endAngle = M_PI_4;     // 45 degrees
    
    sensor_msgs::msg::LaserScan laserScan_ = *msg;
    float angleMin = laserScan_.angle_min;
    float angleMax = laserScan_.angle_max;
    float angleIncr = laserScan_.angle_increment;
    std::vector<float> ranges_ = laserScan_.ranges;

    // Normalize startAngle and endAngle to be within [angleMin, angleMax]
    float normalizedStartAngle = startAngle;
    float normalizedEndAngle = endAngle;

    if (normalizedStartAngle < angleMin) {
        normalizedStartAngle += 2 * M_PI;
    }
    if (normalizedEndAngle < angleMin) {
        normalizedEndAngle += 2 * M_PI;
    }

    // Calculate the indices corresponding to the normalized start and end angles
    int startIndex = std::round((normalizedStartAngle - angleMin) / angleIncr);
    int endIndex = std::round((normalizedEndAngle - angleMin) / angleIncr);

    // Ensure indices are within bounds, wrapping if necessary
    if (startIndex < 0) {
        startIndex += ranges_.size();
    }
    if (endIndex >= ranges_.size()) {
        endIndex -= ranges_.size();
    }

    RCLCPP_INFO(this->get_logger(), "Filtering ranges from index %d to %d.", startIndex, endIndex);

    std::vector<float> filtered_ranges;

    // Handle wrapping cases
    if (startIndex <= endIndex) {
        // Normal case: the range does not wrap
        filtered_ranges.assign(ranges_.begin() + startIndex, ranges_.begin() + endIndex + 1);
    } else {
        // Wrapping case: the range wraps around the end of the vector
        filtered_ranges.assign(ranges_.begin() + startIndex, ranges_.end());
        filtered_ranges.insert(filtered_ranges.end(), ranges_.begin(), ranges_.begin() + endIndex + 1);
    }

    // Update the LaserScan message
    laserScan_.ranges = filtered_ranges;
    laserScan_.angle_min = startAngle;
    laserScan_.angle_max = endAngle;

    // Publish the modified LaserScan message
    scan_pub_->publish(laserScan_);

    RCLCPP_INFO(this->get_logger(), "Published modified LaserScan message with %zu ranges.", filtered_ranges.size());
}

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dataProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
