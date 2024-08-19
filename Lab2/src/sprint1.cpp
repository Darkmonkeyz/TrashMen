#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <cmath>

class sprint1 : public rclcpp::Node
{
public:
    sprint1() : Node("data_processor")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&sprint1::scanCallback, this, std::placeholders::_1));

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/sensor_msgs/msg/LasScanMod", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received a LaserScan message with %zu ranges.", msg->ranges.size());
    //Set N in this case 3. 
    int nth = 3;
    //read in the msg convert it to a laser scan
    sensor_msgs::msg::LaserScan laserScan_ = *msg;
    float angleMin = laserScan_.angle_min;
    float angleMax = laserScan_.angle_max;
    float angleIncr = laserScan_.angle_increment;
    //create vector of ranges from the msg
    std::vector<float> ranges_ = laserScan_.ranges;

    



   //create vector of ranges to store modification
    std::vector<float> filtered_ranges;
    //simple for loop with i as iterator (no auto because we want indexes)
    for (int i = 0; laserScan_.ranges.size() > i ; i++){
        //simple if statement using modulo to see if its multiple.
        if (i % nth == 0){
            //chuck it into the filtered range
            filtered_ranges.push_back(ranges_[i]);
        }
    }

    // Update the LaserScan message to accomodate for the new angle sizes
    laserScan_.angle_increment =  angleIncr*nth;
    laserScan_.angle_max = angleMin + laserScan_.angle_increment * (filtered_ranges.size());

    laserScan_.ranges = filtered_ranges;

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
    auto node = std::make_shared<sprint1>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
