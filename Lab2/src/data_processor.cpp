#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


class dataProcessor : public rclcpp::Node
{
public:
    daraProcessor() : Node("data_processor")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(            "/sensor_msgs/msg/LaserScan", 10, std::bind(&dataProcessor::scanCallback, this, std::placeholders::_1));

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/sensor_msgs/msg/LasScanMod", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        try
        {
            if(laserProcessingPtr_ == nullptr){
        laserProcessingPtr_ = std::make_unique<LaserProcessing>(*msg);
        }
        else{
        laserProcessingPtr_->newScan(*msg);
        }
        


            image_pub_->publish(*modified_msg);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dataProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}