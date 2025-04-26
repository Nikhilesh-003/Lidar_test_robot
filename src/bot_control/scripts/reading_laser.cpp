#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class LidarSub : public rclcpp::Node
{
public:
    LidarSub()
    : Node("lidar_sub_node")
    {
        // Subscriber to original /scan topic
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&LidarSub::lidarCallback, this, _1)
        );

        // Publisher to new /filtered_scan topic
        filtered_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/filtered_scan",
            10
        );
    }

private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Filter from 0 to 120 degrees (out of full 360°)
        size_t total_samples = msg->ranges.size();
        size_t fov_samples = static_cast<size_t>(total_samples * (120.0 / 360.0));

        auto filtered_scan = sensor_msgs::msg::LaserScan();
        filtered_scan.header = msg->header;
        filtered_scan.angle_min = msg->angle_min;
        filtered_scan.angle_max = msg->angle_min + (msg->angle_increment * fov_samples);
        filtered_scan.angle_increment = msg->angle_increment;
        filtered_scan.time_increment = msg->time_increment;
        filtered_scan.scan_time = msg->scan_time;
        filtered_scan.range_min = msg->range_min;
        filtered_scan.range_max = msg->range_max;

        // Copy only the required ranges and intensities
        filtered_scan.ranges.assign(msg->ranges.begin(), msg->ranges.begin() + fov_samples);
        filtered_scan.intensities.assign(msg->intensities.begin(), msg->intensities.begin() + fov_samples);

        // Publish the filtered scan
        filtered_pub_->publish(filtered_scan);

        // (Optional) Also print the filtered ranges
        RCLCPP_INFO(this->get_logger(), "Filtered scan published with %zu samples.", fov_samples);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSub>());
    rclcpp::shutdown();
    return 0;
}
