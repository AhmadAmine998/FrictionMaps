#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    // rclcpp::shutdown();
    return 0;
}