#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "costmap_core.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

  private:
    robot::CostmapCore costmap_;
    

    //ROS Subscribers and Publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;

    // Callback function for laser scan messages
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
};

#endif 