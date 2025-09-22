#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;

    // ROS Subscribers and Publishers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_memory_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr pub_timer_;

    // Callback functions
    void costMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishMapMemory();
    void integrateCostmap();

    // Map memory
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_map_;

    //Parameters
    const double RESOLUTION = 0.1; // 10 cm per cell
    const int ARRAY_WIDTH = 300; // 30 meters wide
    const int ARRAY_HEIGHT = 300; // 30 meters tall

    // Robot movement tracking
    double last_x = 0.0;
    double last_y = 0.0;
    double yaw = 0.0;
    const double distance_threshold = 1.5; // Update map if robot moves more than 1.5 meters
    bool should_update_map_ = false;
    bool costmap_updated_ = false;
};

#endif 
