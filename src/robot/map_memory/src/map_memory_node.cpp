#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  cost_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap",
      10,
      std::bind(&MapMemoryNode::costMapCallback, this, std::placeholders::_1));

  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered",
      10,
      std::bind(&MapMemoryNode::odometryCallback, this, std::placeholders::_1));

  map_memory_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map",
      10);

  pub_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), // Publish every 1s
      std::bind(&MapMemoryNode::publishMapMemory, this));
  

  global_map_.header.stamp = this->now();
  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = RESOLUTION; // 0.1 m per cell
  global_map_.info.width = ARRAY_WIDTH;     // 30 meters wide
  global_map_.info.height = ARRAY_HEIGHT;   // 30 meters tall
  global_map_.info.origin.position.x = -(ARRAY_WIDTH * global_map_.info.resolution) / 2.0;
  global_map_.info.origin.position.y = -(ARRAY_HEIGHT * global_map_.info.resolution) / 2.0;
}

void MapMemoryNode::costMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Store the latest costmap
  latest_map_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double current_x = msg->pose.pose.position.x;
  double current_y = msg->pose.pose.position.y;

  // Compute distance traveled
  double distance = std::sqrt(std::pow(current_x - last_x, 2) + std::pow(current_y - last_y, 2));
  if (distance >= distance_threshold)
  {
    last_x = current_x;
    last_y = current_y;
    should_update_map_ = true;
  }
}

// Integrate the latest costmap into the global map
void integrateCostmap()
{
  // Transform and merge the latest costmap into the global map
  
}

void MapMemoryNode::publishMapMemory()
{
  if (should_update_map_ && costmap_updated_)
  {
    integrateCostmap();
    map_memory_pub_->publish(global_map_);
    should_update_map_ = false;
  }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
