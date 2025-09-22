#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  cost_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap",
      10,
      std::bind(&MapMemoryNode::costMapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom/filtered" ,
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
  global_map_.data.resize(ARRAY_WIDTH * ARRAY_HEIGHT, 0);
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

  // Extract yaw from quaternion
  const auto &q = msg->pose.pose.orientation;
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  yaw = std::atan2(siny_cosp, cosy_cosp);

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
void MapMemoryNode::integrateCostmap()
{
  // Transform and merge the latest costmap into the global map
  // for (int j = 0; j < latest_map_.info.height; ++j)
  // {
  //   for (int i = 0; i < latest_map_.info.width; ++i)
  //   {
  //     int global_x = (i - latest_map_.info.width / 2) * std::cos(yaw) - (j - latest_map_.info.height / 2) * std::sin(yaw) + (global_map_.info.width / 2);
  //     int global_y = (j - latest_map_.info.height / 2) * std::cos(yaw) + (i - latest_map_.info.width / 2) * std::sin(yaw) + (global_map_.info.height / 2);
  //     if (global_x >= 0 && global_x < global_map_.info.width && global_y >= 0 && global_y < global_map_.info.height)
  //     {
  //       int global_index = global_y * global_map_.info.width + global_x;
  //       // Simple max merge strategy
  //       global_map_.data[global_index] = std::max(global_map_.data[global_index], latest_map_.data[index]);
  //     }
  //   }
  // }
}

void MapMemoryNode::publishMapMemory()
{
  if (should_update_map_ && costmap_updated_)
  {
    integrateCostmap();
    global_map_.header.stamp = this->now();
    map_memory_pub_->publish(global_map_);
    should_update_map_ = false;
    costmap_updated_ = false;
  }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
