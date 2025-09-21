#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar",
      10,
      std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/costmap",
      10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  std::vector<std::vector<int8_t>> array_2d;
  const int ARRAY_WIDTH = 300;
  const int ARRAY_HEIGHT = 300;
  const double RESOLUTION = 0.1; // Each cell is 0.1m x 0.1m, which makes the 2D array 30m x 30m (30x30 squares in foxglove)
  const int8_t MAX_COST = 100; // Cost value for occupied cells
  const double INFLATION_RADIUS = 1.5; // Inflate obstacles by 1m

  array_2d.resize(ARRAY_HEIGHT, std::vector<int8_t>(ARRAY_WIDTH, 0)); // Create a 2D array of 300x300 cells filled with 0's

  for (size_t i = 0; i < scan->ranges.size(); ++i) // Iterate through each laser scan range
  {
    float range = scan->ranges[i];
    if (range < scan->range_min || range > scan->range_max)
    {
      continue; // Skip invalid ranges
    }

    float angle = scan->angle_min + i * scan->angle_increment; // Calculate the angle of the current laser scan point
    int x = static_cast<int>((range * std::cos(angle)) / RESOLUTION) + ARRAY_WIDTH / 2; // Convert to 2D array coordinates, centering the robot in the array
    int y = static_cast<int>((range * std::sin(angle)) / RESOLUTION) + ARRAY_HEIGHT / 2;

    if (x >= 0 && x < ARRAY_WIDTH && y >= 0 && y < ARRAY_HEIGHT)
    {
      array_2d[y][x] = MAX_COST; // Mark occupied cells with 100
    }
    else
    {
      continue;
    }
    // If cell is occupied, inflate it
    for (int di = -INFLATION_RADIUS/RESOLUTION; di <= INFLATION_RADIUS/RESOLUTION; ++di) // Inflate in y direction, adding: -1 , 0 , 1
    {
      for (int dj = -INFLATION_RADIUS/RESOLUTION; dj <= INFLATION_RADIUS/RESOLUTION; ++dj) // Inflate in x direction, adding: -1 , 0 , 1
      {
        int ni = y + di;
        int nj = x + dj;
        if (ni >= 0 && ni < ARRAY_HEIGHT && nj >= 0 && nj < ARRAY_WIDTH) // Check if inflation is within bounds
        {
          double euclidean_distance = std::sqrt(std::pow(ni - y, 2) + std::pow(nj - x, 2)); // Calculate straight line distance
          if (euclidean_distance <= INFLATION_RADIUS/RESOLUTION)
          {
            array_2d[ni][nj] = std::max<int>(array_2d[ni][nj], MAX_COST * (1.0 - (euclidean_distance / (INFLATION_RADIUS/RESOLUTION)))); // Inflate cell value with linear decay
          }
        }
      }
    }
  }
  
  nav_msgs::msg::OccupancyGrid costmap_msg;
  costmap_msg.header = scan->header;
  costmap_msg.info.resolution = RESOLUTION;
  costmap_msg.info.width = ARRAY_WIDTH;
  costmap_msg.info.height = ARRAY_HEIGHT;
  costmap_msg.info.origin.position.x = - (ARRAY_WIDTH * RESOLUTION) / 2.0;
  costmap_msg.info.origin.position.y = - (ARRAY_HEIGHT * RESOLUTION) / 2.0;

  costmap_msg.data.resize(ARRAY_WIDTH * ARRAY_HEIGHT);
  for (int i = 0; i < ARRAY_HEIGHT; ++i){
    for (int j = 0; j < ARRAY_WIDTH; ++j){
      costmap_msg.data[i * ARRAY_WIDTH + j] = array_2d[i][j];
    }
  }

  occupancy_grid_pub_->publish(costmap_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}