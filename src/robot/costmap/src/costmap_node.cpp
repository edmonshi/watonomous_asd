#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar",
      10,
      std::bind(&robot::CostmapCore::laserCallback, &costmap_, std::placeholders::_1));

  occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/costmap",
      10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  std::vector<std::vector<int8_t>> grid;
  const int grid_width = 300;
  const int grid_height = 300;
  const int resolution = 0.1;     // Each cell is 0.1m x 0.1m, which makes the grid 30m x 30m (30x30 squares in foxglove)
  const int inflation_radius = 1; // Inflate obstacles by 1 cell (0.1m)

  grid.resize(grid_height, std::vector<int8_t>(grid_width, 0)); // Create a 2D grid of 300x300 cells filled with 0's

  for (size_t i = 0; i < scan->ranges.size(); ++i)
  {
    float range = scan->ranges[i];
    if (range < scan->range_min || range > scan->range_max)
    {
      continue; // Skip invalid ranges
    }

    float angle = scan->angle_min + i * scan->angle_increment;
    int x = static_cast<int>((range * std::cos(angle)) / resolution) + grid_width / 2;
    int y = static_cast<int>((range * std::sin(angle)) / resolution) + grid_height / 2;

    if (x >= 0 && x < grid_width && y >= 0 && y < grid_height)
    {
      grid[y][x] = 100; // Mark occupied cells with 100
    }
    else
    {
      continue;
    }
    // If cell is occupied, inflate it
    for (int di = -inflation_radius; di <= inflation_radius; ++di) // Inflate in y direction, adding: -1 , 0 , 1
    {
      for (int dj = -inflation_radius; dj <= inflation_radius; ++dj) // Inflate in x direction, adding: -1 , 0 , 1
      {
        int ni = y + di;
        int nj = x + dj;
        if (ni >= 0 && ni < grid_height && nj >= 0 && nj < grid_width) // Check if inflation is within bounds
        {
          double euclidean_distance = std::sqrt(di * di + dj * dj) * resolution;
          if (euclidean_distance <= inflation_radius)
          {
            grid[ni][nj] = std::max<int>(grid[ni][nj], 100 * (1.0 - (euclidean_distance / inflation_radius))); // Inflate cell value based on distance
          }
        }
      }
    }
  }
  
  nav_msgs::msg::OccupancyGrid occupancy_grid;
  occupancy_grid.header.stamp = scan->header.stamp;
  occupancy_grid.header.frame_id = scan->header.frame_id;
  occupancy_grid.info.resolution = resolution; 
  occupancy_grid.info.width = grid_width;
  occupancy_grid.info.height = grid_height;
  occupancy_grid.info.origin.position.x = - (grid_width * resolution) / 2.0; 
  occupancy_grid.info.origin.position.y = - (grid_height * resolution) / 2.0;
  
  occupancy_grid.data.resize(grid_width * grid_height);
  for (int i = 0; i < grid_height; ++i){
    for (int j = 0; j < grid_width; ++j){
      occupancy_grid.data[i * grid_width + j] = grid[i][j];
    }
  }

  occupancy_grid_pub_->publish(occupancy_grid);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}