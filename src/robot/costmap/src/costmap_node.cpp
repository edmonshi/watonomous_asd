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
  const int GRID_WIDTH = 300;
  const int GRID_HEIGHT = 300;
  const double RESOLUTION = 0.1; // Each cell is 0.1m x 0.1m, which makes the grid 30m x 30m (30x30 squares in foxglove)
  const int8_t MAX_COST = 100; // Cost value for occupied cells
  const double INFLATION_RADIUS = 5.0; // Inflate obstacles by 5 cells (0.5m)

  grid.resize(GRID_HEIGHT, std::vector<int8_t>(GRID_WIDTH, 0)); // Create a 2D grid of 300x300 cells filled with 0's

  for (size_t i = 0; i < scan->ranges.size(); ++i) // Iterate through each laser scan range
  {
    float range = scan->ranges[i];
    if (range < scan->range_min || range > scan->range_max)
    {
      continue; // Skip invalid ranges
    }

    float angle = scan->angle_min + i * scan->angle_increment; // Calculate the angle of the current laser scan point
    int x = static_cast<int>((range * std::cos(angle)) / RESOLUTION) + GRID_WIDTH / 2; // Convert to grid coordinates, centering the robot in the grid
    int y = static_cast<int>((range * std::sin(angle)) / RESOLUTION) + GRID_HEIGHT / 2;

    if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT)
    {
      grid[y][x] = MAX_COST; // Mark occupied cells with 100
    }
    else
    {
      continue;
    }
    // If cell is occupied, inflate it
    for (int di = -INFLATION_RADIUS; di <= INFLATION_RADIUS; ++di) // Inflate in y direction, adding: -1 , 0 , 1
    {
      for (int dj = -INFLATION_RADIUS; dj <= INFLATION_RADIUS; ++dj) // Inflate in x direction, adding: -1 , 0 , 1
      {
        int ni = y + di;
        int nj = x + dj;
        if (ni >= 0 && ni < GRID_HEIGHT && nj >= 0 && nj < GRID_WIDTH) // Check if inflation is within bounds
        {
          double euclidean_distance = std::sqrt(di * di + dj * dj) * RESOLUTION; // Calculate straight line distance
          if (euclidean_distance <= INFLATION_RADIUS)
          {
            grid[ni][nj] = std::max<int>(grid[ni][nj], MAX_COST * (1.0 - (euclidean_distance / INFLATION_RADIUS))); // Inflate cell value with linear decay
          }
        }
      }
    }
  }
  
  nav_msgs::msg::OccupancyGrid occupancy_grid;
  occupancy_grid.header.stamp = scan->header.stamp;
  occupancy_grid.header.frame_id = scan->header.frame_id;
  occupancy_grid.info.resolution = RESOLUTION; 
  occupancy_grid.info.width = GRID_WIDTH;
  occupancy_grid.info.height = GRID_HEIGHT;
  occupancy_grid.info.origin.position.x = - (GRID_WIDTH * RESOLUTION) / 2.0; 
  occupancy_grid.info.origin.position.y = - (GRID_HEIGHT * RESOLUTION) / 2.0;
  
  occupancy_grid.data.resize(GRID_WIDTH * GRID_HEIGHT);
  for (int i = 0; i < GRID_HEIGHT; ++i){
    for (int j = 0; j < GRID_WIDTH; ++j){
      occupancy_grid.data[i * GRID_WIDTH + j] = grid[i][j];
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