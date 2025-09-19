#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

namespace robot
{

// ------------------- Supporting Structures -------------------

// 2D grid index
struct CellIndex
{
  int x;
  int y;
 
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
 
  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }
 
  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};
 
// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};
 
// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h
 
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};
 
// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);
    
    // Public methods
    nav_msgs::msg::Path planPath(const nav_msgs::msg::OccupancyGrid::SharedPtr map,
                                const geometry_msgs::msg::PointStamped& goal,
                                const nav_msgs::msg::Odometry::SharedPtr odom);

  private:
    rclcpp::Logger logger_;
    
    // Map parameters
    int map_width_, map_height_;
    double map_resolution_;
    double map_origin_x_, map_origin_y_;
    
    // A* algorithm methods
    std::vector<geometry_msgs::msg::PoseStamped> aStar(const std::vector<std::vector<signed char>>& map,
                                                      const CellIndex& start, const CellIndex& goal);
    double heuristic(const CellIndex& a, const CellIndex& b);
    double gScore(const CellIndex& current, const CellIndex& neighbor);
    std::vector<CellIndex> getNeighbors(const CellIndex& cell, const std::vector<std::vector<signed char>>& map);
    bool isValidCell(const CellIndex& cell, const std::vector<std::vector<signed char>>& map);
    bool isObstacle(const CellIndex& cell, const std::vector<std::vector<signed char>>& map);
    
    // Coordinate conversion
    CellIndex worldToGrid(double world_x, double world_y) const;
    geometry_msgs::msg::PoseStamped gridToWorld(const CellIndex& cell) const;
    CellIndex odomToGrid(const nav_msgs::msg::Odometry::SharedPtr odom) const;
    CellIndex pointToGrid(const geometry_msgs::msg::PointStamped& point) const;
    
    // Path smoothing
    std::vector<geometry_msgs::msg::PoseStamped> smoothPath(const std::vector<geometry_msgs::msg::PoseStamped>& path);
};

}  

#endif  
