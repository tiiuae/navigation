#ifndef NAVIGATION_ASTAR_PLANNER_HPP
#define NAVIGATION_ASTAR_PLANNER_HPP

#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <octomap/octomap.h>
#include <queue>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <rclcpp/logger.hpp>

namespace navigation
{

enum TreeValue
{
  OCCUPIED = 0,
  FREE     = 1
};

enum PlanningResult
{
  COMPLETE = 0,
  GOAL_REACHED,
  INCOMPLETE,
  GOAL_IN_OBSTACLE,
  FAILURE
};


struct Node
{
  octomap::OcTreeKey key;
  float              total_cost;
  float              cum_dist;
  float              goal_dist;

  bool operator==(const Node &other) const;
  bool operator!=(const Node &other) const;
  bool operator<(const Node &other) const;
  bool operator<=(const Node &other) const;
};

struct CostComparator
{
  bool operator()(const Node &n1, const Node &n2) const;
};

struct LeafComparator
{
  bool operator()(const std::pair<octomap::OcTree::iterator, float> &l1, const std::pair<octomap::OcTree::iterator, float> &l2) const;
};

struct HashFunction
{
  bool operator()(const Node &n) const;
};

class AstarPlanner {

public:
  AstarPlanner(float safe_obstacle_distance, float euclidean_distance_cutoff, float planning_tree_resolution, float distance_penalty, float greedy_penalty,
               float min_altitude, float max_altitude, float timeout_threshold, float max_waypoint_distance, bool unknown_is_occupied,
               const rclcpp::Logger &logger);

private:
  float          safe_obstacle_distance;
  float          euclidean_distance_cutoff;
  float          planning_tree_resolution;
  float          distance_penalty;
  float          greedy_penalty;
  float          timeout_threshold;
  float          max_waypoint_distance;
  float          min_altitude;
  float          max_altitude;
  bool           unknown_is_occupied;
  rclcpp::Logger logger_;

public:
  std::pair<std::vector<octomap::point3d>, PlanningResult> findPath(
      const octomap::point3d &start_coord, const octomap::point3d &goal_coord, std::shared_ptr<octomap::OcTree> mapping_tree, float timeout,
      std::function<void(const octomap::OcTree &)> visualizeTree,
      std::function<void(const std::unordered_set<Node, HashFunction> &, const std::unordered_set<Node, HashFunction> &, const octomap::OcTree &)>
          visualizeExpansions);

private:
  const std::vector<std::vector<int>> EXPANSION_DIRECTIONS = {{-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 0}, {-1, 0, 1}, {-1, 1, -1},
                                                              {-1, 1, 0},   {-1, 1, 1},  {0, -1, -1}, {0, -1, 0},  {0, -1, 1}, {0, 0, -1}, {0, 0, 1},
                                                              {0, 1, -1},   {0, 1, 0},   {0, 1, 1},   {1, -1, -1}, {1, -1, 0}, {1, -1, 1}, {1, 0, -1},
                                                              {1, 0, 0},    {1, 0, 1},   {1, 1, -1},  {1, 1, 0},   {1, 1, 1}};
  float                               getNodeDepth(const octomap::OcTreeKey &key, octomap::OcTree &tree);

  std::vector<octomap::OcTreeKey> getNeighborhood(const octomap::OcTreeKey &key, octomap::OcTree &tree);

  octomap::OcTreeKey expand(const octomap::OcTreeKey &key, const std::vector<int> &direction);

  float distEuclidean(const octomap::point3d &p1, const octomap::point3d &p2);

  float distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, octomap::OcTree &tree);

  bool freeStraightPath(const octomap::point3d p1, const octomap::point3d p2, octomap::OcTree &tree);

  std::vector<octomap::OcTreeKey> backtrackPathKeys(const Node &start, const Node &end, std::unordered_map<Node, Node, HashFunction> &parent_map);

  std::vector<octomap::point3d> keysToCoords(std::vector<octomap::OcTreeKey> keys, octomap::OcTree &tree);

  DynamicEDTOctomap euclideanDistanceTransform(std::shared_ptr<octomap::OcTree> tree);

  std::shared_ptr<octomap::OcTree> createPlanningTree(std::shared_ptr<octomap::OcTree> tree, float resolution);

  std::vector<octomap::point3d> createEscapeTunnel(const std::shared_ptr<octomap::OcTree> mapping_tree, const std::shared_ptr<octomap::OcTree> planning_tree,
                                                   const octomap::point3d &start);

  std::pair<octomap::point3d, bool> generateTemporaryGoal(const octomap::point3d &start, const octomap::point3d &goal, octomap::OcTree &tree);

  std::vector<octomap::point3d> filterPath(const std::vector<octomap::point3d> &waypoints, octomap::OcTree &tree);

  std::vector<octomap::point3d> prepareOutputPath(const std::vector<octomap::OcTreeKey> &keys, octomap::OcTree &tree);

  /* geometry_msgs::msg::Quaternion yawToQuaternionMsg(const float &yaw); */
};

}  // namespace navigation
#endif
