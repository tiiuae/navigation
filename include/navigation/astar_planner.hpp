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
  double             total_cost;
  double             cum_dist;
  double             goal_dist;

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
  bool operator()(const std::pair<octomap::OcTree::iterator, double> &l1, const std::pair<octomap::OcTree::iterator, double> &l2) const;
};

struct HashFunction
{
  bool operator()(const Node &n) const;
};

class AstarPlanner {

public:
  AstarPlanner(double safe_obstacle_distance, double euclidean_distance_cutoff, double planning_tree_resolution, double distance_penalty, double greedy_penalty,
               double vertical_penalty, double min_altitude, double max_altitude, double timeout_threshold, double max_waypoint_distance,
               bool unknown_is_occupied);

private:
  double safe_obstacle_distance;
  double euclidean_distance_cutoff;
  double planning_tree_resolution;
  double distance_penalty;
  double greedy_penalty;
  double vertical_penalty;
  double timeout_threshold;
  double max_waypoint_distance;
  double min_altitude;
  double max_altitude;
  bool   unknown_is_occupied;

public:
  std::pair<std::vector<octomap::point3d>, PlanningResult> findPath(
      const octomap::point3d &start_coord, const octomap::point3d &goal_coord, std::shared_ptr<octomap::OcTree> mapping_tree, double timeout,
      std::function<void(const octomap::OcTree &)> visualizeTree,
      std::function<void(const std::unordered_set<Node, HashFunction> &, const std::unordered_set<Node, HashFunction> &, const octomap::OcTree &)>
          visualizeExpansions);

private:
  const std::vector<std::vector<int>> EXPANSION_DIRECTIONS = {{-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 0}, {-1, 0, 1}, {-1, 1, -1},
                                                              {-1, 1, 0},   {-1, 1, 1},  {0, -1, -1}, {0, -1, 0},  {0, -1, 1}, {0, 0, -1}, {0, 0, 1},
                                                              {0, 1, -1},   {0, 1, 0},   {0, 1, 1},   {1, -1, -1}, {1, -1, 0}, {1, -1, 1}, {1, 0, -1},
                                                              {1, 0, 0},    {1, 0, 1},   {1, 1, -1},  {1, 1, 0},   {1, 1, 1}};
  double                              getNodeDepth(const octomap::OcTreeKey &key, octomap::OcTree &tree);

  std::vector<octomap::OcTreeKey> getNeighborhood(const octomap::OcTreeKey &key, octomap::OcTree &tree);

  octomap::OcTreeKey expand(const octomap::OcTreeKey &key, const std::vector<int> &direction);

  double distEuclidean(const octomap::point3d &p1, const octomap::point3d &p2);

  double distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, octomap::OcTree &tree);

  bool freeStraightPath(const octomap::point3d p1, const octomap::point3d p2, octomap::OcTree &tree);

  std::vector<octomap::OcTreeKey> backtrackPathKeys(const Node &start, const Node &end, std::unordered_map<Node, Node, HashFunction> &parent_map);

  std::vector<octomap::point3d> keysToCoords(std::vector<octomap::OcTreeKey> keys, octomap::OcTree &tree);

  DynamicEDTOctomap euclideanDistanceTransform(std::shared_ptr<octomap::OcTree> tree);

  std::optional<std::pair<octomap::OcTree, std::vector<octomap::point3d>>> createPlanningTree(std::shared_ptr<octomap::OcTree> tree,
                                                                                              const octomap::point3d &start, double resolution);

  std::pair<octomap::point3d, bool> generateTemporaryGoal(const octomap::point3d &start, const octomap::point3d &goal, octomap::OcTree &tree);

  std::vector<octomap::point3d> filterPath(const std::vector<octomap::point3d> &waypoints, octomap::OcTree &tree);

  std::vector<octomap::point3d> prepareOutputPath(const std::vector<octomap::OcTreeKey> &keys, octomap::OcTree &tree);

  /* geometry_msgs::msg::Quaternion yawToQuaternionMsg(const double &yaw); */
};

}  // namespace navigation
#endif
