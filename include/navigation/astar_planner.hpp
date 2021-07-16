#ifndef NAVIGATION_ASTAR_PLANNER_HPP
#define NAVIGATION_ASTAR_PLANNER_HPP

#define VISUALIZE

#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <memory>
#include <octomap/octomap.h>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace navigation
{

enum TreeValue
{
  OCCUPIED = 0,
  FREE     = 1
};

struct Expansion
{
  octomap::OcTreeKey key;
  double             total_cost;
  double             cum_dist;
  double             goal_dist;

  bool operator==(const Expansion &other) const;
  bool operator!=(const Expansion &other) const;
  bool operator<(const Expansion &other) const;
};

struct CostComparator
{
  bool operator()(const Expansion &n1, const Expansion &n2) const;
};

struct LeafComparator
{
  bool operator()(const std::pair<octomap::OcTree::iterator, double> &l1, const std::pair<octomap::OcTree::iterator, double> &l2) const;
};

struct HashFunction
{
  bool operator()(const Expansion &n) const;
};

class AstarPlanner {

public:
  AstarPlanner(double safe_obstacle_distance, double euclidean_distance_cutoff, double planning_tree_resolution, double distance_penalty, double greedy_penalty,
               double vertical_penalty, double edf_penalty, bool unknown_is_occupied, float navigation_tolerance, double max_waypoint_distance,
               double planning_timeout);

private:
  double                        safe_obstacle_distance;
  double                        euclidean_distance_cutoff;
  double                        planning_tree_resolution;
  double                        distance_penalty;
  double                        greedy_penalty;
  double                        vertical_penalty;
  double                        edf_penalty;
  double                        max_waypoint_distance;
  bool                          unknown_is_occupied;
  float                         navigation_tolerance;
  double                        planning_timeout;
  octomap::OcTree *             binary_tree = NULL;
  std::vector<octomap::point3d> tunnel;
  DynamicEDTOctomap *           edf_;

public:
  // return a series of waypoints + true if the goal is actually reached / false
  // if only partial path is found
  std::pair<std::vector<octomap::point3d>, bool> findPath(
      const octomap::point3d &start, const octomap::point3d &goal, std::function<void(const octomap::OcTree &)> visualizeTree,
      std::function<void(const std::set<Expansion, CostComparator> &, const octomap::OcTree &)> visualizeExpansions, bool visualize);

  std::vector<octomap::point3d> postprocessPath(const std::vector<octomap::point3d> &waypoints);

  /**
   * @brief Compute the Euclidean distance between two 3D points
   *
   * @param p1
   * @param p2
   *
   * @return Euclidean distance in meters
   */
  static double distEuclidean(const octomap::point3d &p1, const octomap::point3d &p2);

  /**
   * @brief Compute the Euclidean distance between two tree nodes identified by
   * keys
   *
   * @param k1
   * @param k2
   * @param tree
   *
   * @return Euclidean distance in meters
   */
  static double distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, octomap::OcTree &tree);

  static double distPenalizeVertical(const octomap::point3d &p1, const octomap::point3d &p2, double vertical_penalty);

  static double distPenalizeVertical(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, octomap::OcTree &tree, double vertical_penalty);

  double edfCost(const octomap::point3d &p);

  void generatePlanningTree(std::shared_ptr<octomap::OcTree> mapping_tree, const octomap::point3d &start, double resolution);

  bool inMap(const octomap::point3d &p);

  bool isFree(const octomap::point3d &p);

  octomap::point3d nearestFreeCoord(const octomap::point3d &p, const octomap::point3d &uav_pos);

  std::pair<octomap::point3d, bool> generateTemporaryGoal(const octomap::point3d &start, const octomap::point3d &goal);

private:
  const std::vector<octomap::point3d> EXPANSION_DIRECTIONS = {{0, 0, 1}, {0, 0, -1}, {0, 1, 0}, {0, -1, 0}, {1, 0, 0}, {-1, 0, 0}};

  /**
   * @brief Compute the depth of a tree node at given key
   *
   * @param key of the node to be evaluated
   *
   * @return depth of the node (by default, the tree depth goes up to 16 for the
   * finest resolution)
   */
  double getExpansionDepth(const octomap::OcTreeKey &key, octomap::OcTree &tree);

  /**
   * @brief Return the 6 neighboring cells of the given key. If the neighbor is
   * NULL, it will not be returned
   *
   * @param key of the node to be evaluated
   * @param tree
   *
   * @return vector containing keys of neighboring nodes
   */
  std::vector<octomap::OcTreeKey> getNeighborhood(const octomap::OcTreeKey &key, const octomap::OcTree &tree);

  /**
   * @brief Search the tree in a given direction
   *
   * @param key of the node to be evaluated
   * @param direction 3D direction vector
   *
   * @return key of the next tree node in this direction (will make larger steps
   * if the tree depth is low at the current key)
   */
  octomap::OcTreeKey expand(const octomap::OcTreeKey &key, const octomap::point3d &direction, const octomap::OcTree &tree);

  /**
   * @brief Check if two points can be connected by a straight line of free
   * cells
   *
   * @param p1
   * @param p2
   * @param tree
   * @param max_distance_threshold
   *
   * @return true if the line only contains free cells and does not exceed the
   * given threshold
   */
  bool freeStraightPath(const octomap::point3d p1, const octomap::point3d p2, double max_distance_threshold = 1e6);

  /**
   * @brief Trace a path from Expansion @start to Expansion @end
   * NOTE: requires the parent map to be consistent (@start is required to have
   * a parent, and the subsequent parentage tracking is required to contain
   * @end)
   *
   * @param start
   * @param end
   *
   * @return vector of keys, ordered from end to start
   */
  std::vector<octomap::OcTreeKey> backtrackPathKeys(const Expansion &start, const Expansion &end,
                                                    std::unordered_map<Expansion, Expansion, HashFunction> &parent_map);

  /**
   * @brief Convert a vector of keys to a vector of 3D points
   *
   * @param keys
   * @param tree
   *
   * @return vector of 3D points
   */
  std::vector<octomap::point3d> keysToCoords(std::vector<octomap::OcTreeKey> keys, octomap::OcTree &tree);

  /**
   * @brief Perform an Euclidean distance transform across the entire tree
   *
   * @param euclidean_distance_cutoff min distance from obstacles to stop the
   * transform
   * @param unknown_is_occupied treat unknown cells as occupied
   *
   * @return Euclidean distance transform of the octree
   */
  DynamicEDTOctomap *euclideanDistanceTransform(std::shared_ptr<octomap::OcTree> tree);
};

}  // namespace navigation
#endif
