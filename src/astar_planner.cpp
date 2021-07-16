#include <algorithm>
#include <chrono>
#include <navigation/astar_planner.hpp>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap_types.h>
#include <rclcpp/rclcpp.hpp>

namespace navigation
{

bool Expansion::operator==(const Expansion &other) const {
  return key == other.key;
}
bool Expansion::operator!=(const Expansion &other) const {
  return key != other.key;
}
bool Expansion::operator<(const Expansion &other) const {
  if (total_cost == other.total_cost) {
    return goal_dist < other.goal_dist;
  }
  return total_cost < other.total_cost;
}

bool CostComparator::operator()(const Expansion &n1, const Expansion &n2) const {
  if (n1.total_cost == n2.total_cost) {
    return n1.goal_dist < n2.goal_dist;
  }
  return n1.total_cost < n2.total_cost;
}

bool LeafComparator::operator()(const std::pair<octomap::OcTree::iterator, double> &l1, const std::pair<octomap::OcTree::iterator, double> &l2) const {
  return l1.second < l2.second;
}

bool HashFunction::operator()(const Expansion &n) const {
  using std::hash;
  return ((hash<int>()(n.key.k[0]) ^ (hash<int>()(n.key.k[1]) << 1)) >> 1) ^ (hash<int>()(n.key.k[2]) << 1);
}

/* AstarPlanner constructor //{ */
AstarPlanner::AstarPlanner(double safe_obstacle_distance, double euclidean_distance_cutoff, double planning_tree_resolution, double distance_penalty,
                           double greedy_penalty, double vertical_penalty, double edf_penalty, bool unknown_is_occupied, float navigation_tolerance,
                           double max_waypoint_distance, double planning_timeout) {
  this->safe_obstacle_distance    = safe_obstacle_distance;
  this->euclidean_distance_cutoff = euclidean_distance_cutoff;
  this->planning_tree_resolution  = planning_tree_resolution;
  this->distance_penalty          = distance_penalty;
  this->greedy_penalty            = greedy_penalty;
  this->vertical_penalty          = vertical_penalty;
  this->edf_penalty               = edf_penalty;
  this->unknown_is_occupied       = unknown_is_occupied;
  this->navigation_tolerance      = navigation_tolerance;
  this->max_waypoint_distance     = max_waypoint_distance;
  this->planning_timeout          = planning_timeout;
}
//}

/* findPath //{ */
std::pair<std::vector<octomap::point3d>, bool> AstarPlanner::findPath(
    const octomap::point3d &start_coord, const octomap::point3d &goal_coord, std::function<void(const octomap::OcTree &)> visualizeTree,
    std::function<void(const std::set<Expansion, CostComparator> &, const octomap::OcTree &)> visualizeExpansions, bool visualize) {

  octomap::point3d shifted_goal  = goal_coord;
  octomap::point3d goal_to_start = (start_coord - goal_coord).normalized() * binary_tree->getResolution();
  while (rclcpp::ok() && binary_tree->search(shifted_goal) == NULL) {
    // goal is outside of map -> move closer to UAV
    std::cout << "Goal is outside of map -> move closer to UAV" << std::endl;
    shifted_goal += goal_to_start;
    goal_to_start = (start_coord - shifted_goal).normalized() * binary_tree->getResolution();
  }

  if (visualize) {
    visualizeTree(*binary_tree);
  }

  octomap::OcTreeKey start;
  if (tunnel.empty()) {
    start = binary_tree->coordToKey(start_coord);
  } else {
    start = binary_tree->coordToKey(tunnel.back());
  }

  std::set<Expansion, CostComparator>                    open;
  std::unordered_set<Expansion, HashFunction>            closed;
  std::unordered_map<Expansion, Expansion, HashFunction> parent_map;  // first = child, second = parent
  std::set<Expansion, CostComparator>                    reachable_leafs;

  auto planning_start = binary_tree->keyToCoord(start);
  auto goal           = binary_tree->coordToKey(shifted_goal);

  std::cout << "Start planning from: " << planning_start.x() << ", " << planning_start.y() << ", " << planning_start.z() << std::endl;

  Expansion first;
  first.key        = start;
  first.cum_dist   = 0;
  first.goal_dist  = distPenalizeVertical(start, goal, *binary_tree, vertical_penalty);
  first.total_cost = first.cum_dist + first.goal_dist;
  open.insert(first);

  auto start_time = std::chrono::high_resolution_clock::now();

  while (!open.empty() && rclcpp::ok()) {
    if (visualize) {
      visualizeExpansions(open, *binary_tree);
    }

    auto current = *open.begin();
    open.erase(current);
    closed.insert(current);

    double elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count() / 1000.0;
    /* std::cout << "Elapsed time: " << elapsed_time << std::endl; */
    if (elapsed_time > planning_timeout) {
      std::cout << "PLANNING INTERRUPTED BY TIMEOUT" << std::endl;
      std::cout << "USING BEST CLOSED EXPANSION AS A TEMPORARY GOAL" << std::endl;
      Expansion best_expansion = *closed.begin();
      for (auto &e : closed) {
        if (e < best_expansion) {
          best_expansion = e;
        }
      }
      auto path_keys = backtrackPathKeys(best_expansion, first, parent_map);
      std::cout << "PARTIAL PATH FOUND! Length: " << path_keys.size() << std::endl;
      auto path_coords = keysToCoords(path_keys, *binary_tree);
      return {path_coords, false};
    }

    auto current_coord = binary_tree->keyToCoord(current.key);

    if (freeStraightPath(current_coord, shifted_goal)) {
      auto path_keys   = backtrackPathKeys(current, first, parent_map);
      auto path_coords = keysToCoords(path_keys, *binary_tree);
      path_coords.push_back(goal_coord);
      std::cout << "PATH FOUND! Length: " << path_coords.size() << std::endl;
      return {path_coords, true};
    }

    // expand
    auto neighbors = getNeighborhood(current.key, *binary_tree);

    for (auto &nkey : neighbors) {
      Expansion n;
      n.key = nkey;
      // check if open
      auto open_query = open.find(n);
      if (open_query != open.end()) {
        // in open map

        n.goal_dist  = distPenalizeVertical(nkey, goal, *binary_tree, vertical_penalty);
        n.cum_dist   = current.cum_dist + distPenalizeVertical(current.key, nkey, *binary_tree, vertical_penalty);
        n.total_cost = n.goal_dist + n.cum_dist + edfCost(current_coord);

        if (n < current) {
          // new path is better -> update
          Expansion open_node = *open_query;
          open.erase(open_node);
          parent_map.erase(open_node);

          open.insert(n);
          parent_map[n] = current;
          continue;
        }
      }

      // check if closed
      auto closed_query = closed.find(n);
      if (closed_query == closed.end()) {
        // not in closed map -> open

        n.goal_dist  = distPenalizeVertical(nkey, goal, *binary_tree, vertical_penalty);
        n.cum_dist   = current.cum_dist + distPenalizeVertical(current.key, nkey, *binary_tree, vertical_penalty);
        n.total_cost = distance_penalty * n.cum_dist + greedy_penalty * n.goal_dist + edfCost(current_coord);
        open.insert(n);
        parent_map[n] = current;
      }
    }
  }
  // unknown cells are in the path
  // use best reachable node
  Expansion best_expansion = *closed.begin();
  for (auto &e : closed) {
    if (e < best_expansion) {
      best_expansion = e;
    }
  }
  std::cout << "Trying to force exploration of unknown cells" << std::endl;
  auto path_keys = backtrackPathKeys(best_expansion, first, parent_map);
  std::cout << "PATH FOUND! Length: " << path_keys.size() << std::endl;
  return {keysToCoords(path_keys, *binary_tree), false};
}
//}

/* postprocessPath //{ */
std::vector<octomap::point3d> AstarPlanner::postprocessPath(const std::vector<octomap::point3d> &waypoints) {

  if (waypoints.size() < 2) {
    std::cout << "Not enough points for postprocessing!" << std::endl;
    return waypoints;
  }

  std::vector<octomap::point3d> padded         = waypoints;
  size_t                        waypoints_size = waypoints.size();

  /* padding with additional points if the distances exceed threshold //{ */
  for (size_t i = 1; i < waypoints_size; i++) {
    if (distEuclidean(padded[i], padded[i - 1]) > max_waypoint_distance) {
      auto direction = (padded[i] - padded[i - 1]).normalized() * max_waypoint_distance;
      padded.insert(padded.begin() + i, padded[i - 1] + direction);
      waypoints_size++;
    }
  }
  //}

  if (padded.size() < 3) {
    std::cout << "Not enough points for postprocessing!" << std::endl;
    return padded;
  }

  std::vector<octomap::point3d> filtered;

  /* removing obsolete points //{ */
  filtered.push_back(padded.front());
  size_t i = 2;
  while (i < padded.size()) {
    if (!freeStraightPath(filtered.back(), padded[i], max_waypoint_distance)) {
      filtered.push_back(padded[i - 1]);
    }
    i++;
  }
  filtered.push_back(padded.back());
  //}

  return filtered;
}  // namespace navigation
//}

/* getExpansionDepth //{ */
double AstarPlanner::getExpansionDepth(const octomap::OcTreeKey &key, octomap::OcTree &tree) {
  for (auto it = tree.begin(); it != tree.end(); it++) {
    if (it.getKey() == key) {
      return it.getDepth();
    }
  }
  return tree.getTreeDepth();
}
//}

/* getNeighborhood //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::getNeighborhood(const octomap::OcTreeKey &key, const octomap::OcTree &tree) {
  std::vector<octomap::OcTreeKey> neighbors;

  for (auto &d : EXPANSION_DIRECTIONS) {
    auto newkey    = expand(key, d, tree);
    auto tree_node = tree.search(newkey);
    if (tree_node != NULL) {
      if (tree_node->getValue() == TreeValue::FREE) {
        // only apply to free cells
        neighbors.push_back(newkey);
      }
    }
  }

  return neighbors;
}
//}

/* expand //{ */
octomap::OcTreeKey AstarPlanner::expand(const octomap::OcTreeKey &key, const octomap::point3d &direction, [[maybe_unused]] const octomap::OcTree &tree) {

  octomap::OcTreeKey k;
  k.k[0] = key.k[0] + direction.x();
  k.k[1] = key.k[1] + direction.y();
  k.k[2] = key.k[2] + direction.z();

  return k;
}
//}

/* distEuclidean //{ */
double AstarPlanner::distEuclidean(const octomap::point3d &p1, const octomap::point3d &p2) {
  return (p1 - p2).norm();
}

double AstarPlanner::distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, octomap::OcTree &tree) {
  double voxel_dist = std::sqrt(std::pow(k1.k[0] - k2.k[0], 2) + std::pow(k1.k[1] - k2.k[1], 2) + std::pow(k1.k[2] - k2.k[2], 2));
  return voxel_dist * tree.getResolution();
}
//}

/* distPenalizeVertical //{ */
double AstarPlanner::distPenalizeVertical(const octomap::point3d &p1, const octomap::point3d &p2, double vertical_penalty) {
  return std::sqrt(std::pow(p1.x() - p2.x(), 2) + std::pow(p1.y() - p2.y(), 2) + vertical_penalty * std::pow(p2.z() - p2.z(), 2));
}

double AstarPlanner::distPenalizeVertical(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, octomap::OcTree &tree, double vertical_penalty) {
  double voxel_dist = std::sqrt(std::pow(k1.k[0] - k2.k[0], 2) + std::pow(k1.k[1] - k2.k[1], 2) + vertical_penalty * std::pow(k1.k[2] - k2.k[2], 2));
  return voxel_dist * tree.getResolution();
}
//}

double AstarPlanner::edfCost(const octomap::point3d &p) {
  double cost = euclidean_distance_cutoff - edf_->getDistance(p);
  if (cost < 0.0) {
    return 0.0;
  }
  return cost * edf_penalty;
}

/* freeStraightPath //{ */
bool AstarPlanner::freeStraightPath(const octomap::point3d p1, const octomap::point3d p2, double max_waypoint_distance) {

  octomap::KeyRay ray;
  binary_tree->computeRayKeys(p1, p2, ray);
  for (auto &k : ray) {
    auto tree_node = binary_tree->search(k);
    if (tree_node == NULL) {
      // Path may exist, but goes through unknown cells
      return false;
    }
    if (tree_node->getValue() == TreeValue::OCCUPIED) {
      // Path goes through occupied cells
      return false;
    }
    if ((p1 - p2).norm() > max_waypoint_distance) {
      return false;
    }
  }
  return true;
}
//}

/* backtrackPathKeys //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::backtrackPathKeys(const Expansion &from, const Expansion &to,
                                                                std::unordered_map<Expansion, Expansion, HashFunction> &parent_map) {
  std::vector<octomap::OcTreeKey> keys;

  Expansion current = from;
  keys.push_back(current.key);

  while (rclcpp::ok() && current != to) {
    current = parent_map.find(current)->second;  // get parent
    keys.push_back(current.key);
  };

  // reverse order
  std::reverse(keys.begin(), keys.end());
  return keys;
}
//}

/* keysToCoords //{ */
std::vector<octomap::point3d> AstarPlanner::keysToCoords(std::vector<octomap::OcTreeKey> keys, octomap::OcTree &tree) {
  std::vector<octomap::point3d> coords;
  for (auto &k : keys) {
    coords.push_back(tree.keyToCoord(k));
  }
  return coords;
}
//}

/* euclideanDistanceTransform //{ */
DynamicEDTOctomap *AstarPlanner::euclideanDistanceTransform(std::shared_ptr<octomap::OcTree> tree) {
  double x, y, z;
  tree->getMetricMin(x, y, z);
  octomap::point3d metric_min(x, y, z);
  tree->getMetricMax(x, y, z);
  octomap::point3d   metric_max(x, y, z);
  DynamicEDTOctomap *edf = new DynamicEDTOctomap(euclidean_distance_cutoff, tree.get(), metric_min, metric_max, unknown_is_occupied);
  edf->update();
  return edf;
}
//}

/* generatePlanningTree//{ */
void AstarPlanner::generatePlanningTree(std::shared_ptr<octomap::OcTree> mapping_tree, const octomap::point3d &start, double resolution) {

  tunnel.clear();
  edf_        = euclideanDistanceTransform(mapping_tree);
  binary_tree = new octomap::OcTree(resolution);
  mapping_tree->expand();
  for (auto it = mapping_tree->begin(); it != mapping_tree->end(); it++) {
    if (edf_->getDistance(it.getCoordinate()) <= safe_obstacle_distance) {
      binary_tree->setNodeValue(it.getCoordinate(),
                                TreeValue::OCCUPIED);  // obstacle or close to obstacle
    } else {
      binary_tree->setNodeValue(it.getCoordinate(),
                                TreeValue::FREE);  // free and safe
    }
  }

  octomap::point3d current_coords    = start;
  auto             binary_tree_query = binary_tree->search(current_coords);
  if (binary_tree_query != NULL && binary_tree_query->getValue() != TreeValue::FREE) {
    std::cout << "Start is inside an inflated obstacle. Tunneling out..." << std::endl;
    // tunnel out of inflated obstacles
    while (rclcpp::ok() && binary_tree_query != NULL) {
      tunnel.push_back(current_coords);
      binary_tree->setNodeValue(current_coords, TreeValue::FREE);
      float            obstacle_dist;
      octomap::point3d closest_obstacle;
      edf_->getDistanceAndClosestObstacle(current_coords, obstacle_dist, closest_obstacle);
      octomap::point3d dir_away_from_obstacle = current_coords - closest_obstacle;
      dir_away_from_obstacle.z()              = 0.0f;
      if (obstacle_dist >= safe_obstacle_distance) {
        std::cout << "Tunnel created" << std::endl;
        break;
      }
      current_coords += dir_away_from_obstacle.normalized() * binary_tree->getResolution();
      while (rclcpp::ok() && binary_tree->search(current_coords) == binary_tree_query) {
        current_coords += dir_away_from_obstacle.normalized() * binary_tree->getResolution();
      }
      binary_tree_query = binary_tree->search(current_coords);
    }
  }
}
//}

/* nearestFreeCoord //{ */
octomap::point3d AstarPlanner::nearestFreeCoord(const octomap::point3d &p, const octomap::point3d &uav_pos) {
  if (binary_tree == NULL) {
    std::cout << "Planning tree not created!" << std::endl;
    return p;
  }
  auto query = binary_tree->search(p);
  if (query != NULL && query->getValue() == TreeValue::FREE) {
    return p;
  }
  auto neighbors = getNeighborhood(binary_tree->coordToKey(p), *binary_tree);
  for (auto &n : neighbors) {
    auto query = binary_tree->search(n);
    if (query != NULL && query->getValue() == TreeValue::FREE) {
      return binary_tree->keyToCoord(n);
    }
  }
  // no free neighbor -> try a point closer to UAV
  octomap::point3d dir_to_uav;
  dir_to_uav = (uav_pos - p).normalized() * binary_tree->getResolution();
  return nearestFreeCoord(p + dir_to_uav, uav_pos);
}
//}

/* generateTemporaryGoal //{ */
std::pair<octomap::point3d, bool> AstarPlanner::generateTemporaryGoal(const octomap::point3d &start, const octomap::point3d &goal) {
  if (binary_tree == NULL) {
    std::cout << "Planning tree not created!" << std::endl;
    return {start, false};
  }

  octomap::point3d temp_goal;

  // if the goal is ABOVE or BELOW the mapped area, do the vertical movement
  // before anything else
  float alt_diff = goal.z() - start.z();

  if (std::abs(alt_diff) > navigation_tolerance) {
    bool can_move_up = true;
    temp_goal.x()    = start.x();
    temp_goal.y()    = start.y();
    temp_goal.z()    = goal.z() + (std::abs(alt_diff) / alt_diff) * navigation_tolerance;

    octomap::KeyRay ray;
    binary_tree->computeRayKeys(start, temp_goal, ray);
    for (auto &k : ray) {
      auto query = binary_tree->search(k);
      if (query == NULL) {
        continue;
      }
      if (query->getValue() == TreeValue::OCCUPIED) {
        can_move_up = false;
        break;
      }
    }

    if (can_move_up) {
      return {temp_goal, true};
    }
  }

  // try to explore unknown cells
  std::set<std::pair<octomap::OcTree::iterator, double>, LeafComparator> leafs;
  for (auto it = binary_tree->begin_leafs(); it != binary_tree->end_leafs(); it++) {
    if (it->getValue() == TreeValue::OCCUPIED) {
      continue;
    }
    auto k = it.getKey();
    k.k[2] += 1;
    if (binary_tree->search(k) == NULL) {
      continue;
    }
    k.k[2] -= 2;
    if (binary_tree->search(k) == NULL) {
      continue;
    }

    /* good but slow //{ */
    /* bool            possibly_free_path = true; */
    /* octomap::KeyRay ray; */
    /* binary_tree->computeRayKeys(it.getCoordinate(), goal, ray); */
    /* for (auto &k : ray) { */
    /*   auto query = binary_tree->search(k); */
    /*   if (query == NULL) { */
    /*     continue; */
    /*   } */
    /*   if (query->getValue() == TreeValue::OCCUPIED) { */
    /*     possibly_free_path = false; */
    /*     break; */
    /*   } */
    /* } */
    /* if (possibly_free_path) { */
    /*   leafs.insert({it, distEuclidean(it.getCoordinate(), goal)}); */
    /* } */
    //}

    leafs.insert({it, distEuclidean(it.getCoordinate(), goal)});
  }
  // sort free nodes on the map edge by their distance from goal
  if (!leafs.empty()) {
    // select the closest point
    return {leafs.begin()->first.getCoordinate(), false};
  }

  // solution that is only good for smaller obstacles
  octomap::KeyRay ray;
  binary_tree->computeRayKeys(start, goal, ray);
  for (auto &k : ray) {
    auto coords = binary_tree->keyToCoord(k);
    if (isFree(coords)) {
      temp_goal = coords;
    }
  }
  return {temp_goal, false};
}
//}

/* inMap //{ */
bool AstarPlanner::inMap(const octomap::point3d &p) {
  if (binary_tree == NULL) {
    std::cout << "Planning tree not created!" << std::endl;
    return false;
  }
  return binary_tree->search(p) != NULL;
}
//}

/* isFree //{ */
bool AstarPlanner::isFree(const octomap::point3d &p) {
  if (binary_tree == NULL) {
    std::cout << "Planning tree not created!" << std::endl;
    return false;
  }
  auto query = binary_tree->search(p);
  return query != NULL && query->getValue() == TreeValue::FREE;
}
//}

}  // namespace navigation
