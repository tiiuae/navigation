#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <navigation/astar_planner.hpp>

namespace navigation
{

bool Node::operator==(const Node &other) const {
  return key == other.key;
}

bool Node::operator!=(const Node &other) const {
  return key != other.key;
}

bool Node::operator<(const Node &other) const {

  if (total_cost == other.total_cost) {
    return goal_dist < other.goal_dist;
  }

  return total_cost < other.total_cost;
}

bool Node::operator<=(const Node &other) const {

  if (total_cost == other.total_cost) {
    return goal_dist <= other.goal_dist;
  }

  return total_cost <= other.total_cost;
}

bool CostComparator::operator()(const Node &n1, const Node &n2) const {

  if (n1.total_cost == n2.total_cost) {
    return n1.goal_dist > n2.goal_dist;
  }

  return n1.total_cost > n2.total_cost;
}

bool HashFunction::operator()(const Node &n) const {
  using std::hash;
  return ((hash<int>()(n.key.k[0]) ^ (hash<int>()(n.key.k[1]) << 1)) >> 1) ^ (hash<int>()(n.key.k[2]) << 1);
}

bool LeafComparator::operator()(const std::pair<octomap::OcTree::iterator, double> &l1, const std::pair<octomap::OcTree::iterator, double> &l2) const {
  return l1.second < l2.second;
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
  this->unknown_is_occupied       = unknown_is_occupied;
}
//}

/* findPath //{ */

std::pair<std::vector<octomap::point3d>, PlanningResult> AstarPlanner::findPath(
    const octomap::point3d &start_coord, const octomap::point3d &goal_coord, std::shared_ptr<octomap::OcTree> mapping_tree, double timeout,
    std::function<void(const octomap::OcTree &)> visualizeTree,
    std::function<void(const std::unordered_set<Node, HashFunction> &, const std::unordered_set<Node, HashFunction> &, const octomap::OcTree &)>
        visualizeExpansions) {

  printf("[Astar]: Astar: start [%.2f, %.2f, %.2f]\n", start_coord.x(), start_coord.y(), start_coord.z());
  printf("[Astar]: Astar: goal [%.2f, %.2f, %.2f]\n", goal_coord.x(), goal_coord.y(), goal_coord.z());

  auto time_start = std::chrono::high_resolution_clock::now();

  this->timeout_threshold = timeout;

  auto time_start_planning_tree = std::chrono::high_resolution_clock::now();
  auto tree_with_tunnel         = createPlanningTree(mapping_tree, start_coord, planning_tree_resolution);
  printf("[Astar]: the planning tree took %.2f s to create\n",
         std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - time_start_planning_tree).count());

  if (!tree_with_tunnel) {
    printf("[Astar]: could not create a planning tree\n");
    return {std::vector<octomap::point3d>(), FAILURE};
  }

  visualizeTree((*tree_with_tunnel).first);

  auto tree   = tree_with_tunnel.value().first;
  auto tunnel = tree_with_tunnel.value().second;

  auto map_goal      = goal_coord;
  auto map_query     = tree_with_tunnel->first.search(goal_coord);
  bool original_goal = true;

  if (map_query == NULL) {
    printf("[Astar]: Goal is outside of map\n");
    auto temp_goal = generateTemporaryGoal(start_coord, goal_coord, tree);
    printf("[Astar]: Generated a temporary goal: [%.2f, %.2f, %.2f]\n", temp_goal.first.x(), temp_goal.first.y(), temp_goal.first.z());
    if (temp_goal.second) {
      std::vector<octomap::point3d> vertical_path;
      vertical_path.push_back(start_coord);
      vertical_path.push_back(temp_goal.first);
      return {vertical_path, INCOMPLETE};
    } else {
      map_goal      = temp_goal.first;
      original_goal = false;
    }
  } else if (map_query->getValue() == TreeValue::OCCUPIED) {
    printf("[Astar]: Goal is inside an inflated obstacle\n");
    if (distEuclidean(map_goal, start_coord) <= 1 * safe_obstacle_distance) {
      printf("[Astar]: Path special case, we cannot get closer\n");
      return {std::vector<octomap::point3d>(), GOAL_REACHED};
    }
  }

  std::priority_queue<Node, std::vector<Node>, CostComparator> open_heap;
  std::unordered_set<Node, HashFunction>                       open;
  std::unordered_set<Node, HashFunction>                       closed;
  std::unordered_map<Node, Node, HashFunction>                 parent_map;  // first = child, second = parent

  octomap::OcTreeKey start;
  if (tunnel.empty()) {
    start = tree.coordToKey(start_coord);
  } else {
    start = tree.coordToKey(tunnel.back());
  }

  auto planning_start = tree.keyToCoord(start);
  auto goal           = tree.coordToKey(map_goal);

  if (distEuclidean(planning_start, map_goal) <= 2 * planning_tree_resolution) {

    printf("[Astar]: Path special case, we are there\n");

    visualizeExpansions(open, closed, tree);

    return {std::vector<octomap::point3d>(), GOAL_REACHED};
  }

  std::cout << "[Astar]: Planning from: " << planning_start.x() << ", " << planning_start.y() << ", " << planning_start.z() << "\n";
  std::cout << "[Astar]: Planning to: " << map_goal.x() << ", " << map_goal.y() << ", " << map_goal.z() << "\n";

  Node first;
  first.key        = start;
  first.cum_dist   = 0;
  first.goal_dist  = distEuclidean(start, goal, tree);
  first.total_cost = first.cum_dist + first.goal_dist;
  open_heap.push(first);
  open.insert(first);

  Node best_node        = first;
  Node best_node_greedy = first;

  Node last_closed;

  while (!open.empty() && rclcpp::ok()) {

    Node current = open_heap.top();
    open_heap.pop();
    open.erase(current);
    closed.insert(current);

    last_closed = current;

    auto time_now = std::chrono::high_resolution_clock::now();

    if (std::chrono::duration<double>(time_now - time_start).count() > timeout_threshold) {

      printf("[Astar]: Planning timeout! Using current best node as goal.\n");
      auto path_keys = backtrackPathKeys(best_node == first ? best_node_greedy : best_node, first, parent_map);
      printf("[Astar]: Path found. Length: %ld\n", path_keys.size());

      visualizeExpansions(open, closed, tree);

      return {prepareOutputPath(path_keys, tree), INCOMPLETE};
    }

    auto current_coord = tree.keyToCoord(current.key);

    if (distEuclidean(current_coord, map_goal) <= 2 * planning_tree_resolution) {

      auto path_keys = backtrackPathKeys(current, first, parent_map);
      path_keys.push_back(tree.coordToKey(map_goal));
      printf("[Astar]: Path found. Length: %ld\n", path_keys.size());

      visualizeExpansions(open, closed, tree);

      if (original_goal) {
        return {prepareOutputPath(path_keys, tree), COMPLETE};
      }
      return {prepareOutputPath(path_keys, tree), INCOMPLETE};
    }

    // expand
    auto neighbors = getNeighborhood(current.key, tree);

    for (auto &nkey : neighbors) {

      Node n;
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

      auto closed_query = closed.find(n);
      auto open_query   = open.find(n);

      // in open map
      n.goal_dist  = distEuclidean(nkey, goal, tree);
      n.cum_dist   = current.cum_dist + distEuclidean(current.key, nkey, tree);
      n.total_cost = greedy_penalty * n.goal_dist + distance_penalty * n.cum_dist;

      if (closed_query == closed.end() && open_query == open.end()) {

        if (n <= best_node) {
          best_node = n;
        }

        if (n.goal_dist <= best_node_greedy.goal_dist) {
          best_node_greedy = n;
        }

        n.goal_dist  = distPenalizeVertical(nkey, goal, *binary_tree, vertical_penalty);
        n.cum_dist   = current.cum_dist + distPenalizeVertical(current.key, nkey, *binary_tree, vertical_penalty);
        n.total_cost = distance_penalty * n.cum_dist + greedy_penalty * n.goal_dist + edfCost(current_coord);
        open.insert(n);
        parent_map[n] = current;
      }
    }
  }

  visualizeExpansions(open, closed, tree);

  if (best_node != first) {

    auto path_keys = backtrackPathKeys(best_node, first, parent_map);

    printf("[Astar]: direct path does not exist, going to the 'best_node'\n");

    return {prepareOutputPath(path_keys, tree), INCOMPLETE};
  }

  if (best_node_greedy != first) {

    auto path_keys = backtrackPathKeys(best_node_greedy, first, parent_map);

    printf("[Astar]: direct path does not exist, going to the best_node_greedy'\n");

    return {prepareOutputPath(path_keys, tree), INCOMPLETE};
  }

  if(!tunnel.empty()){
    std::vector<octomap::point3d> path_to_safety;
    path_to_safety.push_back(start_coord);
    path_to_safety.push_back(tunnel.back());
    printf("[Astar]: path does not exist, escaping no-go zone'\n");
    return {path_to_safety, INCOMPLETE};
  }

  printf("[Astar]: PATH DOES NOT EXIST!\n");

  return {std::vector<octomap::point3d>(), FAILURE};
}
//}

/* getNeighborhood() //{ */

std::vector<octomap::OcTreeKey> AstarPlanner::getNeighborhood(const octomap::OcTreeKey &key, octomap::OcTree &tree) {

  std::vector<octomap::OcTreeKey> neighbors;

  for (auto &d : EXPANSION_DIRECTIONS) {

    auto newkey    = expand(key, d);
    auto tree_node = tree.search(newkey);

    if (tree_node != NULL) {
      // free cell?
      if (tree_node->getValue() == TreeValue::FREE && tree.keyToCoord(newkey).z() >= min_altitude && tree.keyToCoord(newkey).z() <= max_altitude) {
        neighbors.push_back(newkey);
      }
    }
  }

  return neighbors;
}

//}

/* expand() //{ */

octomap::OcTreeKey AstarPlanner::expand(const octomap::OcTreeKey &key, const std::vector<int> &direction) {

  octomap::OcTreeKey k;

  k.k[0] = key.k[0] + direction[0];
  k.k[1] = key.k[1] + direction[1];
  k.k[2] = key.k[2] + direction[2];

  return k;
}

//}

/* distEuclidean() //{ */

double AstarPlanner::distEuclidean(const octomap::point3d &p1, const octomap::point3d &p2) {

  return (p1 - p2).norm();
}

double AstarPlanner::distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, octomap::OcTree &tree) {

  double voxel_dist = sqrt(pow(k1.k[0] - k2.k[0], 2) + pow(k1.k[1] - k2.k[1], 2) + pow(k1.k[2] - k2.k[2], 2));

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
  tree.computeRayKeys(p1, p2, ray);

  for (auto &k : ray) {

    auto tree_node = tree.search(k);

    if (tree_node == NULL) {
      // Path may exist, but goes through unknown cells
      return false;
    }

    if (tree_node->getValue() == TreeValue::OCCUPIED) {
      // Path goes through occupied cells
      return false;
    }
  }

  return true;
}

//}

/* backtrackPathKeys() //{ */

std::vector<octomap::OcTreeKey> AstarPlanner::backtrackPathKeys(const Node &from, const Node &to, std::unordered_map<Node, Node, HashFunction> &parent_map) {

  std::vector<octomap::OcTreeKey> keys;

  Node current = from;
  keys.push_back(current.key);

  while (current.key != to.key) {
    current = parent_map.find(current)->second;
    keys.push_back(current.key);
  };

  keys.push_back(to.key);

  // reverse order
  std::reverse(keys.begin(), keys.end());
  return keys;
}

//}

/* keysToCoords() //{ */

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

/* createPlanningTree() //{ */

  tunnel.clear();
  edf_        = euclideanDistanceTransform(mapping_tree);
  binary_tree = new octomap::OcTree(resolution);
  mapping_tree->expand();
  for (auto it = mapping_tree->begin(); it != mapping_tree->end(); it++) {
    if (edf_->getDistance(it.getCoordinate()) <= safe_obstacle_distance) {
      binary_tree->setNodeValue(it.getCoordinate(),
                                TreeValue::OCCUPIED);  // obstacle or close to obstacle
    } else {
      binary_tree.setNodeValue(it.getCoordinate(), TreeValue::FREE);  // free and safe
    }
  }

  std::vector<octomap::point3d> tunnel;

  octomap::point3d current_coords    = start;
  auto             binary_tree_query = binary_tree.search(current_coords);

  if (binary_tree_query != NULL && binary_tree_query->getValue() != TreeValue::FREE) {

    printf("[Astar]: start is inside of an inflated obstacle, tunneling out\n");

    // tunnel out of expanded walls

    int iter1 = 0;

    while (rclcpp::ok() && binary_tree_query != NULL && iter1++ <= 100) {

      if (iter1++ > 100) {
        return {};
      }

      tunnel.push_back(current_coords);
      binary_tree.setNodeValue(current_coords, TreeValue::FREE);

      float            obstacle_dist;
      octomap::point3d closest_obstacle;
      edf_->getDistanceAndClosestObstacle(current_coords, obstacle_dist, closest_obstacle);
      octomap::point3d dir_away_from_obstacle = current_coords - closest_obstacle;

      if (obstacle_dist >= safe_obstacle_distance) {
        printf("[Astar]: tunnel created with %d\n", int(tunnel.size()));
        break;
      }

      current_coords += dir_away_from_obstacle.normalized() * float(binary_tree.getResolution());

      int iter2 = 0;

      while (binary_tree.search(current_coords) == binary_tree_query) {

        if (iter2++ > 100) {
          return {};
        }

        current_coords += dir_away_from_obstacle.normalized() * float(binary_tree.getResolution());
      }

      binary_tree_query = binary_tree.search(current_coords);
    }
  }

  std::pair<octomap::OcTree, std::vector<octomap::point3d>> result = {binary_tree, tunnel};

  return result;
}

//}

/* filterPath() //{ */

std::vector<octomap::point3d> AstarPlanner::filterPath(const std::vector<octomap::point3d> &waypoints, octomap::OcTree &tree) {

  if (waypoints.size() < 3) {
    printf("[Astar]: Not enough points for filtering!\n");
    return waypoints;
  }

  /* removing obsolete points //{ */

  std::vector<octomap::point3d> filtered;

  filtered.push_back(waypoints.front());

  size_t k = 2;

  while (k < waypoints.size()) {

    if (!freeStraightPath(filtered.back(), waypoints[k], tree)) {
      filtered.push_back(waypoints[k - 1]);
    }

    k++;
  }

  filtered.push_back(waypoints.back());
  //}

  return filtered;
}
//}

/* prepareOutputPath() //{ */

std::vector<octomap::point3d> AstarPlanner::prepareOutputPath(const std::vector<octomap::OcTreeKey> &keys, octomap::OcTree &tree) {
  auto waypoints = keysToCoords(keys, tree);
  auto processed = filterPath(waypoints, tree);

  return processed;
}  // namespace navigation
//}

/* generateTemporaryGoal() //{ */

std::pair<octomap::point3d, bool> AstarPlanner::generateTemporaryGoal(const octomap::point3d &start, const octomap::point3d &goal, octomap::OcTree &tree) {

  bool             vertical_priority = false;
  octomap::point3d temp_goal;

  // check if it is necessary to change altitude and acquire more of map
  if (std::abs(goal.z() - start.z()) > planning_tree_resolution / 2.0) {
    vertical_priority = true;
    printf("[Astar]: give priority to vertical motion\n");
    temp_goal.x() = start.x();
    temp_goal.y() = start.y();

    double extra_motion = goal.z() - start.z();
    extra_motion        = (extra_motion / std::abs(extra_motion)) * planning_tree_resolution;

    temp_goal.z() = goal.z() + extra_motion;

    if (temp_goal.z() > max_altitude) {
      printf("[Astar]: capping at max altitude\n");
      temp_goal.z() = max_altitude;
    }
    if (temp_goal.z() < min_altitude) {
      printf("[Astar]: capping at min altitude\n");
      temp_goal.z() = min_altitude;
    }
    return {temp_goal, vertical_priority};
  }

  // try to explore unknown cells
  std::set<std::pair<octomap::OcTree::iterator, double>, LeafComparator> leafs;

  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); it++) {

    if (it->getValue() == TreeValue::OCCUPIED) {
      continue;
    }

    auto k = it.getKey();
    k.k[2] += 1;

    if (tree.search(k) == NULL) {
      continue;
    }

    k.k[2] -= 2;

    if (tree.search(k) == NULL) {
      continue;
    }

    leafs.insert({it, distEuclidean(it.getCoordinate(), goal)});
  }

  // sort free nodes on the map edge by their distance from goal
  if (!leafs.empty()) {
    // select the closest point
    return {leafs.begin()->first.getCoordinate(), vertical_priority};
  }

  // solution that is only good for smaller obstacles
  octomap::KeyRay ray;
  tree.computeRayKeys(start, goal, ray);

  for (auto &k : ray) {
    auto coords = tree.keyToCoord(k);
    if (tree.search(coords) != NULL && tree.search(coords)->getValue() == TreeValue::FREE) {
      temp_goal = coords;
    }
  }

  return {temp_goal, vertical_priority};
}
//}

}  // namespace navigation
