#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <navigation/astar_planner.hpp>

namespace navigation
{

/* Node class implementation and related free functions //{ */

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

//}

bool LeafComparator::operator()(const std::pair<octomap::OcTree::iterator, float> &l1, const std::pair<octomap::OcTree::iterator, float> &l2) const {
  return l1.second < l2.second;
}

/* AstarPlanner class implementation //{ */

/* AstarPlanner constructor //{ */
AstarPlanner::AstarPlanner(float safe_obstacle_distance, float euclidean_distance_cutoff, float planning_tree_resolution, float distance_penalty,
                           float greedy_penalty, float min_altitude, float max_altitude, float ground_cutoff, float timeout_threshold,
                           float max_waypoint_distance, bool unknown_is_occupied, const rclcpp::Logger &logger)
    : logger_(logger) {
  this->safe_obstacle_distance    = safe_obstacle_distance;
  this->euclidean_distance_cutoff = euclidean_distance_cutoff;
  this->planning_tree_resolution  = planning_tree_resolution;
  this->distance_penalty          = distance_penalty;
  this->greedy_penalty            = greedy_penalty;
  this->min_altitude              = min_altitude;
  this->max_altitude              = max_altitude;
  this->ground_cutoff             = ground_cutoff;
  this->timeout_threshold         = timeout_threshold;
  this->max_waypoint_distance     = max_waypoint_distance;
  this->unknown_is_occupied       = unknown_is_occupied;
}
//}

/* findPath //{ */

std::pair<std::vector<octomap::point3d>, PlanningResult> AstarPlanner::findPath(
    const octomap::point3d &start_coord, const octomap::point3d &goal_coord, std::shared_ptr<octomap::OcTree> mapping_tree, float timeout,
    std::function<void(const std::shared_ptr<octomap::OcTree> &)> visualizeTree,
    std::function<void(const std::unordered_set<Node, HashFunction> &, const std::unordered_set<Node, HashFunction> &,
                       const std::shared_ptr<octomap::OcTree> &)>
        visualizeExpansions) {

  RCLCPP_INFO(logger_, "[Astar]: Astar: start [%.2f, %.2f, %.2f]", start_coord.x(), start_coord.y(), start_coord.z());
  RCLCPP_INFO(logger_, "[Astar]: Astar: goal [%.2f, %.2f, %.2f]", goal_coord.x(), goal_coord.y(), goal_coord.z());

  auto time_start = std::chrono::high_resolution_clock::now();

  this->timeout_threshold = timeout;

  auto time_start_planning_tree = std::chrono::high_resolution_clock::now();
  /* auto tree_with_tunnel         = createPlanningTree(mapping_tree, start_coord, planning_tree_resolution); */
  auto planning_tree = createPlanningTree(mapping_tree, planning_tree_resolution);

  if (planning_tree->size() < 1) {
    RCLCPP_INFO(logger_, "[Astar]: could not create a planning tree");
    return {std::vector<octomap::point3d>(), FAILURE};
  }

  RCLCPP_INFO(logger_, "[Astar]: the planning tree took %.2f s to create",
              std::chrono::duration<float>(std::chrono::high_resolution_clock::now() - time_start_planning_tree).count());
  visualizeTree(planning_tree);

  /* check if planning start is within altitude bounds //{ */
  if (start_coord.z() < min_altitude) {
    RCLCPP_INFO(logger_, "[Astar]: Start is below minimum altitude, creating a temporary goal to force vertical movement");
    octomap::point3d temp_goal;
    temp_goal.x() = start_coord.x();
    temp_goal.y() = start_coord.y();
    temp_goal.z() = min_altitude + planning_tree_resolution;

    std::vector<octomap::point3d> vertical_path;
    vertical_path.push_back(start_coord);
    vertical_path.push_back(temp_goal);
    return {vertical_path, INCOMPLETE};
  }

  if (start_coord.z() > max_altitude) {
    RCLCPP_INFO(logger_, "[Astar]: Start is above maximum altitude, creating a temporary goal to force vertical movement");
    octomap::point3d temp_goal;
    temp_goal.x() = start_coord.x();
    temp_goal.y() = start_coord.y();
    temp_goal.z() = max_altitude - planning_tree_resolution;

    std::vector<octomap::point3d> vertical_path;
    vertical_path.push_back(start_coord);
    vertical_path.push_back(temp_goal);
    return {vertical_path, INCOMPLETE};
  }
  //}

  /* check if planning start is in octomap //{ */
  const auto start_query = planning_tree->search(start_coord);

  if (start_query == nullptr) {

    RCLCPP_INFO(logger_, "[Astar]: Start is outside of map, creating a temporary goal to force vertical movement");
    octomap::point3d temp_goal;
    temp_goal.x() = start_coord.x();
    temp_goal.y() = start_coord.y();
    temp_goal.z() = start_coord.z() + planning_tree_resolution;

    if (temp_goal.z() > max_altitude) {
      RCLCPP_INFO(logger_, "[Astar]: capping at max altitude");
      temp_goal.z() = max_altitude;
    }
    if (temp_goal.z() < min_altitude) {
      RCLCPP_INFO(logger_, "[Astar]: capping at min altitude");
      temp_goal.z() = min_altitude;
    }

    RCLCPP_INFO(logger_, "[Astar]: Generated a temporary goal: [%.2f, %.2f, %.2f]", start_coord.x(), start_coord.y(), start_coord.z());

    std::vector<octomap::point3d> vertical_path;
    vertical_path.push_back(start_coord);
    vertical_path.push_back(temp_goal);
    vertical_path.push_back(start_coord);
    return {vertical_path, INCOMPLETE};
  } else if (planning_tree->isNodeOccupied(start_query)) {
    RCLCPP_INFO(logger_, "[Astar]: Start is inside an inflated obstacle, searching for an escape path");

    std::vector<octomap::point3d> tunnel = createEscapeTunnel(mapping_tree, planning_tree, start_coord);

    // can we get to safety by moving sideways?
    if (!tunnel.empty()) {
      RCLCPP_INFO(logger_, "[Astar]: Escape tunnel created'");

      std::vector<octomap::point3d> path_to_safety;
      path_to_safety.push_back(start_coord);
      path_to_safety.push_back(tunnel.back());
      return {path_to_safety, INCOMPLETE};
    } else {
      // nearest obstacle is above or below the drone, move vertically
      tunnel = createVerticalTunnel(mapping_tree, start_coord);
      return {tunnel, INCOMPLETE};
    }
  }
  //}


  const auto map_query     = planning_tree->search(goal_coord);
  auto       map_goal      = goal_coord;
  bool       original_goal = true;

  if (map_query == nullptr) {
    RCLCPP_INFO(logger_, "[Astar]: Goal is outside of map");
    const auto [temp_goal, vertical_priority] = generateTemporaryGoal(start_coord, goal_coord, planning_tree);
    RCLCPP_INFO(logger_, "[Astar]: Generated a temporary goal: [%.2f, %.2f, %.2f]", temp_goal.x(), temp_goal.y(), temp_goal.z());
    if (vertical_priority) {
      std::vector<octomap::point3d> vertical_path;
      vertical_path.push_back(start_coord);
      vertical_path.push_back(temp_goal);
      return {vertical_path, INCOMPLETE};
    } else {
      map_goal      = temp_goal;
      original_goal = false;
    }
  } else if (planning_tree->isNodeOccupied(map_query)) {
    RCLCPP_INFO(logger_, "[Astar]: Goal is inside an inflated obstacle");
    if (distEuclidean(map_goal, start_coord) <= 1 * safe_obstacle_distance) {
      RCLCPP_INFO(logger_, "[Astar]: Path special case, we cannot get closer");
      return {std::vector<octomap::point3d>(), GOAL_REACHED};
    }
  }

  std::priority_queue<Node, std::vector<Node>, CostComparator> open_heap;
  std::unordered_set<Node, HashFunction>                       open;
  std::unordered_set<Node, HashFunction>                       closed;
  std::unordered_map<Node, Node, HashFunction>                 parent_map;  // first = child, second = parent

  octomap::OcTreeKey start = planning_tree->coordToKey(start_coord);

  auto planning_start = planning_tree->keyToCoord(start);
  auto goal           = planning_tree->coordToKey(map_goal);

  if (distEuclidean(planning_start, map_goal) <= 2 * planning_tree_resolution) {

    RCLCPP_INFO(logger_, "[Astar]: Path special case, we are there");

    visualizeExpansions(open, closed, planning_tree);

    return {std::vector<octomap::point3d>(), GOAL_REACHED};
  }

  RCLCPP_INFO_STREAM(logger_, "[Astar]: Planning from: " << planning_start.x() << ", " << planning_start.y() << ", " << planning_start.z());
  RCLCPP_INFO_STREAM(logger_, "[Astar]: Planning to: " << map_goal.x() << ", " << map_goal.y() << ", " << map_goal.z());

  Node first;
  first.key        = start;
  first.cum_dist   = 0;
  first.goal_dist  = distEuclidean(start, goal, planning_tree);
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

    if (std::chrono::duration<float>(time_now - time_start).count() > timeout_threshold) {

      RCLCPP_INFO(logger_, "[Astar]: Planning timeout! Using current best node as goal.");
      auto path_keys = backtrackPathKeys(best_node == first ? best_node_greedy : best_node, first, parent_map);
      RCLCPP_INFO(logger_, "[Astar]: Path found. Length: %ld", path_keys.size());

      visualizeExpansions(open, closed, planning_tree);

      return {prepareOutputPath(path_keys, planning_tree, true), INCOMPLETE};
    }

    auto current_coord = planning_tree->keyToCoord(current.key);

    if (distEuclidean(current_coord, map_goal) <= 2 * planning_tree_resolution) {

      auto path_keys = backtrackPathKeys(current, first, parent_map);
      path_keys.push_back(planning_tree->coordToKey(map_goal));
      RCLCPP_INFO(logger_, "[Astar]: Path found. Length: %ld", path_keys.size());

      visualizeExpansions(open, closed, planning_tree);

      if (original_goal) {
        return {prepareOutputPath(path_keys, planning_tree, false), COMPLETE};
      }
      return {prepareOutputPath(path_keys, planning_tree, true), INCOMPLETE};
    }

    // expand
    auto neighbors = getNeighborhood(current.key, planning_tree);

    for (auto &nkey : neighbors) {

      Node n;
      n.key = nkey;

      auto closed_query = closed.find(n);
      auto open_query   = open.find(n);

      // in open map
      n.goal_dist  = distEuclidean(nkey, goal, planning_tree);
      n.cum_dist   = current.cum_dist + distEuclidean(current.key, nkey, planning_tree);
      n.total_cost = greedy_penalty * n.goal_dist + distance_penalty * n.cum_dist;

      if (closed_query == closed.end() && open_query == open.end()) {

        if (n <= best_node) {
          best_node = n;
        }

        if (n.goal_dist <= best_node_greedy.goal_dist) {
          best_node_greedy = n;
        }

        open_heap.push(n);
        open.insert(n);
        parent_map[n] = current;
      }
    }
  }

  visualizeExpansions(open, closed, planning_tree);

  if (best_node != first) {

    auto path_keys = backtrackPathKeys(best_node, first, parent_map);

    RCLCPP_INFO(logger_, "[Astar]: direct path does not exist, going to the 'best_node'");

    return {prepareOutputPath(path_keys, planning_tree, true), INCOMPLETE};
  }

  if (best_node_greedy != first) {

    auto path_keys = backtrackPathKeys(best_node_greedy, first, parent_map);

    RCLCPP_INFO(logger_, "[Astar]: direct path does not exist, going to the best_node_greedy'");

    return {prepareOutputPath(path_keys, planning_tree, true), INCOMPLETE};
  }


  RCLCPP_INFO(logger_, "[Astar]: PATH DOES NOT EXIST!");

  return {std::vector<octomap::point3d>(), FAILURE};
}
//}

/* getNeighborhood() //{ */

std::vector<octomap::OcTreeKey> AstarPlanner::getNeighborhood(const octomap::OcTreeKey &key, std::shared_ptr<octomap::OcTree> tree) {

  std::vector<octomap::OcTreeKey> neighbors;

  for (auto &d : EXPANSION_DIRECTIONS) {

    auto newkey    = expand(key, d);
    auto tree_node = tree->search(newkey);

    if (tree_node != NULL) {
      // free cell?
      if (!tree->isNodeOccupied(tree_node) && tree->keyToCoord(newkey).z() >= min_altitude && tree->keyToCoord(newkey).z() <= max_altitude) {
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

float AstarPlanner::distEuclidean(const octomap::point3d &p1, const octomap::point3d &p2) {

  return (float)(p1 - p2).norm();
}

float AstarPlanner::distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, std::shared_ptr<octomap::OcTree> tree) {

  double voxel_dist = sqrt(pow(k1.k[0] - k2.k[0], 2) + pow(k1.k[1] - k2.k[1], 2) + pow(k1.k[2] - k2.k[2], 2));
  double res        = voxel_dist * tree->getResolution();
  return (float)res;
}

//}

/* freeStraightPath() //{ */

bool AstarPlanner::freeStraightPath(const octomap::point3d p1, const octomap::point3d p2, std::shared_ptr<octomap::OcTree> tree) {

  if (tree->search(p1) == NULL) {
    return false;
  }

  if (tree->search(p2) == NULL) {
    return false;
  }

  octomap::KeyRay ray;
  tree->computeRayKeys(p1, p2, ray);

  octomap::OcTreeKey p2_key = tree->coordToKey(p2);

  for (const auto &k : ray) {
    auto query = tree->search(k);
    if (query == NULL || query->getOccupancy() >= tree->getOccupancyThres()) {
      return false;
    }

    if (k == p2_key) {
      return true;
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

std::vector<octomap::point3d> AstarPlanner::keysToCoords(std::vector<octomap::OcTreeKey> keys, std::shared_ptr<octomap::OcTree> tree) {

  std::vector<octomap::point3d> coords;

  for (auto &k : keys) {
    coords.push_back(tree->keyToCoord(k));
  }

  return coords;
}

//}

/* euclideanDistanceTransform() //{ */

DynamicEDTOctomap AstarPlanner::euclideanDistanceTransform(std::shared_ptr<octomap::OcTree> tree) {
  double x, y, z;

  std::shared_ptr<octomap::OcTree> temp_tree = std::make_shared<octomap::OcTree>(tree->getResolution());
  for (auto it = tree->begin(); it != tree->end(); it++) {
    if (it.getZ() <= ground_cutoff) {
      temp_tree->setNodeValue(it.getCoordinate(), TreeValue::FREE);
    } else {
      temp_tree->setNodeValue(it.getCoordinate(), it->getValue());
    }
  }

  temp_tree->getMetricMin(x, y, z);
  octomap::point3d metric_min(x, y, z);

  temp_tree->getMetricMax(x, y, z);
  octomap::point3d metric_max(x, y, z);

  DynamicEDTOctomap edf(euclidean_distance_cutoff, temp_tree.get(), metric_min, metric_max, unknown_is_occupied);
  edf.update();

  return edf;
}
//}

/* createPlanningTree() //{ */

std::shared_ptr<octomap::OcTree> AstarPlanner::createPlanningTree(std::shared_ptr<octomap::OcTree> tree, float resolution) {

  auto edf = euclideanDistanceTransform(tree);


  std::shared_ptr<octomap::OcTree> binary_tree = std::make_shared<octomap::OcTree>(resolution);

  tree->expand();

  for (auto it = tree->begin(); it != tree->end(); it++) {
    if (edf.getDistance(it.getCoordinate()) <= safe_obstacle_distance) {
      binary_tree->setNodeValue(it.getCoordinate(), TreeValue::OCCUPIED);  // obstacle or close to obstacle
    } else {
      binary_tree->setNodeValue(it.getCoordinate(), TreeValue::FREE);  // free and safe
    }
  }

  tree->prune();
  binary_tree->prune();

  return binary_tree;
}

//}

/* createEscapeTunnel() //{ */

std::vector<octomap::point3d> AstarPlanner::createEscapeTunnel(const std::shared_ptr<octomap::OcTree> mapping_tree,
                                                               const std::shared_ptr<octomap::OcTree> planning_tree, const octomap::point3d &start) {

  std::vector<octomap::point3d> tunnel;

  octomap::point3d     current_coords = start;
  octomap::OcTreeNode *query          = planning_tree->search(current_coords);

  RCLCPP_INFO(logger_, "[Astar]: Creating escape tunnel");

  auto edf = euclideanDistanceTransform(mapping_tree);

  // tunnel out of expanded walls
  int iter1 = 0;
  while (rclcpp::ok() && query != NULL) {

    RCLCPP_INFO(logger_, "[Astar]: tunnelling through: %.2f, %.2f, %.2f", current_coords.x(), current_coords.y(), current_coords.z());

    if (iter1++ > 100) {
      RCLCPP_INFO(logger_, "[Astar]: Tunnelling aborted!");
      return {};
    }

    tunnel.push_back(current_coords);


    float            obstacle_dist;
    octomap::point3d closest_obstacle;

    edf.getDistanceAndClosestObstacle(current_coords, obstacle_dist, closest_obstacle);
    octomap::point3d dir_away_from_obstacle = current_coords - closest_obstacle;
    dir_away_from_obstacle.z()              = 0;

    if (!planning_tree->isNodeCollapsible(query) && obstacle_dist > safe_obstacle_distance) {
      RCLCPP_INFO(logger_, "[Astar]: tunnelling DONE");
      break;
    }

    current_coords += dir_away_from_obstacle.normalized() * float(planning_tree->getResolution());

    int iter2 = 0;

    while (planning_tree->search(current_coords) == query) {

      if (iter2++ > 100) {
        RCLCPP_INFO(logger_, "[Astar]: Tunnelling aborted!");
        return {};
      }

      current_coords += dir_away_from_obstacle.normalized() * float(planning_tree->getResolution());
    }

    query = planning_tree->search(current_coords);
  }

  return tunnel;
}

//}

/* createVerticalTunnel() //{ */

std::vector<octomap::point3d> AstarPlanner::createVerticalTunnel(const std::shared_ptr<octomap::OcTree> mapping_tree, const octomap::point3d &start) {

  std::vector<octomap::point3d> tunnel;

  RCLCPP_INFO(logger_, "[Astar]: Creating vertical tunnel");


  auto edf = euclideanDistanceTransform(mapping_tree);

  float            obstacle_dist;
  octomap::point3d closest_obstacle;

  edf.getDistanceAndClosestObstacle(start, obstacle_dist, closest_obstacle);

  if (closest_obstacle.z() + planning_tree_resolution < start.z()) {
    RCLCPP_INFO(logger_, "[Astar]: Obstacle is below vehicle, moving up");
    tunnel.push_back(start);
    octomap::point3d end;
    end.x() = start.x();
    end.y() = start.y();
    end.z() = std::min(closest_obstacle.z() + safe_obstacle_distance, max_altitude);
    tunnel.push_back(start);
    tunnel.push_back(end);
    return tunnel;
  }

  if (closest_obstacle.z() - planning_tree_resolution > start.z()) {
    RCLCPP_INFO(logger_, "[Astar]: Obstacle is above vehicle, moving up");
    tunnel.push_back(start);
    octomap::point3d end;
    end.x() = start.x();
    end.y() = start.y();
    end.z() = std::max(closest_obstacle.z() - safe_obstacle_distance, min_altitude);
    tunnel.push_back(start);
    tunnel.push_back(end);
    return tunnel;
  }

  RCLCPP_ERROR(logger_, "[Astar]: Vertical tunnel not found");
  return tunnel;
}

//}

/* filterPath() //{ */

std::vector<octomap::point3d> AstarPlanner::filterPath(const std::vector<octomap::point3d> &waypoints, std::shared_ptr<octomap::OcTree> tree,
                                                       bool append_endpoint) {

  if (waypoints.size() < 3) {
    RCLCPP_INFO(logger_, "[Astar]: Not enough points for filtering!");
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

  if (append_endpoint) {
    filtered.push_back(waypoints.back());
  }
  //}

  return filtered;
}
//}

/* prepareOutputPath() //{ */

std::vector<octomap::point3d> AstarPlanner::prepareOutputPath(const std::vector<octomap::OcTreeKey> &keys, std::shared_ptr<octomap::OcTree> tree,
                                                              bool append_endpoint) {
  auto waypoints = keysToCoords(keys, tree);
  auto processed = filterPath(waypoints, tree, append_endpoint);

  return processed;
}  // namespace navigation
//}

/* generateTemporaryGoal() //{ */

std::pair<octomap::point3d, bool> AstarPlanner::generateTemporaryGoal(const octomap::point3d &start, const octomap::point3d &goal,
                                                                      std::shared_ptr<octomap::OcTree> tree) {

  bool             vertical_priority = false;
  octomap::point3d temp_goal;

  // check if it is necessary to change altitude and acquire more of map
  if (std::abs(goal.z() - start.z()) >= planning_tree_resolution) {
    vertical_priority = true;
    RCLCPP_INFO(logger_, "[Astar]: give priority to vertical motion");
    temp_goal.x() = start.x();
    temp_goal.y() = start.y();
    temp_goal.z() = goal.z();  // scan new layers of octomap if needed

    if (temp_goal.z() > max_altitude) {
      RCLCPP_INFO(logger_, "[Astar]: capping at max altitude");
      temp_goal.z() = max_altitude;
    }
    if (temp_goal.z() < min_altitude) {
      RCLCPP_INFO(logger_, "[Astar]: capping at min altitude");
      temp_goal.z() = min_altitude;
    }
    return {temp_goal, vertical_priority};
  }

  // try to explore unknown cells
  std::set<std::pair<octomap::OcTree::iterator, float>, LeafComparator> leafs;

  RCLCPP_INFO(logger_, "[Astar]: Sorting octree leafs");
  for (auto it = tree->begin_leafs(); it != tree->end_leafs(); it++) {

    if (tree->isNodeOccupied(*it)) {
      continue;
    }

    if (it.getZ() > max_altitude || it.getZ() < min_altitude) {
      continue;
    }

    leafs.insert({it, distEuclidean(it.getCoordinate(), goal)});
  }

  // sort free nodes on the map edge by their distance from goal
  if (!leafs.empty()) {
    // select the closest point
    RCLCPP_INFO(logger_, "[Astar]: Selected closest leaf as temporary goal");
    return {leafs.begin()->first.getCoordinate(), vertical_priority};
  }

  // solution that is only good for smaller obstacles
  octomap::KeyRay ray;
  tree->computeRayKeys(start, goal, ray);

  for (auto &k : ray) {
    if (tree->search(k) != NULL && !tree->isNodeOccupied(tree->search(k))) {
      temp_goal = tree->keyToCoord(k);
    }
  }

  RCLCPP_INFO(logger_, "[Astar]: No valid leaves found. Using raycast to produce temporary goal");

  return {temp_goal, vertical_priority};
}
//}

//}

}  // namespace navigation
