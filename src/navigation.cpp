#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>

#include <eigen3/Eigen/Core>
#include <mutex>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/bounding_box_query.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>

#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>

#include <navigation/astar_planner.hpp>

using namespace std::placeholders;

namespace navigation {

enum status_t { IDLE = 0, PLANNING, MOVING };

std::string status_string[] = {"IDLE", "PLANNING", "MOVING"};

/* class Navigation //{ */
class Navigation : public rclcpp::Node {
public:
  Navigation(rclcpp::NodeOptions options);

private:
  // internal variables
  bool is_initialized_ = false;
  bool getting_octomap_ = false;
  bool getting_odometry_ = false;
  bool priorize_vertical_ = false;
  bool visualize_planner_ = true;
  bool show_unoccupied_ = false;
  std::string parent_frame_;

  octomap::point3d uav_pos_;
  octomap::point3d last_uav_pos_;
  std::mutex octree_mutex_;
  std::shared_ptr<octomap::OcTree> octree_;
  std::mutex status_mutex_;
  status_t status_ = IDLE;

  octomap::point3d goal_;
  octomap::point3d current_goal_;
  std::vector<octomap::point3d> waypoint_out_buffer_;
  std::vector<octomap::point3d> waypoint_in_buffer_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr execution_timer_;
  void navigationRoutine(void);

  // params
  double euclidean_distance_cutoff_;
  double safe_obstacle_distance_;
  double navigation_tolerance_;
  bool unknown_is_occupied_;
  double min_altitude_;
  double distance_penalty_;
  double greedy_penalty_;
  double vertical_penalty_;
  double planning_tree_resolution_;
  double max_waypoint_distance_;
  double planning_timeout_;

  // visualization params
  double tree_points_scale_;
  double field_points_scale_;
  double expansions_points_scale_;
  double path_points_scale_;
  double goal_points_scale_;

  // publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      field_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      binary_tree_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      expansion_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoints_publisher_;

  // subscribers
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr
      octomap_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr goto_subscriber_;

  // subscriber callbacks
  void octomapCallback(const octomap_msgs::msg::Octomap::UniquePtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg);
  void gotoCallback(const nav_msgs::msg::Path::UniquePtr msg);

  // services provided
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr goto_trigger_service_;

  // service callbacks
  bool gotoTriggerCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  bool goalReached(const octomap::point3d &goal, double dist_tolerance);
  nav_msgs::msg::Path
  waypointsToPathMsg(std::vector<octomap::point3d> waypoints);

  AstarPlanner *planner;

  // visualization
  void visualizeTree(const octomap::OcTree &tree);
  void visualizeExpansions(const std::set<Expansion, CostComparator> &open_list,
                           const octomap::OcTree &tree);
  void visualizePath(const std::vector<octomap::point3d> waypoints);
  void visualizeGoals(const std::vector<octomap::point3d> waypoints,
                      const octomap::point3d current_goal);

  std_msgs::msg::ColorRGBA generateColor(const double r, const double g,
                                         const double b, const double a);

  template <class T> bool parse_param(std::string param_name, T &param_dest);
};
//}

/* constructor //{ */
Navigation::Navigation(rclcpp::NodeOptions options)
    : Node("navigation", options) {

  RCLCPP_INFO(this->get_logger(), "[%s]: Initializing...", this->get_name());

  /* parse params from config file //{ */
  parse_param("euclidean_distance_cutoff", euclidean_distance_cutoff_);
  parse_param("safe_obstacle_distance", safe_obstacle_distance_);
  parse_param("unknown_is_occupied", unknown_is_occupied_);
  parse_param("navigation_tolerance", navigation_tolerance_);
  parse_param("min_altitude", min_altitude_);
  parse_param("distance_penalty", distance_penalty_);
  parse_param("greedy_penalty", greedy_penalty_);
  parse_param("vertical_penalty", vertical_penalty_);
  parse_param("planning_tree_resolution", planning_tree_resolution_);
  parse_param("max_waypoint_distance", max_waypoint_distance_);
  parse_param("planning_timeout", planning_timeout_);

  parse_param("visualize_planner", visualize_planner_);
  parse_param("show_unoccupied", show_unoccupied_);
  parse_param("tree_points_scale", tree_points_scale_);
  parse_param("field_points_scale", field_points_scale_);
  parse_param("expansions_points_scale", expansions_points_scale_);
  parse_param("path_points_scale", path_points_scale_);
  parse_param("goal_points_scale", goal_points_scale_);
  //}

  // publishers
  field_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "~/field_markers_out", 1);
  binary_tree_publisher_ =
      this->create_publisher<visualization_msgs::msg::Marker>(
          "~/binary_tree_markers_out", 1);
  expansion_publisher_ =
      this->create_publisher<visualization_msgs::msg::Marker>(
          "~/expansion_markers_out", 1);
  path_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "~/path_markers_out", 10);
  goal_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "~/goal_markers_out", 10);
  waypoints_publisher_ =
      this->create_publisher<nav_msgs::msg::Path>("~/waypoints_out", 10);
  status_publisher_ =
      this->create_publisher<std_msgs::msg::String>("~/status_out", 10);

  // subscribers
  octomap_subscriber_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "~/octomap_in", 1, std::bind(&Navigation::octomapCallback, this, _1));
  odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "~/odometry_in", 1, std::bind(&Navigation::odometryCallback, this, _1));
  goto_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
      "~/goto_in", 1, std::bind(&Navigation::gotoCallback, this, _1));

  // service handlers
  goto_trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/goto_trigger_in",
      std::bind(&Navigation::gotoTriggerCallback, this, _1, _2));

  // timers
  execution_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / 10.0),
      std::bind(&Navigation::navigationRoutine, this), callback_group_);

  planner = new AstarPlanner(
      safe_obstacle_distance_, euclidean_distance_cutoff_,
      planning_tree_resolution_, distance_penalty_, greedy_penalty_,
      vertical_penalty_, unknown_is_occupied_, navigation_tolerance_,
      max_waypoint_distance_, planning_timeout_);

  is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "[%s]: Initialized", this->get_name());
}
//}

/* octomapCallback //{ */
void Navigation::octomapCallback(
    const octomap_msgs::msg::Octomap::UniquePtr msg) {
  getting_octomap_ = true;
  parent_frame_ = msg->header.frame_id;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting octomap",
                   this->get_name());
  octomap_msgs::msg::Octomap map_msg = *msg;

  auto treePtr = octomap_msgs::fullMsgToMap(*msg);

  if (!treePtr) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Octomap message is empty!",
                this->get_name());
  } else {
    std::scoped_lock lock(octree_mutex_);
    octree_ = std::shared_ptr<octomap::OcTree>(
        dynamic_cast<octomap::OcTree *>(treePtr));
  }
}
//}

/* odometryCallback //{ */
void Navigation::odometryCallback(
    const nav_msgs::msg::Odometry::UniquePtr msg) {
  getting_odometry_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting odometry",
                   this->get_name());
  uav_pos_.x() = msg->pose.pose.position.x;
  uav_pos_.y() = msg->pose.pose.position.y;
  uav_pos_.z() = msg->pose.pose.position.z;
}
//}

/* gotoCallback //{ */
void Navigation::gotoCallback(const nav_msgs::msg::Path::UniquePtr msg) {
  RCLCPP_INFO(this->get_logger(), "[%s]: Recieved %ld waypoints",
              this->get_name(), msg->poses.size());

  waypoint_in_buffer_.clear();
  for (const auto &p : msg->poses) {
    octomap::point3d point;
    point.x() = p.pose.position.x;
    point.y() = p.pose.position.y;
    point.z() = p.pose.position.z;
    waypoint_in_buffer_.push_back(point);
  }

  RCLCPP_INFO(this->get_logger(), "[%s]: Waypoint buffer size: %ld",
              this->get_name(), waypoint_in_buffer_.size());
}
//}

/* gotoTriggerCallback //{ */
bool Navigation::gotoTriggerCallback(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request>
        request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (!is_initialized_) {
    response->message = "Goto rejected. Node not initialized";
    response->success = false;
    return false;
  }

  if (!getting_octomap_) {
    response->message = "Goto rejected. Octomap not received";
    response->success = false;
    return false;
  }

  if (status_ != IDLE) {
    response->message = "Goto rejected. Vehicle not IDLE";
    response->success = false;
    return false;
  }

  if (waypoint_in_buffer_.empty()) {
    response->message = "Goto rejected. No waypoint input provided";
    response->success = false;
    return false;
  }

  /* if (request->goal[2] < min_altitude_) { */
  /*   RCLCPP_WARN(this->get_logger(), */
  /*               "[%s]: Goal is below the minimum allowed altitude!", */
  /*               this->get_name()); */
  /*   RCLCPP_INFO(this->get_logger(), "[%s]: Goal shifted to altitude %.2f", */
  /*               this->get_name(), min_altitude_); */
  /*   ; */
  /* } */

  /* current_goal_.x() = request->goal[0]; */
  /* current_goal_.y() = request->goal[1]; */
  /* current_goal_.z() = std::max(request->goal[2], min_altitude_); */

  /* goal_.x() = request->goal[0]; */
  /* goal_.y() = request->goal[1]; */
  /* goal_.z() = std::max(request->goal[2], min_altitude_); */

  RCLCPP_INFO(this->get_logger(), "[%s]: Mission started", this->get_name());
  status_ = PLANNING;

  response->message = "Navigation goal set";
  response->success = true;
  return true;
}
//}

/* navigationRoutine //{ */
void Navigation::navigationRoutine(void) {

  if (is_initialized_ && getting_octomap_ && getting_odometry_) {

    std::scoped_lock lock(status_mutex_);

    switch (status_) {

    /* IDLE //{ */
    case IDLE: {
      break;
    }
      //}

    /* PLANNING //{ */
    case PLANNING: {

      if (waypoint_in_buffer_.empty()) {
        RCLCPP_INFO(this->get_logger(), "[%s]: All waypoints have been visited",
                    this->get_name());
        status_ = IDLE;
        break;
      }

      current_goal_ = waypoint_in_buffer_.front();
      waypoint_in_buffer_.erase(waypoint_in_buffer_.begin());
      RCLCPP_INFO(this->get_logger(),
                  "[%s]: Waypoint [%.2f, %.2f, %.2f] set as a next goal",
                  this->get_name(), current_goal_.x(), current_goal_.y(),
                  current_goal_.z());

      //{
      std::scoped_lock lock(octree_mutex_);

      if (octree_ == NULL || octree_->size() < 1) {
        RCLCPP_WARN(this->get_logger(),
                    "[%s]: Octomap is NULL or empty! Abort mission.",
                    this->get_name());
        status_ = IDLE;
        break;
      }

      visualizeGoals(waypoint_in_buffer_, current_goal_);

      RCLCPP_INFO(this->get_logger(), "[%s]: Planning started",
                  this->get_name());

      planner->generatePlanningTree(octree_, uav_pos_,
                                    planning_tree_resolution_);

      // check if goal is in map and if the map cell is unoccupied
      bool goal_in_map = planner->inMap(current_goal_);
      bool goal_valid = planner->isFree(current_goal_);

      if (goal_in_map && goal_valid) {

        auto waypoints = planner->findPath(
            uav_pos_, current_goal_, octree_,
            std::bind(&Navigation::visualizeTree, this, _1),
            std::bind(&Navigation::visualizeExpansions, this, _1, _2),
            visualize_planner_);
        if (waypoints.size() < 2) {
          RCLCPP_ERROR(this->get_logger(),
                       "[%s]: Path not found! Likely reason: bowl-shaped "
                       "obstacle larger than detection radius",
                       this->get_name());
          status_ = IDLE;
          break;
        }

        RCLCPP_INFO(this->get_logger(), "[%s]: Postprocessing path",
                    this->get_name());
        auto processed = planner->postprocessPath(waypoints);
        RCLCPP_INFO(this->get_logger(),
                    "[%s]: Transformed %ld waypoints to %ld waypoints",
                    this->get_name(), waypoints.size(), processed.size());
        waypoints = processed;

        waypoint_out_buffer_.clear();
        double dist = 0.0;
        waypoint_out_buffer_.push_back(waypoints[0]);
        for (size_t i = 1; i < waypoints.size(); i++) {
          waypoint_out_buffer_.push_back(waypoints[i]);
        }
        visualizePath(waypoints);
        auto waypoints_msg = waypointsToPathMsg(waypoint_out_buffer_);
        waypoints_publisher_->publish(waypoints_msg);
        current_goal_ = waypoint_out_buffer_.front();
        status_ = MOVING;
        break;
      }

      if (goal_in_map && !goal_valid) {
        RCLCPP_WARN(this->get_logger(),
                    "[%s]: Goal is in map, but the cell is unreachable",
                    this->get_name());
        RCLCPP_INFO(this->get_logger(),
                    "[%s]: A nearby replacement goal will be used",
                    this->get_name());

        auto new_goal = planner->nearestFreeCoord(current_goal_, uav_pos_);

        waypoint_in_buffer_.insert(waypoint_in_buffer_.begin(), new_goal);
        // first goal in input buffer is now valid
        // planning will be executed in the next cycle
        break;
      }

      if (!goal_in_map) {
        RCLCPP_INFO(this->get_logger(), "[%s]: Goal is outside of map",
                    this->get_name());
        auto temporary_goal =
            planner->generateTemporaryGoal(uav_pos_, current_goal_);
        auto new_goal = temporary_goal.first;
        priorize_vertical_ = temporary_goal.second;
        RCLCPP_INFO(this->get_logger(), "[%s]: Generated a temporary goal",

                    this->get_name());

        waypoint_in_buffer_.insert(waypoint_in_buffer_.begin(), new_goal);

        if (priorize_vertical_) {
          // move up or down into unknown without planning
          // ASSUME the environment does not change in the vertical direction
          // This could be improved by using a 3D lidar or an upward rangefinder
          priorize_vertical_ = false;
          waypoint_out_buffer_.push_back(current_goal_);
          current_goal_ = new_goal;
          status_ = MOVING;
          break;
        }
        break;
      }
      //}

      break;
    }
      //}

      /* MOVING //{ */
    case MOVING: {

      if (waypoint_out_buffer_.empty()) {
        RCLCPP_INFO(this->get_logger(), "[%s]: End of current segment reached",
                    this->get_name());
        status_ = PLANNING;
        break;
      }

      if (goalReached(current_goal_, navigation_tolerance_)) {
        waypoint_out_buffer_.erase(waypoint_out_buffer_.begin());
        current_goal_ = waypoint_out_buffer_.front();
        RCLCPP_INFO(this->get_logger(),
                    "[%s]: Moving to next waypoint: [%.2f, %.2f, %.2f]",
                    this->get_name(), current_goal_.x(), current_goal_.y(),
                    current_goal_.z());
        auto waypoints_msg = waypointsToPathMsg(waypoint_out_buffer_);
        waypoints_publisher_->publish(waypoints_msg);
        current_goal_ = waypoint_out_buffer_.front();
      }
      break;
    }
      //}
    }
  }

  std_msgs::msg::String msg;
  msg.data = status_string[status_];
  status_publisher_->publish(msg);
}
//}

/* goalReached //{ */
bool Navigation::goalReached(const octomap::point3d &goal,
                             double dist_tolerance) {
  const octomap::point3d uav_pos = {uav_pos_.x(), uav_pos_.y(), uav_pos_.z()};
  return AstarPlanner::distEuclidean(uav_pos, goal) < dist_tolerance;
}
//}

/* waypointsToPathMsg //{ */
nav_msgs::msg::Path
Navigation::waypointsToPathMsg(const std::vector<octomap::point3d> waypoints) {
  nav_msgs::msg::Path msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = parent_frame_;
  for (auto &wp : waypoints) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = wp.x();
    p.pose.position.y = wp.y();
    p.pose.position.z = wp.z();
    msg.poses.push_back(p);
  }
  return msg;
}
//}

/* visualization //{ */

/* visualizeTree //{ */
void Navigation::visualizeTree(const octomap::OcTree &tree) {
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Visualizing tree",
                   this->get_name());
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = parent_frame_;
  msg.header.stamp = this->get_clock()->now();
  msg.ns = "tree";
  msg.type = visualization_msgs::msg::Marker::POINTS;
  msg.id = 8;
  msg.action = visualization_msgs::msg::Marker::ADD;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = tree_points_scale_;
  msg.scale.y = tree_points_scale_;

  for (auto it = tree.begin(); it != tree.end(); it++) {
    if (it->getValue() == TreeValue::OCCUPIED) {
      geometry_msgs::msg::Point gp;
      auto color = generateColor(0, 0, 0, 1.0);
      gp.x = it.getX();
      gp.y = it.getY();
      gp.z = it.getZ();
      msg.points.push_back(gp);
      msg.colors.push_back(color);
    } else if (show_unoccupied_) {
      geometry_msgs::msg::Point gp;
      auto color = generateColor(0.7, 0.7, 0.7, 1.0);
      gp.x = it.getX();
      gp.y = it.getY();
      gp.z = it.getZ();
      msg.points.push_back(gp);
      msg.colors.push_back(color);
    }
  }
  binary_tree_publisher_->publish(msg);
}
//}

/* visualizeExpansions //{ */
void Navigation::visualizeExpansions(
    const std::set<Expansion, CostComparator> &open_list,
    const octomap::OcTree &tree) {
  RCLCPP_INFO_ONCE(this->get_logger(),
                   "[%s]: Visualizing open planning expansions",
                   this->get_name());
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = parent_frame_;
  msg.header.stamp = this->get_clock()->now();
  msg.ns = "expansions";
  msg.type = visualization_msgs::msg::Marker::POINTS;
  msg.id = 8;
  msg.action = visualization_msgs::msg::Marker::ADD;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = expansions_points_scale_;
  msg.scale.y = expansions_points_scale_;

  double max_cost = 0;
  double min_cost = 1e6;

  for (auto it = open_list.begin(); it != open_list.end(); it++) {
    if (it->total_cost > max_cost) {
      max_cost = it->total_cost;
    }
    if (it->total_cost < min_cost) {
      min_cost = it->total_cost;
    }
  }

  for (auto it = open_list.begin(); it != open_list.end(); it++) {
    auto coords = tree.keyToCoord(it->key);
    geometry_msgs::msg::Point gp;
    double brightness = (it->total_cost - min_cost) / (max_cost - min_cost);
    auto color = generateColor(0.65, 0.2, brightness, 0.8);
    gp.x = coords.x();
    gp.y = coords.y();
    gp.z = coords.z();
    msg.points.push_back(gp);
    msg.colors.push_back(color);
  }
  expansion_publisher_->publish(msg);
}
//}

/* visualizePath //{ */
void Navigation::visualizePath(const std::vector<octomap::point3d> waypoints) {
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Visualizing path",
                   this->get_name());
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = parent_frame_;
  msg.header.stamp = this->get_clock()->now();
  msg.ns = "path";
  msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
  msg.id = 8;
  msg.action = visualization_msgs::msg::Marker::ADD;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = path_points_scale_;

  for (size_t i = 1; i < waypoints.size(); i++) {
    geometry_msgs::msg::Point p1, p2;
    std_msgs::msg::ColorRGBA c;
    p1.x = waypoints[i - 1].x();
    p1.y = waypoints[i - 1].y();
    p1.z = waypoints[i - 1].z();
    p2.x = waypoints[i].x();
    p2.y = waypoints[i].y();
    p2.z = waypoints[i].z();
    c = generateColor(0.1, double(i) / double(waypoints.size()), 0.1, 1);
    msg.points.push_back(p1);
    msg.points.push_back(p2);
    msg.colors.push_back(c);
    msg.colors.push_back(c);
  }
  path_publisher_->publish(msg);
}
//}

/* visualizeGoals //{ */
void Navigation::visualizeGoals(const std::vector<octomap::point3d> waypoints,
                                const octomap::point3d current_goal) {
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Visualizing goals",
                   this->get_name());
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = parent_frame_;
  msg.header.stamp = this->get_clock()->now();
  msg.ns = "goals";
  msg.type = visualization_msgs::msg::Marker::POINTS;
  msg.id = 8;
  msg.action = visualization_msgs::msg::Marker::ADD;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = goal_points_scale_;
  msg.scale.y = goal_points_scale_;

  for (const auto &w : waypoints) {
    geometry_msgs::msg::Point p;
    std_msgs::msg::ColorRGBA c = generateColor(0.2, 0.4, 0.8, 1.0);
    p.x = w.x();
    p.y = w.y();
    p.z = w.z();
    msg.points.push_back(p);
    msg.colors.push_back(c);
  }

  // current goal
  geometry_msgs::msg::Point p;
  std_msgs::msg::ColorRGBA c = generateColor(0.2, 1.0, 0.2, 1.0);
  p.x = current_goal.x();
  p.y = current_goal.y();
  p.z = current_goal.z();
  msg.points.push_back(p);
  msg.colors.push_back(c);

  goal_publisher_->publish(msg);
}
//}

/* generateColor//{ */
std_msgs::msg::ColorRGBA Navigation::generateColor(const double r,
                                                   const double g,
                                                   const double b,
                                                   const double a) {
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}
//}

//}

/* parse_param //{ */
template <class T>
bool Navigation::parse_param(std::string param_name, T &param_dest) {
  const std::string param_path = "param_namespace." + param_name;
  this->declare_parameter(param_path);
  if (!this->get_parameter(param_path, param_dest)) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Could not load param '%s'",
                 this->get_name(), param_name.c_str());
    return false;
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "[" << this->get_name()
                                               << "]: Loaded '" << param_name
                                               << "' = '" << param_dest << "'");
  }
  return true;
}

template bool Navigation::parse_param<int>(std::string param_name,
                                           int &param_dest);
template bool Navigation::parse_param<double>(std::string param_name,
                                              double &param_dest);
template bool Navigation::parse_param<float>(std::string param_name,
                                             float &param_dest);
template bool Navigation::parse_param<std::string>(std::string param_name,
                                                   std::string &param_dest);
template bool Navigation::parse_param<bool>(std::string param_name,
                                            bool &param_dest);
template bool Navigation::parse_param<unsigned int>(std::string param_name,
                                                    unsigned int &param_dest);
//}

} // namespace navigation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(navigation::Navigation)
