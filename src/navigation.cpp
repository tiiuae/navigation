#include <fog_msgs/srv/path.hpp>
#include <fog_msgs/srv/vec4.hpp>
#include <fog_msgs/msg/future_trajectory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <deque>
#include <future>
#include <mutex>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <navigation/astar_planner.hpp>
#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/bounding_box_query.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <fog_msgs/msg/control_interface_diagnostics.hpp>
#include <fog_msgs/msg/navigation_diagnostics.hpp>
#include <fog_msgs/msg/obstacle_sectors.hpp>
#include <fog_msgs/srv/waypoint_to_local.hpp>
#include <fog_msgs/srv/path_to_local.hpp>

using namespace std::placeholders;

namespace navigation
{

enum status_t
{
  IDLE = 0,
  PLANNING,
  COMMANDING,
  MOVING,
  AVOIDING
};

enum waypoint_status_t
{
  EMPTY = 0,
  ONGOING,
  REACHED,
  UNREACHABLE
};

const std::string STATUS_STRING[] = {"IDLE", "PLANNING", "COMMANDING", "MOVING", "AVOIDING"};
const std::string WAYPOINT_STATUS_STRING[] = {"EMPTY", "ONGOING", "REACHED", "UNREACHABLE"};

double getYaw(const geometry_msgs::msg::Quaternion &q) {
  return atan2(2.0 * (q.z * q.w + q.x * q.y), -1.0 + 2.0 * (q.w * q.w + q.x * q.x));
}

geometry_msgs::msg::Quaternion yawToQuaternionMsg(const double &yaw) {
  geometry_msgs::msg::Quaternion msg;
  Eigen::Quaterniond             q =
      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  msg.w = q.w();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  return msg;
}

octomap::point3d toPoint3d(const Eigen::Vector4d &vec) {
  octomap::point3d p;
  p.x() = vec.x();
  p.y() = vec.y();
  p.z() = vec.z();
  return p;
}

double nanosecondsToSecs(const int64_t nanoseconds) {
  return nanoseconds / 1e9;
}

/* class Navigation //{ */
class Navigation : public rclcpp::Node {
public:
  Navigation(rclcpp::NodeOptions options);

private:
  // internal variables
  bool is_initialized_              = false;
  bool getting_octomap_             = false;
  bool getting_odometry_            = false;
  bool getting_desired_pose_        = false;
  bool getting_control_diagnostics_ = false;
  bool visualize_planner_           = true;
  bool show_unoccupied_             = false;
  bool override_previous_commands_  = false;
  bool bumper_active_               = false;

  std::string parent_frame_;
  int         replanning_counter_ = 0;

  bool control_moving_  = false;
  bool goal_reached_    = false;
  bool hover_requested_ = false;

  Eigen::Vector4d                  uav_pos_;
  Eigen::Vector4d                  desired_pose_;
  Eigen::Vector4d                  current_goal_;
  Eigen::Vector4d                  last_goal_;
  std::mutex                       octree_mutex_;
  std::mutex                       bumper_mutex_;
  std::shared_ptr<octomap::OcTree> octree_;
  status_t                         status_ = IDLE;
  waypoint_status_t                waypoint_status_ = EMPTY;

  std::vector<Eigen::Vector4d> waypoint_out_buffer_;
  std::deque<Eigen::Vector4d>  waypoint_in_buffer_;
  size_t                       current_waypoint_id_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr     execution_timer_;
  void                             navigationRoutine(void);

  std::unique_ptr<fog_msgs::msg::ObstacleSectors> bumper_msg_;

  // params
  double euclidean_distance_cutoff_;
  double safe_obstacle_distance_;
  double bumper_distance_factor_;
  double navigation_tolerance_;
  bool   unknown_is_occupied_;
  double min_altitude_;
  double max_altitude_;
  double max_goal_distance_;
  double distance_penalty_;
  double greedy_penalty_;
  double planning_tree_resolution_;
  double max_waypoint_distance_;
  double max_yaw_step_;
  double planning_timeout_;
  int    replanning_limit_;
  double replanning_distance_;
  double main_update_rate_;
  bool   bumper_enabled_;

  // visualization params
  double tree_points_scale_;
  double field_points_scale_;
  double expansions_points_scale_;
  double path_points_scale_;
  double goal_points_scale_;

  // publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr field_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr binary_tree_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr expansion_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                status_publisher_;
  rclcpp::Publisher<fog_msgs::msg::FutureTrajectory>::SharedPtr      future_trajectory_publisher_;
  rclcpp::Publisher<fog_msgs::msg::NavigationDiagnostics>::SharedPtr diagnostics_publisher_;

  // subscribers
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr                 octomap_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                    odometry_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr            desired_pose_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr                        goto_subscriber_;
  rclcpp::Subscription<fog_msgs::msg::ControlInterfaceDiagnostics>::SharedPtr control_diagnostics_subscriber_;
  rclcpp::Subscription<fog_msgs::msg::ObstacleSectors>::SharedPtr             bumper_subscriber_;

  // subscriber callbacks
  void octomapCallback(const octomap_msgs::msg::Octomap::UniquePtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg);
  void desiredPoseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr msg);
  void gotoCallback(const nav_msgs::msg::Path::UniquePtr msg);
  void controlDiagnosticsCallback(const fog_msgs::msg::ControlInterfaceDiagnostics::UniquePtr msg);
  void bumperCallback(const fog_msgs::msg::ObstacleSectors::UniquePtr msg);

  // services provided
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr goto_trigger_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hover_service_;

  rclcpp::Service<fog_msgs::srv::Vec4>::SharedPtr local_waypoint_service_;
  rclcpp::Service<fog_msgs::srv::Path>::SharedPtr local_path_service_;
  rclcpp::Service<fog_msgs::srv::Vec4>::SharedPtr gps_waypoint_service_;
  rclcpp::Service<fog_msgs::srv::Path>::SharedPtr gps_path_service_;


  rclcpp::Client<fog_msgs::srv::Path>::SharedPtr            local_path_client_;
  // rclcpp::Client<fog_msgs::srv::Path>::SharedPtr            gps_path_client_;
  rclcpp::Client<fog_msgs::srv::WaypointToLocal>::SharedPtr waypoint_to_local_client_;
  rclcpp::Client<fog_msgs::srv::PathToLocal>::SharedPtr     path_to_local_client_;

  // service callbacks
  bool gotoTriggerCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool hoverCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  bool localWaypointCallback(const std::shared_ptr<fog_msgs::srv::Vec4::Request> request, std::shared_ptr<fog_msgs::srv::Vec4::Response> response);
  bool localPathCallback(const std::shared_ptr<fog_msgs::srv::Path::Request> request, std::shared_ptr<fog_msgs::srv::Path::Response> response);
  bool gpsWaypointCallback(const std::shared_ptr<fog_msgs::srv::Vec4::Request> request, std::shared_ptr<fog_msgs::srv::Vec4::Response> response);
  bool gpsPathCallback(const std::shared_ptr<fog_msgs::srv::Path::Request> request, std::shared_ptr<fog_msgs::srv::Path::Response> response);

  // future callback
  bool waypointFutureCallback(rclcpp::Client<fog_msgs::srv::WaypointToLocal>::SharedFuture future);
  bool pathFutureCallback(rclcpp::Client<fog_msgs::srv::PathToLocal>::SharedFuture future);

  // visualization
  void visualizeTree(const octomap::OcTree &tree);
  void visualizeExpansions(const std::unordered_set<navigation::Node, HashFunction> open, const std::unordered_set<navigation::Node, HashFunction> &closed,
                           const octomap::OcTree &tree);
  void visualizePath(const std::vector<Eigen::Vector4d> &waypoints);
  void visualizeGoals(const std::deque<Eigen::Vector4d> &waypoints);

  std_msgs::msg::ColorRGBA generateColor(const double r, const double g, const double b, const double a);

  std::vector<Eigen::Vector4d> resamplePath(const std::vector<octomap::point3d> &waypoints, const double start_yaw, const double end_yaw);

  std::shared_ptr<fog_msgs::srv::Path::Request> waypointsToPathSrv(const std::vector<Eigen::Vector4d> &waypoints, bool use_first = true);
  void                                          hover();

  void publishDiagnostics();
  void publishFutureTrajectory(std::vector<Eigen::Vector4d> waypoints);

  bool            bumperCheckObstacles(const fog_msgs::msg::ObstacleSectors &bumper_msg);
  Eigen::Vector3d bumperGetAvoidanceVector(const fog_msgs::msg::ObstacleSectors &bumper_msg);

  template <class T>
  bool parse_param(const std::string &param_name, T &param_dest);
};
//}

/* constructor //{ */
Navigation::Navigation(rclcpp::NodeOptions options) : Node("navigation", options) {

  RCLCPP_INFO(this->get_logger(), "[%s]: Initializing...", this->get_name());

  /* parse params from config file //{ */
  RCLCPP_INFO(this->get_logger(), "-------------- Loading parameters --------------");
  bool loaded_successfully = true;
  loaded_successfully &= parse_param("planning.euclidean_distance_cutoff", euclidean_distance_cutoff_);
  loaded_successfully &= parse_param("planning.safe_obstacle_distance", safe_obstacle_distance_);
  loaded_successfully &= parse_param("planning.bumper_distance_factor", bumper_distance_factor_);
  loaded_successfully &= parse_param("planning.unknown_is_occupied", unknown_is_occupied_);
  loaded_successfully &= parse_param("planning.navigation_tolerance", navigation_tolerance_);
  loaded_successfully &= parse_param("planning.min_altitude", min_altitude_);
  loaded_successfully &= parse_param("planning.max_altitude", max_altitude_);
  loaded_successfully &= parse_param("planning.max_goal_distance", max_goal_distance_);
  loaded_successfully &= parse_param("planning.distance_penalty", distance_penalty_);
  loaded_successfully &= parse_param("planning.greedy_penalty", greedy_penalty_);
  loaded_successfully &= parse_param("planning.planning_tree_resolution", planning_tree_resolution_);
  loaded_successfully &= parse_param("planning.max_waypoint_distance", max_waypoint_distance_);
  loaded_successfully &= parse_param("planning.max_yaw_step", max_yaw_step_);
  loaded_successfully &= parse_param("planning.planning_timeout", planning_timeout_);
  loaded_successfully &= parse_param("planning.replanning_limit", replanning_limit_);
  loaded_successfully &= parse_param("planning.replanning_distance", replanning_distance_);
  loaded_successfully &= parse_param("planning.override_previous_commands", override_previous_commands_);
  loaded_successfully &= parse_param("planning.main_update_rate", main_update_rate_);

  loaded_successfully &= parse_param("visualization.visualize_planner", visualize_planner_);
  loaded_successfully &= parse_param("visualization.show_unoccupied", show_unoccupied_);
  loaded_successfully &= parse_param("visualization.tree_points_scale", tree_points_scale_);
  loaded_successfully &= parse_param("visualization.field_points_scale", field_points_scale_);
  loaded_successfully &= parse_param("visualization.expansions_points_scale", expansions_points_scale_);
  loaded_successfully &= parse_param("visualization.path_points_scale", path_points_scale_);
  loaded_successfully &= parse_param("visualization.goal_points_scale", goal_points_scale_);

  loaded_successfully &= parse_param("bumper.enabled", bumper_enabled_);

  if (!loaded_successfully) {
    const std::string str = "Could not load all non-optional parameters. Shutting down.";
    RCLCPP_ERROR(this->get_logger(), str.c_str());
    rclcpp::shutdown();
    return;
  }
  //}

  current_waypoint_id_ = 0;
  bumper_msg_.reset();

  callback_group_        = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto sub_opt           = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;

  // publishers
  field_publisher_             = this->create_publisher<visualization_msgs::msg::Marker>("~/field_markers_out", 1);
  binary_tree_publisher_       = this->create_publisher<visualization_msgs::msg::Marker>("~/binary_tree_markers_out", 1);
  expansion_publisher_         = this->create_publisher<visualization_msgs::msg::Marker>("~/expansion_markers_out", 1);
  path_publisher_              = this->create_publisher<visualization_msgs::msg::Marker>("~/path_markers_out", 1);
  goal_publisher_              = this->create_publisher<visualization_msgs::msg::Marker>("~/goal_markers_out", 1);
  status_publisher_            = this->create_publisher<std_msgs::msg::String>("~/status_out", 1);
  future_trajectory_publisher_ = this->create_publisher<fog_msgs::msg::FutureTrajectory>("~/future_trajectory_out", 1);
  diagnostics_publisher_       = this->create_publisher<fog_msgs::msg::NavigationDiagnostics>("~/diagnostics_out", 5);

  // subscribers
  odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("~/odometry_in", 1, std::bind(&Navigation::odometryCallback, this, _1));
  desired_pose_subscriber_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>("~/desired_pose_in", 1, std::bind(&Navigation::desiredPoseCallback, this, _1));
  goto_subscriber_                = this->create_subscription<nav_msgs::msg::Path>("~/goto_in", 1, std::bind(&Navigation::gotoCallback, this, _1));
  control_diagnostics_subscriber_ = this->create_subscription<fog_msgs::msg::ControlInterfaceDiagnostics>(
      "~/control_diagnostics_in", 1, std::bind(&Navigation::controlDiagnosticsCallback, this, _1));
  octomap_subscriber_ = this->create_subscription<octomap_msgs::msg::Octomap>("~/octomap_in", 1, std::bind(&Navigation::octomapCallback, this, _1), sub_opt);
  bumper_subscriber_  = this->create_subscription<fog_msgs::msg::ObstacleSectors>("~/bumper_in", 1, std::bind(&Navigation::bumperCallback, this, _1), sub_opt);

  // clients
  local_path_client_        = this->create_client<fog_msgs::srv::Path>("~/local_path_out");
  // gps_path_client_          = this->create_client<fog_msgs::srv::Path>("~/gps_path_out");
  waypoint_to_local_client_ = this->create_client<fog_msgs::srv::WaypointToLocal>("~/waypoint_to_local_out");
  path_to_local_client_     = this->create_client<fog_msgs::srv::PathToLocal>("~/path_to_local_out");

  // service handlers
  goto_trigger_service_   = this->create_service<std_srvs::srv::Trigger>("~/goto_trigger_in", std::bind(&Navigation::gotoTriggerCallback, this, _1, _2));
  hover_service_          = this->create_service<std_srvs::srv::Trigger>("~/hover_in", std::bind(&Navigation::hoverCallback, this, _1, _2));
  local_waypoint_service_ = this->create_service<fog_msgs::srv::Vec4>("~/local_waypoint_in", std::bind(&Navigation::localWaypointCallback, this, _1, _2));
  local_path_service_     = this->create_service<fog_msgs::srv::Path>("~/local_path_in", std::bind(&Navigation::localPathCallback, this, _1, _2));
  gps_waypoint_service_   = this->create_service<fog_msgs::srv::Vec4>("~/gps_waypoint_in", std::bind(&Navigation::gpsWaypointCallback, this, _1, _2));
  gps_path_service_       = this->create_service<fog_msgs::srv::Path>("~/gps_path_in", std::bind(&Navigation::gpsPathCallback, this, _1, _2));

  // timers
  execution_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / main_update_rate_), std::bind(&Navigation::navigationRoutine, this), callback_group_);

  if (max_waypoint_distance_ <= 0) {
    max_waypoint_distance_ = replanning_distance_;
  }

  is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "[%s]: Initialized", this->get_name());
}
//}

/* octomapCallback //{ */
void Navigation::octomapCallback(const octomap_msgs::msg::Octomap::UniquePtr msg) {
  getting_octomap_ = true;
  parent_frame_    = msg->header.frame_id;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting octomap", this->get_name());
  octomap_msgs::msg::Octomap map_msg = *msg;

  auto treePtr = octomap_msgs::fullMsgToMap(*msg);

  if (!treePtr) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Octomap message is empty!", this->get_name());
  } else {
    std::scoped_lock lock(octree_mutex_);
    octree_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(treePtr));
  }
}
//}

/* odometryCallback //{ */
void Navigation::odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }

  getting_odometry_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting odometry", this->get_name());
  uav_pos_[0] = msg->pose.pose.position.x;
  uav_pos_[1] = msg->pose.pose.position.y;
  uav_pos_[2] = msg->pose.pose.position.z;
  uav_pos_[3] = getYaw(msg->pose.pose.orientation);
}
//}

/* desiredPoseCallback //{ */
void Navigation::desiredPoseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }

  getting_desired_pose_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting desired pose", this->get_name());
  desired_pose_[0] = msg->pose.position.x;
  desired_pose_[1] = msg->pose.position.y;
  desired_pose_[2] = msg->pose.position.z;
  desired_pose_[3] = getYaw(msg->pose.orientation);
}
//}

/* controlDiagnosticsCallback //{ */
void Navigation::controlDiagnosticsCallback(const fog_msgs::msg::ControlInterfaceDiagnostics::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }
  getting_control_diagnostics_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting control_interface diagnostics", this->get_name());
  control_moving_ = msg->moving;
  goal_reached_   = msg->mission_finished;
}
//}

/* bumperCallback //{ */
void Navigation::bumperCallback(const fog_msgs::msg::ObstacleSectors::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting bumper msgs", this->get_name());
  std::scoped_lock lock(bumper_mutex_);
  bumper_msg_ = std::unique_ptr<fog_msgs::msg::ObstacleSectors>{new fog_msgs::msg::ObstacleSectors{*msg}};
}
//}

/* gotoCallback //{ */
void Navigation::gotoCallback(const nav_msgs::msg::Path::UniquePtr msg) {

  if (!is_initialized_) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Goto rejected, node not initialized", this->get_name());
    return;
  }

  if (!getting_octomap_) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Goto rejected, octomap not received", this->get_name());
    return;
  }

  if (!getting_control_diagnostics_) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Goto rejected, control_interface diagnostics not received", this->get_name());
    return;
  }

  if (msg->poses.empty()) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Goto rejected, path input does not contain any waypoints", this->get_name());
    return;
  }

  if (status_ != IDLE) {
    if (!override_previous_commands_) {
      RCLCPP_ERROR(this->get_logger(), "[%s]: Goto rejected, vehicle not IDLE", this->get_name());
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "[%s]: Override previous navigation commands", this->get_name());
      hover();
    }
  }

  RCLCPP_INFO(this->get_logger(), "[%s]: Recieved %ld waypoints", this->get_name(), msg->poses.size());

  waypoint_in_buffer_.clear();
  current_waypoint_id_ = 0;
  for (const auto &p : msg->poses) {
    Eigen::Vector4d point;
    point[0] = p.pose.position.x;
    point[1] = p.pose.position.y;
    point[2] = p.pose.position.z;
    point[3] = getYaw(p.pose.orientation);
    waypoint_in_buffer_.push_back(point);
  }
  RCLCPP_INFO(this->get_logger(), "[%s]: Waiting for planning trigger", this->get_name());
}
//}

/* gotoTriggerCallback //{ */
bool Navigation::gotoTriggerCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response>                       response) {
  if (!is_initialized_) {
    response->message = "Goto rejected, node not initialized";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!getting_octomap_) {
    response->message = "Goto rejected, octomap not received";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!getting_control_diagnostics_) {
    response->message = "Goto rejected, control_interface diagnostics not received";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (current_waypoint_id_ >= waypoint_in_buffer_.size() || waypoint_in_buffer_.empty()) {
    response->message = "Goto rejected, no waypoint provided";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (status_ != IDLE) {
    if (!override_previous_commands_) {
      RCLCPP_ERROR(this->get_logger(), "[%s]: Goto rejected, vehicle not IDLE", this->get_name());
      return true;
    } else {
      RCLCPP_INFO(this->get_logger(), "[%s]: Override previous navigation commands", this->get_name());
      hover();
    }
  }

  response->message = "Planning started";
  response->success = true;
  RCLCPP_INFO(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
  status_ = PLANNING;
  return true;
}
//}

/* localPathCallback //{ */
bool Navigation::localPathCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::Path::Request> request,
                                   std::shared_ptr<fog_msgs::srv::Path::Response>                       response) {
  if (!is_initialized_) {
    response->message = "Path rejected, node not initialized";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!getting_octomap_) {
    response->message = "Path rejected, octomap not received";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!getting_control_diagnostics_) {
    response->message = "Path rejected, control_interface diagnostics not received";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (request->path.poses.empty()) {
    response->message = "Path rejected, path input does not contain any waypoints";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (status_ != IDLE) {
    if (!override_previous_commands_) {
      response->message = "Path rejected, vehicle not IDLE";
      response->success = false;
      RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
      return true;
    } else {
      RCLCPP_INFO(this->get_logger(), "[%s]: Override previous navigation commands", this->get_name());
      hover();
    }
  }

  waypoint_in_buffer_.clear();
  current_waypoint_id_ = 0;
  for (const auto &p : request->path.poses) {
    Eigen::Vector4d point;
    point[0] = p.pose.position.x;
    point[1] = p.pose.position.y;
    point[2] = p.pose.position.z;
    point[3] = getYaw(p.pose.orientation);
    waypoint_in_buffer_.push_back(point);
  }

  RCLCPP_INFO(this->get_logger(), "[%s]: Recieved %ld waypoints. Planning started", this->get_name(), request->path.poses.size());
  response->message = "Planning started";
  response->success = true;
  status_           = PLANNING;
  return true;
}
//}

/* gpsPathCallback //{ */
bool Navigation::gpsPathCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::Path::Request> request,
                                 std::shared_ptr<fog_msgs::srv::Path::Response>                       response) {
  if (!is_initialized_) {
    response->message = "Path rejected, node not initialized";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!getting_octomap_) {
    response->message = "Path rejected, octomap not received";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!getting_control_diagnostics_) {
    response->message = "Path rejected, control_interface diagnostics not received";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (request->path.poses.empty()) {
    response->message = "Path rejected, path input does not contain any waypoints";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (status_ != IDLE) {
    if (!override_previous_commands_) {
      response->message = "Path rejected, vehicle not IDLE";
      response->success = false;
      RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
      return true;
    } else {
      RCLCPP_INFO(this->get_logger(), "[%s]: Override previous navigation commands", this->get_name());
      hover();
    }
  }

  auto convert_path_srv  = std::make_shared<fog_msgs::srv::PathToLocal::Request>();
  convert_path_srv->path = request->path;
  RCLCPP_INFO(this->get_logger(), "[%s]: Calling Path transform", this->get_name());
  auto call_result = path_to_local_client_->async_send_request(convert_path_srv, std::bind(&Navigation::pathFutureCallback, this, std::placeholders::_1));

  response->message = "Processing path";
  response->success = true;
  return true;
}
//}

/* localWaypointCallback //{ */
bool Navigation::localWaypointCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
                                       std::shared_ptr<fog_msgs::srv::Vec4::Response>                       response) {
  if (!is_initialized_) {
    response->message = "Goto rejected, node not initialized";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!getting_octomap_) {
    response->message = "Goto rejected, octomap not received";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!getting_control_diagnostics_) {
    response->message = "Goto rejected, control_interface diagnostics not received";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (status_ != IDLE) {
    if (!override_previous_commands_) {
      response->message = "Goto rejected, vehicle not IDLE";
      response->success = false;
      RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
      return true;
    } else {
      RCLCPP_INFO(this->get_logger(), "[%s]: Override previous navigation commands", this->get_name());
      hover();
    }
  }

  waypoint_in_buffer_.clear();
  current_waypoint_id_ = 0;
  Eigen::Vector4d point;
  point[0] = request->goal[0];
  point[1] = request->goal[1];
  point[2] = request->goal[2];
  point[3] = request->goal[3];
  waypoint_in_buffer_.push_back(point);

  RCLCPP_INFO(this->get_logger(), "[%s]: Waypoint set: %.2f, %.2f, %.2f, %.2f", this->get_name(), point.x(), point.y(), point.z(), point.w());

  RCLCPP_INFO(this->get_logger(), "[%s]: Planning started", this->get_name());
  status_ = PLANNING;

  response->message = "Planning started";
  response->success = true;
  return true;
}
//}

/* gpsWaypointCallback //{ */
bool Navigation::gpsWaypointCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
                                     std::shared_ptr<fog_msgs::srv::Vec4::Response>                       response) {
  if (!is_initialized_) {
    response->message = "Goto rejected, node not initialized";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!getting_octomap_) {
    response->message = "Goto rejected, octomap not received";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!getting_control_diagnostics_) {
    response->message = "Goto rejected, control_interface diagnostics not received";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (status_ != IDLE) {
    if (!override_previous_commands_) {
      response->message = "Goto rejected, vehicle not IDLE";
      response->success = false;
      RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
      return true;
    } else {
      RCLCPP_INFO(this->get_logger(), "[%s]: Override previous navigation commands", this->get_name());
      hover();
    }
  }

  RCLCPP_INFO(this->get_logger(), "[%s]: Creating coord transform request", this->get_name());
  auto waypoint_convert_srv                 = std::make_shared<fog_msgs::srv::WaypointToLocal::Request>();
  waypoint_convert_srv->latitude_deg        = request->goal[0];
  waypoint_convert_srv->longitude_deg       = request->goal[1];
  waypoint_convert_srv->relative_altitude_m = request->goal[2];
  waypoint_convert_srv->yaw                 = request->goal[3];
  RCLCPP_INFO(this->get_logger(), "[%s]: Calling coord transform", this->get_name());
  auto call_result =
      waypoint_to_local_client_->async_send_request(waypoint_convert_srv, std::bind(&Navigation::waypointFutureCallback, this, std::placeholders::_1));


  response->message = "Processing goto";
  response->success = true;
  return true;
}
//}

/* hoverCallback //{ */
bool Navigation::hoverCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response>                       response) {
  if (!is_initialized_) {
    response->message = "Hover rejected, node not initialized";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!getting_control_diagnostics_) {
    response->message = "Hover rejected, control_interface diagnostics not received";
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (status_ == IDLE) {
    response->message = "Hover not necessary, vehicle is IDLE";
    response->success = false;
    RCLCPP_WARN(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  hover_requested_  = true;
  response->message = "Navigation stopped. Hovering";
  response->success = true;
  RCLCPP_INFO(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
  return true;
}
//}

/* navigationRoutine //{ */
void Navigation::navigationRoutine(void) {
  if (is_initialized_ && getting_octomap_ && getting_control_diagnostics_ && getting_odometry_ && getting_desired_pose_) {

    if (bumper_enabled_) {
      std::scoped_lock lock(bumper_mutex_);
      if (bumper_msg_ == nullptr || nanosecondsToSecs((this->get_clock()->now() - bumper_msg_->header.stamp).nanoseconds()) > 1.0) {
        RCLCPP_WARN(this->get_logger(), "[%s]: Missing bumper data calling hover.", this->get_name());
        hover();
        status_ = IDLE;
        waypoint_status_ = EMPTY;
      }
    }

    switch (status_) {

      /* IDLE //{ */
      case IDLE: {
        hover_requested_    = false;
        replanning_counter_ = 0;

        if (bumper_enabled_) {
          std::scoped_lock lock(bumper_mutex_);

          bool obstacle_detected = bumperCheckObstacles(*bumper_msg_);
          if (!bumper_active_) {
            if (obstacle_detected) {
              RCLCPP_WARN(this->get_logger(), "[%s]: [Bumper] - obstacle in proximity! Starting avoidance", this->get_name());
              status_ = AVOIDING;
              break;
            }
          } else if (!obstacle_detected) {
            RCLCPP_WARN(this->get_logger(), "[%s]: Deactivating bumper", this->get_name());
            bumper_active_ = false;
            status_        = PLANNING;
            break;
          }
        }

        std::vector<Eigen::Vector4d> waypoints;
        waypoints.push_back(uav_pos_);
        publishFutureTrajectory(waypoints);
        break;
      }
        //}

      /* PLANNING //{ */
      case PLANNING: {

        waypoint_out_buffer_.clear();

        if (hover_requested_) {
          hover();
          status_ = IDLE;
          waypoint_status_ = EMPTY;
          break;
        }

        std::scoped_lock lock(octree_mutex_);

        if (octree_ == NULL || octree_->size() < 1) {
          RCLCPP_WARN(this->get_logger(), "[%s]: Octomap is NULL or empty! Abort planning and swiching to IDLE", this->get_name());
          status_ = IDLE;
          waypoint_status_ = EMPTY;
          break;
        }

        if (waypoint_in_buffer_.empty() || (current_waypoint_id_) >= waypoint_in_buffer_.size()) {
          RCLCPP_WARN(this->get_logger(), "[%s]: No navigation goals available. Switching to IDLE", this->get_name());
          status_ = IDLE;
          break;
        }

        last_goal_    = current_goal_;
        current_goal_ = waypoint_in_buffer_[current_waypoint_id_];

        RCLCPP_INFO(this->get_logger(), "[%s]: Waypoint [%.2f, %.2f, %.2f, %.2f] set as a next goal", this->get_name(), 
                                          current_goal_[0], current_goal_[1], current_goal_[2], current_goal_[3]);
        RCLCPP_INFO(this->get_logger(), "[%s]: Last received desired pose [%.2f, %.2f, %.2f, %.2f]", this->get_name(),
                                          desired_pose_[0], desired_pose_[1], desired_pose_[2], desired_pose_[3]);            

        visualizeGoals(waypoint_in_buffer_);

        if (replanning_counter_ >= replanning_limit_) {
          RCLCPP_ERROR(this->get_logger(),
                       "[%s]: No waypoint produced after %d repeated attempts. "
                       "Please provide a new waypoint",
                       this->get_name(), replanning_counter_);
          status_ = IDLE;
          waypoint_status_ = UNREACHABLE;
        }

        navigation::AstarPlanner planner =
            navigation::AstarPlanner(safe_obstacle_distance_, euclidean_distance_cutoff_, planning_tree_resolution_, distance_penalty_, greedy_penalty_,
                                     min_altitude_, max_altitude_, planning_timeout_, max_waypoint_distance_, unknown_is_occupied_);

        octomap::point3d planning_start = toPoint3d(uav_pos_);
        octomap::point3d pos_cmd        = toPoint3d(desired_pose_);
        /*if ((desired_pose_.head<3>() - uav_pos_.head<3>()).norm() <= navigation_tolerance_) {
          planning_start = toPoint3d(desired_pose_);
        } else {
          planning_start = toPoint3d(uav_pos_);
        }*/
        octomap::point3d planning_goal  = toPoint3d(current_goal_);

        std::pair<std::vector<octomap::point3d>, PlanningResult> waypoints =
            planner.findPath(planning_start, planning_goal, pos_cmd, octree_, planning_timeout_, std::bind(&Navigation::visualizeTree, this, _1),
                             std::bind(&Navigation::visualizeExpansions, this, _1, _2, _3));

        RCLCPP_INFO(this->get_logger(), "[%s]: Planner returned %ld waypoints, before resampling:", this->get_name(), waypoints.first.size());

        for (auto &w :waypoints.first) {
          RCLCPP_INFO(this->get_logger(), "[%s]:        %.2f, %.2f, %.2f", this->get_name(), w.x(), w.y(), w.z());
        }
        /* GOAL_REACHED //{ */
        if (waypoints.second == GOAL_REACHED) {
          RCLCPP_INFO(this->get_logger(), "[%s]: Current goal reached", this->get_name());
          waypoint_status_ = REACHED;
          if (current_waypoint_id_ >= waypoint_in_buffer_.size()) {
            RCLCPP_INFO(this->get_logger(), "[%s]: The last provided navigation goal has been visited. Switching to IDLE", this->get_name());
            status_ = IDLE;
            break;

          } else {
            current_waypoint_id_++;
          }

          break;
        }
        //}

        /* COMPLETE //{ */
        if (waypoints.second == COMPLETE) {
          replanning_counter_ = 0;
        }
        //}

        /* INCOMPLETE //{ */
        if (waypoints.second == INCOMPLETE) {

          if (waypoints.first.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "[%s]: path not found", this->get_name());
            replanning_counter_++;
            break;
          }

          Eigen::Vector3d w_start;
          w_start.x() = waypoints.first.front().x();
          w_start.y() = waypoints.first.front().y();
          w_start.z() = waypoints.first.front().z();

          Eigen::Vector3d w_end;
          w_end.x() = waypoints.first.back().x();
          w_end.y() = waypoints.first.back().y();
          w_end.z() = waypoints.first.back().z();

          double path_start_end_dist = (w_end - w_start).norm();

          if (path_start_end_dist < 1.1 * planning_tree_resolution_) {
            RCLCPP_WARN(this->get_logger(), "[%s]: path too short", this->get_name());
            replanning_counter_++;
            break;
          }
        }
        //}

        /* GOAL_IN_OBSTACLE //{ */
        if (waypoints.second == GOAL_IN_OBSTACLE) {
          replanning_counter_ = 0;
          RCLCPP_WARN(this->get_logger(), "[%s]: Goal [%.2f, %.2f, %.2f, %.2f] is inside an inflated obstacle", this->get_name(), current_goal_[0],
                      current_goal_[1], current_goal_[2], current_goal_[3]);
        }
        //}

        /* FAILURE //{ */
        if (waypoints.second == FAILURE) {
          RCLCPP_WARN(this->get_logger(), "[%s]: path to goal not found", this->get_name());
          replanning_counter_++;
          break;
        }
        //}

        /* resample path and add yaw //{ */
        std::vector<Eigen::Vector4d> resampled = resamplePath(waypoints.first, desired_pose_.w(), current_goal_.w());

        bool output_current_goal = waypoints.second == COMPLETE;
        for (auto &w : resampled) {
          if ((w.head<3>() - desired_pose_.head<3>()).norm() <= replanning_distance_) {
            waypoint_out_buffer_.push_back(w);
          } else {
            RCLCPP_INFO(this->get_logger(), "[%s]: Path exceeding replanning distance", this->get_name());
            output_current_goal = false;
            break;
          }
        }

        if (output_current_goal) {
          waypoint_out_buffer_.push_back(current_goal_);
        }
        //}

        status_ = COMMANDING;
        waypoint_status_ = ONGOING;
        break;
      }
      //}

      /* COMMANDING //{ */
      case COMMANDING: {

        if (hover_requested_) {
          hover();
          status_ = IDLE;
          waypoint_status_ = EMPTY;
          break;
        }

        if (waypoint_out_buffer_.size() < 1) {
          RCLCPP_WARN(this->get_logger(), "[%s]: No waypoints in the output buffer. Replanning", this->get_name());
          replanning_counter_++;
          status_ = PLANNING;
          break;
        }

        RCLCPP_INFO(this->get_logger(), "[%s]: Sending %ld waypoints to the control interface:", this->get_name(), waypoint_out_buffer_.size());
        for (auto &w : waypoint_out_buffer_) {
          RCLCPP_INFO(this->get_logger(), "[%s]:        %.2f, %.2f, %.2f, %.2f", this->get_name(), w.x(), w.y(), w.z(), w.w());
        }
        visualizePath(waypoint_out_buffer_);
        publishFutureTrajectory(waypoint_out_buffer_);
        auto waypoints_srv = waypointsToPathSrv(waypoint_out_buffer_, false);
        auto call_result   = local_path_client_->async_send_request(waypoints_srv);
        status_            = MOVING;
        break;
      }
        //}

        /* MOVING //{ */
      case MOVING: {
        if (hover_requested_) {
          hover();
          status_ = IDLE;
          waypoint_status_ = EMPTY;
          break;
        }

        if (bumper_enabled_) {
          std::scoped_lock lock(bumper_mutex_);
          bool             obstacle_detected = bumperCheckObstacles(*bumper_msg_);
          if (!bumper_active_) {
            if (obstacle_detected) {
              RCLCPP_WARN(this->get_logger(), "[%s]: [Bumper] - obstacle in proximity while moving! Calling hover", this->get_name());
              hover();
              status_ = IDLE;
              break;
            }
          }
        }

        replanning_counter_ = 0;
        publishFutureTrajectory(waypoint_out_buffer_);
        if ((!control_moving_ && goal_reached_)) {
          RCLCPP_INFO(this->get_logger(), "[%s]: End of current segment reached", this->get_name());
          if (bumper_active_) {
            status_ = AVOIDING;
          } else {
            status_ = PLANNING;
          }
        }
        break;
      }
        //}

        /* AVOIDING //{ */
      case AVOIDING: {

        std::scoped_lock lock(bumper_mutex_);
        Eigen::Vector3d  avoidance_vector = bumperGetAvoidanceVector(*bumper_msg_);

        if (avoidance_vector.norm() == 0) {
          RCLCPP_INFO(this->get_logger(), "[Navigation]: Nothing to avoid");
          status_ = IDLE;
          break;
        }

        Eigen::Vector4d new_goal = uav_pos_;
        new_goal.x() += avoidance_vector.x();
        new_goal.y() += avoidance_vector.y();
        new_goal.z() = desired_pose_.z() + avoidance_vector.z();  // TODO this is a temporary workaround until state estimation (odometry pkg) is integrated
        new_goal.w() = desired_pose_.w();

        RCLCPP_INFO(this->get_logger(), "[Bumper]: Avoiding obstacle by moving from [%.2f, %.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f, %.2f]", uav_pos_.x(),
                    uav_pos_.y(), uav_pos_.z(), uav_pos_.w(), new_goal.x(), new_goal.y(), new_goal.z(), new_goal.w());

        waypoint_out_buffer_.clear();
        waypoint_out_buffer_.push_back(desired_pose_);
        waypoint_out_buffer_.push_back(new_goal);

        if (!bumper_active_) {
          RCLCPP_WARN(this->get_logger(), "[%s]: Activating bumper", this->get_name());
          bumper_active_ = true;
        }

        status_ = COMMANDING;
        break;
      }
        //}
    }
  }

  std_msgs::msg::String msg;
  msg.data = STATUS_STRING[status_];
  status_publisher_->publish(msg);
  publishDiagnostics();
}
//}

/* bumperCheckObstacles //{ */
bool Navigation::bumperCheckObstacles(const fog_msgs::msg::ObstacleSectors &bumper_msg) {


  for (int i = 0; i < int(bumper_msg.n_horizontal_sectors); i++) {

    if (bumper_msg.sectors[i] < 0) {
      continue;
    }

    // if the sector is under safe distance
    if (bumper_msg.sectors[i] <= safe_obstacle_distance_ * bumper_distance_factor_) {
      return true;
    }
  }

  return false;
}
//}

/* bumperGetAvoidanceVector//{ */
Eigen::Vector3d Navigation::bumperGetAvoidanceVector(const fog_msgs::msg::ObstacleSectors &bumper_msg) {

  double sector_size = (2.0 * M_PI) / double(bumper_msg.n_horizontal_sectors);

  Eigen::Vector3d forward = Eigen::Vector3d::UnitX();

  for (int i = 0; i < int(bumper_msg.n_horizontal_sectors); i++) {

    if (bumper_msg.sectors[i] < 0) {
      continue;
    }

    if (bumper_msg.sectors[i] <= safe_obstacle_distance_ * bumper_distance_factor_) {
      Eigen::Quaterniond q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(sector_size * i + M_PI + uav_pos_.w(), Eigen::Vector3d::UnitZ());

      Eigen::Vector3d avoidance_vector = q * ((safe_obstacle_distance_ - bumper_msg.sectors[i] + planning_tree_resolution_) * forward);

      return avoidance_vector;
    }
  }
  return Eigen::Vector3d::Zero();
}
//}

/* resamplePath //{ */

std::vector<Eigen::Vector4d> Navigation::resamplePath(const std::vector<octomap::point3d> &waypoints, const double start_yaw, const double end_yaw) {
  std::vector<Eigen::Vector4d> ret;

  if (waypoints.size() < 2) {
    for (auto &w : waypoints) {
      ret.push_back(Eigen::Vector4d(w.x(), w.y(), w.z(), end_yaw));
    }
    return ret;
  }

  ret.push_back(Eigen::Vector4d(waypoints.front().x(), waypoints.front().y(), waypoints.front().z(), 0.0));

  size_t i = 1;
  while (i < waypoints.size()) {
    double dist = std::sqrt(std::pow(ret.back().x() - waypoints[i].x(), 2) + std::pow(ret.back().y() - waypoints[i].y(), 2) +
                            std::pow(ret.back().z() - waypoints[i].z(), 2));
    if (dist > max_waypoint_distance_) {
      Eigen::Vector3d direction;
      direction.x() = waypoints[i].x() - ret.back().x();
      direction.y() = waypoints[i].y() - ret.back().y();
      direction.z() = waypoints[i].z() - ret.back().z();
      direction     = direction.normalized() * max_waypoint_distance_;

      Eigen::Vector4d padded;
      padded.x() = ret.back().x() + direction.x();
      padded.y() = ret.back().y() + direction.y();
      padded.z() = ret.back().z() + direction.z();
      ret.push_back(padded);
    } else {
      ret.push_back(Eigen::Vector4d(waypoints[i].x(), waypoints[i].y(), waypoints[i].z(), 0.0));
      i++;
    }
  }

  std::cout << "[Navigation]: Padded " << waypoints.size() << " original waypoints to " << ret.size() << " points\n";

  /* add yaw //{ */

  double delta_yaw = std::atan2(std::sin(end_yaw - start_yaw), std::cos(end_yaw - start_yaw));
  double yaw_step  = delta_yaw / ret.size();
  std::cout << "[Navigation]: Start yaw: " << start_yaw << ", end yaw: " << end_yaw << ", yaw step: " << yaw_step << "\n";

  if (std::abs(yaw_step) <= max_yaw_step_) {
    ret.front().w() = start_yaw;
    for (size_t j = 1; j < ret.size(); j++) {
      ret[j].w() = ret[j - 1].w() + yaw_step;
    }
  } else {
    // resample again to limit yaw rate and avoid fast turning
    int resampling_factor = int(std::abs(yaw_step) / max_yaw_step_) + 1;
    std::cout << "[Navigation]: Yaw step: " << yaw_step << " is greater than max yaw step: " << max_yaw_step_ << "\n";
    std::cout << "[Navigation]: Resampling factor: " << resampling_factor << "\n";
    std::vector<Eigen::Vector4d> resampled;
    for (const auto &p : ret) {
      for (int j = 0; j < resampling_factor; j++) {
        Eigen::Vector4d v(p[0], p[1], p[2], p[3]);
        resampled.push_back(v);
      }
    }

    ret.clear();
    ret.insert(ret.end(), resampled.begin(), resampled.end());
    yaw_step = delta_yaw / ret.size();
    std::cout << "[Navigation]: New yaw step: " << yaw_step << "\n";
    ret.front().w() = start_yaw;
    for (size_t j = 1; j < ret.size(); j++) {
      ret[j].w() = ret[j - 1].w() + yaw_step;
    }
  }


  //}

  return ret;
}
//}

/* waypointsToPathSrv //{ */
std::shared_ptr<fog_msgs::srv::Path::Request> Navigation::waypointsToPathSrv(const std::vector<Eigen::Vector4d> &waypoints, bool use_first) {
  nav_msgs::msg::Path msg;
  msg.header.stamp    = this->get_clock()->now();
  msg.header.frame_id = parent_frame_;
  size_t i            = 0;
  if (!use_first) {
    ++i;
  }
  while (i < waypoints.size()) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x  = waypoints[i].x();
    p.pose.position.y  = waypoints[i].y();
    p.pose.position.z  = waypoints[i].z();
    p.pose.orientation = yawToQuaternionMsg(waypoints[i].w());
    msg.poses.push_back(p);
    i++;
  }
  auto path_srv  = std::make_shared<fog_msgs::srv::Path::Request>();
  path_srv->path = msg;
  return path_srv;
}
//}

/* hover //{ */
void Navigation::hover() {
  std::vector<Eigen::Vector4d> waypoints;
  waypoints.push_back(desired_pose_);
  auto waypoints_srv = waypointsToPathSrv(waypoints, true);
  auto call_result   = local_path_client_->async_send_request(waypoints_srv);
}
//}

/* publishDiagnostics //{ */
void Navigation::publishDiagnostics() {
  fog_msgs::msg::NavigationDiagnostics msg;
  msg.header.stamp        = this->get_clock()->now();
  msg.header.frame_id     = parent_frame_;
  msg.state               = STATUS_STRING[status_];
  msg.current_waypoint_status = WAYPOINT_STATUS_STRING[waypoint_status_];
  msg.waypoints_in_buffer = waypoint_in_buffer_.size();
  msg.bumper_active       = bumper_active_;
  msg.current_waypoint_id = current_waypoint_id_;
  msg.current_nav_goal[0] = current_goal_.x();
  msg.current_nav_goal[1] = current_goal_.y();
  msg.current_nav_goal[2] = current_goal_.z();
  msg.last_nav_goal[0]    = last_goal_.x();
  msg.last_nav_goal[1]    = last_goal_.y();
  msg.last_nav_goal[2]    = last_goal_.z();
  diagnostics_publisher_->publish(msg);
}
//}

/* publishFutureTrajectory //{ */
void Navigation::publishFutureTrajectory(std::vector<Eigen::Vector4d> waypoints) {
  fog_msgs::msg::FutureTrajectory msg;
  msg.header.stamp    = this->get_clock()->now();
  msg.header.frame_id = parent_frame_;
  for (const auto &w : waypoints) {
    fog_msgs::msg::Vector4Stamped v;
    v.x               = w.x();
    v.y               = w.y();
    v.z               = w.z();
    v.w               = w.w();
    v.header.frame_id = parent_frame_;
    msg.poses.push_back(v);
  }
  future_trajectory_publisher_->publish(msg);
}
//}

/* waypointFutureCallback //{ */
bool Navigation::waypointFutureCallback(rclcpp::Client<fog_msgs::srv::WaypointToLocal>::SharedFuture future) {
  std::shared_ptr<fog_msgs::srv::WaypointToLocal_Response> result = future.get();

  if (result->success) {
    RCLCPP_INFO(this->get_logger(), "[%s]: Coordinate transform returned: %.2f, %.2f", this->get_name(), result->local_x, result->local_y);

    Eigen::Vector4d point;
    point[0] = result->local_x;
    point[1] = result->local_y;
    point[2] = result->local_z;
    point[3] = result->yaw;

    if (point.z() < min_altitude_) {
      RCLCPP_WARN(this->get_logger(), "[%s]: Goal Z coordinate (%.2f) is below the minimum allowed altitude (%.2f)", this->get_name(), point.z(),
                  min_altitude_);
      return false;
    }

    if (point.z() > max_altitude_) {
      RCLCPP_WARN(this->get_logger(), "[%s]: Goal Z coordinate (%.2f) is above the maximum allowed altitude (%.2f)", this->get_name(), point.z(),
                  max_altitude_);
      return false;
    }

    if ((point.head<3>() - desired_pose_.head<3>()).norm() > max_goal_distance_) {
      RCLCPP_WARN(this->get_logger(), "[%s]: Distance to goal (%.2f) exceeds the maximum allowed distance (%.2f m)", this->get_name(),
                  (point.head<3>() - desired_pose_.head<3>()).norm(), max_goal_distance_);
      return false;
    }

    waypoint_in_buffer_.push_back(point);

    RCLCPP_INFO(this->get_logger(), "[%s]: Waypoint added (LOCAL): %.2f, %.2f, %.2f, %.2f", this->get_name(), point.x(), point.y(), point.z(), point.w());

    RCLCPP_INFO(this->get_logger(), "[%s]: Planning started", this->get_name());
    status_ = PLANNING;

    return true;
  }
  return false;
}
//}

/* pathFutureCallback //{ */
bool Navigation::pathFutureCallback(rclcpp::Client<fog_msgs::srv::PathToLocal>::SharedFuture future) {
  std::shared_ptr<fog_msgs::srv::PathToLocal_Response> result = future.get();

  if (result->success) {
    RCLCPP_INFO(this->get_logger(), "[%s]: Coordinate transform returned %ld points", this->get_name(), result->path.poses.size());

    for (auto &pose : result->path.poses) {

      Eigen::Vector4d point;
      point[0] = pose.pose.position.x;
      point[1] = pose.pose.position.y;
      point[2] = pose.pose.position.z;
      point[3] = getYaw(pose.pose.orientation);

      if (point.z() < min_altitude_) {
        RCLCPP_WARN(this->get_logger(), "[%s]: Goal Z coordinate (%.2f) is below the minimum allowed altitude (%.2f)", this->get_name(), point.z(),
                    min_altitude_);
        return false;
      }

      if (point.z() > max_altitude_) {
        RCLCPP_WARN(this->get_logger(), "[%s]: Goal Z coordinate (%.2f) is above the maximum allowed altitude (%.2f)", this->get_name(), point.z(),
                    max_altitude_);
        return false;
      }

      if ((point.head<3>() - desired_pose_.head<3>()).norm() > max_goal_distance_) {
        RCLCPP_WARN(this->get_logger(), "[%s]: Distance to goal (%.2f) exceeds the maximum allowed distance (%.2f m)", this->get_name(),
                    (point.head<3>() - desired_pose_.head<3>()).norm(), max_goal_distance_);
        return false;
      }

      waypoint_in_buffer_.push_back(point);
      RCLCPP_INFO(this->get_logger(), "[%s]: Waypoint added (LOCAL): %.2f, %.2f, %.2f, %.2f", this->get_name(), point.x(), point.y(), point.z(), point.w());
    }

    RCLCPP_INFO(this->get_logger(), "[%s]: Planning started", this->get_name());
    status_ = PLANNING;

    return true;
  }
  return false;
}
//}

/* visualization //{ */

/* visualizeTree //{ */
void Navigation::visualizeTree(const octomap::OcTree &tree) {
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Visualizing tree", this->get_name());
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id    = parent_frame_;
  msg.header.stamp       = this->get_clock()->now();
  msg.ns                 = "tree";
  msg.type               = visualization_msgs::msg::Marker::POINTS;
  msg.id                 = 8;
  msg.action             = visualization_msgs::msg::Marker::ADD;
  msg.pose.orientation.w = 1.0;
  msg.scale.x            = tree_points_scale_;
  msg.scale.y            = tree_points_scale_;

  for (auto it = tree.begin(); it != tree.end(); it++) {
    if (it->getValue() == TreeValue::OCCUPIED) {
      geometry_msgs::msg::Point gp;
      auto                      color = generateColor(0, 0, 0, 1.0);
      gp.x                            = it.getX();
      gp.y                            = it.getY();
      gp.z                            = it.getZ();
      msg.points.push_back(gp);
      msg.colors.push_back(color);
    } else if (show_unoccupied_) {
      geometry_msgs::msg::Point gp;
      auto                      color = generateColor(0.7, 0.7, 0.7, 0.7);
      gp.x                            = it.getX();
      gp.y                            = it.getY();
      gp.z                            = it.getZ();
      msg.points.push_back(gp);
      msg.colors.push_back(color);
    }
  }
  binary_tree_publisher_->publish(msg);
}
//}

/* visualizeExpansions //{ */
void Navigation::visualizeExpansions(const std::unordered_set<navigation::Node, HashFunction>  open,
                                     const std::unordered_set<navigation::Node, HashFunction> &closed, const octomap::OcTree &tree) {
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Visualizing open planning expansions", this->get_name());
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id    = parent_frame_;
  msg.header.stamp       = this->get_clock()->now();
  msg.ns                 = "expansions";
  msg.type               = visualization_msgs::msg::Marker::POINTS;
  msg.id                 = 8;
  msg.action             = visualization_msgs::msg::Marker::ADD;
  msg.pose.orientation.w = 1.0;
  msg.scale.x            = expansions_points_scale_;
  msg.scale.y            = expansions_points_scale_;

  double max_cost = 0;
  double min_cost = 1e6;

  for (auto it = open.begin(); it != open.end(); it++) {
    if (it->total_cost > max_cost) {
      max_cost = it->total_cost;
    }
    if (it->total_cost < min_cost) {
      min_cost = it->total_cost;
    }
  }

  for (auto it = open.begin(); it != open.end(); it++) {
    auto                      coords = tree.keyToCoord(it->key);
    geometry_msgs::msg::Point gp;
    double                    brightness = (it->total_cost - min_cost) / (max_cost - min_cost);
    auto                      color      = generateColor(0.0, brightness, 0.3, 0.8);
    gp.x                                 = coords.x();
    gp.y                                 = coords.y();
    gp.z                                 = coords.z();
    msg.points.push_back(gp);
    msg.colors.push_back(color);
  }

  for (auto it = closed.begin(); it != closed.end(); it++) {
    auto                      coords = tree.keyToCoord(it->key);
    geometry_msgs::msg::Point gp;
    auto                      color = generateColor(0.8, 0.0, 0, 0.8);
    gp.x                            = coords.x();
    gp.y                            = coords.y();
    gp.z                            = coords.z();
    msg.points.push_back(gp);
    msg.colors.push_back(color);
  }
  expansion_publisher_->publish(msg);
}
//}

/* visualizePath //{ */
void Navigation::visualizePath(const std::vector<Eigen::Vector4d> &waypoints) {
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Visualizing path", this->get_name());
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id    = parent_frame_;
  msg.header.stamp       = this->get_clock()->now();
  msg.ns                 = "path";
  msg.type               = visualization_msgs::msg::Marker::LINE_STRIP;
  msg.id                 = 8;
  msg.action             = visualization_msgs::msg::Marker::ADD;
  msg.pose.orientation.w = 1.0;
  msg.scale.x            = path_points_scale_;

  for (size_t i = 1; i < waypoints.size(); i++) {
    geometry_msgs::msg::Point p1, p2;
    std_msgs::msg::ColorRGBA  c;
    p1.x = waypoints[i - 1].x();
    p1.y = waypoints[i - 1].y();
    p1.z = waypoints[i - 1].z();
    p2.x = waypoints[i].x();
    p2.y = waypoints[i].y();
    p2.z = waypoints[i].z();
    c    = generateColor(0.1, double(i) / double(waypoints.size()), 0.1, 1);
    msg.points.push_back(p1);
    msg.points.push_back(p2);
    msg.colors.push_back(c);
    msg.colors.push_back(c);
  }
  path_publisher_->publish(msg);
}
//}

/* visualizeGoals //{ */
void Navigation::visualizeGoals(const std::deque<Eigen::Vector4d> &waypoints) {
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Visualizing goals", this->get_name());
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id    = parent_frame_;
  msg.header.stamp       = this->get_clock()->now();
  msg.ns                 = "goals";
  msg.type               = visualization_msgs::msg::Marker::POINTS;
  msg.id                 = 8;
  msg.action             = visualization_msgs::msg::Marker::ADD;
  msg.pose.orientation.w = 1.0;
  msg.scale.x            = goal_points_scale_;
  msg.scale.y            = goal_points_scale_;

  for (size_t id = 0; id < waypoints.size(); id++) {
    const Eigen::Vector4d    w = waypoints[id];
    std_msgs::msg::ColorRGBA c;

    if (id == current_waypoint_id_) {
      c = generateColor(0.3, 0.5, 1.0, 1.0);
    } else {
      c = generateColor(0.1, 0.3, 0.7, 1.0);
    }

    geometry_msgs::msg::Point p;
    p.x = w.x();
    p.y = w.y();
    p.z = w.z();
    msg.points.push_back(p), w.w();
    msg.colors.push_back(c);
  }
  goal_publisher_->publish(msg);
}
//}

/* generateColor//{ */
std_msgs::msg::ColorRGBA Navigation::generateColor(const double r, const double g, const double b, const double a) {
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
bool Navigation::parse_param(const std::string &param_name, T &param_dest) {
  this->declare_parameter(param_name);
  if (!this->get_parameter(param_name, param_dest)) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Could not load param '%s'", this->get_name(), param_name.c_str());
    return false;
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "[" << this->get_name() << "]: Loaded '" << param_name << "' = '" << param_dest << "'");
  }
  return true;
}

/* template bool Navigation::parse_param<int>(const std::string &param_name, int &param_dest); */
/* template bool Navigation::parse_param<double>(const std::string &param_name, double &param_dest); */
/* template bool Navigation::parse_param<float>(const std::string &param_name, float &param_dest); */
/* template bool Navigation::parse_param<std::string>(const std::string &param_name, std::string &param_dest); */
/* template bool Navigation::parse_param<bool>(const std::string &param_name, bool &param_dest); */
/* template bool Navigation::parse_param<unsigned int>(const std::string &param_name, unsigned int &param_dest); */
//}

}  // namespace navigation
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(navigation::Navigation)
