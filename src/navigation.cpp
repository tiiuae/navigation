// clang: MatousFormat

// change the default Eigen formatting for prettier printing (has to be done before including Eigen)
#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "[", "]")

/* includes //{ */
#include <chrono>
#include <fog_msgs/srv/path.hpp>
#include <fog_msgs/srv/vec4.hpp>
#include <fog_msgs/msg/future_trajectory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <deque>
#include <future>
#include <mutex>
#include <shared_mutex>
#include <octomap/math/Vector3.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
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
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <limits>
#include <tuple>

#include <fog_msgs/msg/control_interface_diagnostics.hpp>
#include <fog_msgs/msg/navigation_diagnostics.hpp>
#include <fog_msgs/msg/obstacle_sectors.hpp>
#include <fog_msgs/srv/waypoint_to_local.hpp>
#include <fog_msgs/srv/path_to_local.hpp>
#include <fog_msgs/action/control_interface_action.hpp>
#include <fog_msgs/action/navigation_action.hpp>

#include <control_interface/enums.h>
#include <fog_lib/mutex_utils.h>
#include <fog_lib/params.h>
#include <fog_lib/misc.h>
#include <fog_lib/geometry/cyclic.h>
#include <fog_lib/geometry/misc.h>

#include "navigation/enums.h"
#include "navigation/astar_planner.hpp"

//}

using namespace std::placeholders;
using namespace fog_lib;
using namespace fog_lib::geometry;
constexpr double nand = std::numeric_limits<double>::quiet_NaN();

namespace navigation
{
  using vec3_t = Eigen::Vector3d;
  using vec4_t = Eigen::Vector4d;
  using quat_t = Eigen::Quaterniond;
  using anax_t = Eigen::AngleAxisd;
  using control_diag_msg_t = fog_msgs::msg::ControlInterfaceDiagnostics;
  using vehicle_state_t = control_interface::vehicle_state_t;
  using mission_state_t = control_interface::mission_state_t;
  using ControlAction = fog_msgs::action::ControlInterfaceAction;
  using ControlActionClient = rclcpp_action::Client<ControlAction>;
  using ControlGoalHandle = rclcpp_action::ClientGoalHandle<ControlAction>;

  std::string toupper(std::string s)
  {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::toupper(c); });
    return s;
  }

  /* class Navigation //{ */
  class Navigation : public rclcpp::Node
  {
    using NavigationAction = fog_msgs::action::NavigationAction;
    using NavigationGoalHandle = rclcpp_action::ServerGoalHandle<NavigationAction>;
    const std::shared_ptr<NavigationGoalHandle> no_goal = nullptr;

  public:
    Navigation(rclcpp::NodeOptions options);

  private:
    // flags set in callbacks
    std::shared_mutex control_diags_mutex_;
    bool getting_control_diagnostics_ = false;
    vehicle_state_t control_vehicle_state_;
    mission_state_t control_mission_state_;
    uint32_t control_command_id_ = 0;
    uint32_t control_response_id_ = 0;

    std::shared_mutex uav_pose_mutex_;
    bool getting_uav_pose_ = false;
    vec4_t uav_pose_;

    std::shared_mutex cmd_pose_mutex_;
    bool getting_cmd_pose_ = false;
    vec4_t cmd_pose_;

    std::shared_mutex octree_mutex_;
    bool getting_octomap_ = false;
    std::string octree_frame_;
    std::shared_ptr<octomap::OcTree> octree_;

    std::atomic<bool> is_initialized_ = false;

    // control_interface action client
    std::recursive_mutex control_ac_mutex_;
    ControlActionClient::SharedPtr control_ac_ptr_;
    std::shared_future<ControlGoalHandle::SharedPtr> control_goal_handle_future_;
    std::shared_future<ControlGoalHandle::WrappedResult> control_goal_result_future_;

    // bumper-related variables
    std::shared_mutex bumper_mutex_;
    std::shared_ptr<fog_msgs::msg::ObstacleSectors> bumper_msg_ = nullptr;
    bool bumper_active_ = false;
    bool bumper_obstacle_detected_ = false;

    std::shared_mutex state_mutex_;
    nav_state_t state_ = nav_state_t::not_ready;

    std::recursive_mutex waypoints_mutex_;
    std::deque<vec4_t> waypoints_in_;
    size_t waypoint_current_it_ = 0;
    waypoint_state_t waypoint_state_ = waypoint_state_t::empty;
    int replanning_counter_ = 0;

    rclcpp::TimerBase::SharedPtr execution_timer_;
    void navigationRoutine();

    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    void diagnosticsRoutine();

    void state_navigation_common();
    void state_navigation_not_ready();
    void state_navigation_idle();
    void state_navigation_planning();
    void state_navigation_commanding();
    void state_navigation_moving();
    void state_navigation_avoiding();

    std::pair<std::vector<vec4_t>, bool> planPath(const vec4_t& goal, std::shared_ptr<octomap::OcTree> mapping_tree);

    // action server
    std::recursive_mutex action_server_mutex_;
    rclcpp_action::Server<NavigationAction>::SharedPtr action_server_;
    std::shared_ptr<NavigationGoalHandle> action_server_goal_handle_ = nullptr;
    uint32_t mission_id_ = 0;

    // action server methods
    rclcpp_action::GoalResponse actionServerHandleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const NavigationAction::Goal> goal);
    rclcpp_action::CancelResponse actionServerHandleCancel(const std::shared_ptr<NavigationGoalHandle> goal_handle);
    void actionServerHandleAccepted(const std::shared_ptr<NavigationGoalHandle> goal_handle);

    // params
    float euclidean_distance_cutoff_;
    float safe_obstacle_distance_;
    float navigation_tolerance_;
    bool unknown_is_occupied_;
    float min_altitude_;
    float max_altitude_;
    float ground_cutoff_;
    float max_goal_distance_;
    float distance_penalty_;
    float greedy_penalty_;
    float planning_tree_resolution_;
    float max_waypoint_distance_;
    float max_heading_step_;
    float planning_timeout_;
    float replanning_distance_;
    int replanning_limit_;
    float altitude_acceptance_radius_;
    double main_update_rate_;
    double diagnostics_rate_;

    bool bumper_enabled_;
    double bumper_distance_factor_;
    rclcpp::Duration bumper_min_replan_period_ = rclcpp::Duration(std::chrono::milliseconds(500));

    // visualization params
    bool visualize_planner_ = true;
    bool show_unoccupied_ = false;
    bool override_previous_commands_ = false;
    double tree_points_scale_;
    double expansions_points_scale_;
    double path_points_scale_;
    double goal_points_scale_;

    // publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr binary_tree_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr expansion_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_publisher_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Publisher<fog_msgs::msg::FutureTrajectory>::SharedPtr future_trajectory_publisher_;
    rclcpp::Publisher<fog_msgs::msg::NavigationDiagnostics>::SharedPtr diagnostics_publisher_;

    // subscribers
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_pose_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr goto_subscriber_;
    rclcpp::Subscription<fog_msgs::msg::ControlInterfaceDiagnostics>::SharedPtr control_diagnostics_subscriber_;
    rclcpp::Subscription<fog_msgs::msg::ObstacleSectors>::SharedPtr bumper_subscriber_;

    // subscriber callbacks
    void octomapCallback(const octomap_msgs::msg::Octomap::UniquePtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg);
    void cmdPoseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr msg);
    void controlDiagnosticsCallback(const fog_msgs::msg::ControlInterfaceDiagnostics::UniquePtr msg);
    void bumperCallback(const fog_msgs::msg::ObstacleSectors::UniquePtr msg);

    // services provided
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hover_service_;

    rclcpp::Service<fog_msgs::srv::Vec4>::SharedPtr local_waypoint_service_;
    rclcpp::Service<fog_msgs::srv::Path>::SharedPtr local_path_service_;
    rclcpp::Service<fog_msgs::srv::Vec4>::SharedPtr gps_waypoint_service_;
    rclcpp::Service<fog_msgs::srv::Path>::SharedPtr gps_path_service_;

    rclcpp::Client<fog_msgs::srv::WaypointToLocal>::SharedPtr waypoint_to_local_client_;
    rclcpp::Client<fog_msgs::srv::PathToLocal>::SharedPtr path_to_local_client_;

    template <typename T>
    size_t addWaypoints(const T& path, const std::shared_ptr<NavigationGoalHandle> goal_handle, const bool override, std::string& fail_reason_out);
    void pathCallback(const nav_msgs::msg::Path::UniquePtr msg);

    // service callbacks
    bool hoverCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    bool localWaypointCallback(const std::shared_ptr<fog_msgs::srv::Vec4::Request> request, std::shared_ptr<fog_msgs::srv::Vec4::Response> response);
    bool localPathCallback(const std::shared_ptr<fog_msgs::srv::Path::Request> request, std::shared_ptr<fog_msgs::srv::Path::Response> response);
    bool gpsWaypointCallback(const std::shared_ptr<fog_msgs::srv::Vec4::Request> request, std::shared_ptr<fog_msgs::srv::Vec4::Response> response);
    bool gpsPathCallback(const std::shared_ptr<fog_msgs::srv::Path::Request> request, std::shared_ptr<fog_msgs::srv::Path::Response> response);

    // future callback
    void waypointFutureCallback(rclcpp::Client<fog_msgs::srv::WaypointToLocal>::SharedFuture future);
    void pathFutureCallback(rclcpp::Client<fog_msgs::srv::PathToLocal>::SharedFuture future, const std::shared_ptr<NavigationGoalHandle> goal_handle);

    // visualization
    void visualizeTree(const std::shared_ptr<octomap::OcTree> tree);
    void visualizeExpansions(const std::unordered_set<navigation::Node, HashFunction> open, const std::unordered_set<navigation::Node, HashFunction>& closed,
                             const std::shared_ptr<octomap::OcTree> tree);
    void visualizePath(const std::vector<vec4_t>& waypoints);
    void visualizeGoals(const std::deque<vec4_t>& waypoints);

    std::vector<vec4_t> resamplePath(const std::vector<octomap::point3d>& waypoints, const double end_heading) const;
    ControlAction::Goal waypointsToGoal(const std::vector<vec4_t>& waypoints);
    std::shared_future<ControlGoalHandle::SharedPtr> commandWaypoints(const std::vector<vec4_t>& waypoints);
    void hover();
    bool cancel_goal(const std::shared_ptr<NavigationGoalHandle> goal_handle, std::string& fail_reason_out);

    void finish_goal(const std::shared_ptr<NavigationGoalHandle> goal_handle);
    void abort_goal(const std::shared_ptr<NavigationGoalHandle> goal_handle, const std::string& reason);
    void update_goal(const std::shared_ptr<NavigationGoalHandle> goal_handle, const int current_waypoint, const int waypoints);

    void publishDiagnostics();
    void publishFutureTrajectory(const std::vector<vec4_t>& waypoints);

    // bumper
    bool bumperDataOld(const fog_msgs::msg::ObstacleSectors::SharedPtr bumper_msg);
    bool bumperCheckObstacles(const fog_msgs::msg::ObstacleSectors::SharedPtr bumper_msg);
    vec3_t bumperGetAvoidanceVector(const fog_msgs::msg::ObstacleSectors::SharedPtr bumper_msg);

    std_msgs::msg::ColorRGBA generateColor(const double r, const double g, const double b, const double a);

    std::vector<rclcpp::CallbackGroup::SharedPtr> callback_groups_;
    rclcpp::CallbackGroup::SharedPtr new_cbk_grp();
  };
  //}

  template <typename T>
  bool future_ready(const std::shared_future<T>& f);
  vec4_t to_eigen(const vec4_t& vec);
  vec3_t to_eigen(const octomath::Vector3& vec);
  vec4_t to_eigen(const octomath::Vector3& vec, const double heading);
  vec4_t to_eigen(const geometry_msgs::msg::PoseStamped& pose);
  octomap::point3d toPoint3d(const vec4_t& vec);

  /* constructor //{ */
  Navigation::Navigation(rclcpp::NodeOptions options) : Node("navigation", options)
  {

    RCLCPP_INFO(get_logger(), "Initializing...");

    /* parse params from config file //{ */
    RCLCPP_INFO(get_logger(), "-------------- Loading parameters --------------");
    bool loaded_successfully = true;
    loaded_successfully &= parse_param("planning.euclidean_distance_cutoff", euclidean_distance_cutoff_, *this);
    loaded_successfully &= parse_param("planning.safe_obstacle_distance", safe_obstacle_distance_, *this);
    loaded_successfully &= parse_param("planning.unknown_is_occupied", unknown_is_occupied_, *this);
    loaded_successfully &= parse_param("planning.navigation_tolerance", navigation_tolerance_, *this);
    loaded_successfully &= parse_param("planning.min_altitude", min_altitude_, *this);
    loaded_successfully &= parse_param("planning.max_altitude", max_altitude_, *this);
    loaded_successfully &= parse_param("planning.ground_cutoff", ground_cutoff_, *this);
    loaded_successfully &= parse_param("planning.max_goal_distance", max_goal_distance_, *this);
    loaded_successfully &= parse_param("planning.distance_penalty", distance_penalty_, *this);
    loaded_successfully &= parse_param("planning.greedy_penalty", greedy_penalty_, *this);
    loaded_successfully &= parse_param("planning.planning_tree_resolution", planning_tree_resolution_, *this);
    loaded_successfully &= parse_param("planning.max_waypoint_distance", max_waypoint_distance_, *this);
    loaded_successfully &= parse_param("planning.max_heading_step", max_heading_step_, *this);
    loaded_successfully &= parse_param("planning.planning_timeout", planning_timeout_, *this);
    loaded_successfully &= parse_param("planning.replanning_limit", replanning_limit_, *this);
    loaded_successfully &= parse_param("planning.replanning_distance", replanning_distance_, *this);
    loaded_successfully &= parse_param("planning.override_previous_commands", override_previous_commands_, *this);
    loaded_successfully &= parse_param("planning.main_update_rate", main_update_rate_, *this);
    loaded_successfully &= parse_param("planning.diagnostics_rate", diagnostics_rate_, *this);

    loaded_successfully &= parse_param("visualization.visualize_planner", visualize_planner_, *this);
    loaded_successfully &= parse_param("visualization.show_unoccupied", show_unoccupied_, *this);
    loaded_successfully &= parse_param("visualization.tree_points_scale", tree_points_scale_, *this);
    loaded_successfully &= parse_param("visualization.expansions_points_scale", expansions_points_scale_, *this);
    loaded_successfully &= parse_param("visualization.path_points_scale", path_points_scale_, *this);
    loaded_successfully &= parse_param("visualization.goal_points_scale", goal_points_scale_, *this);

    loaded_successfully &= parse_param("bumper.enabled", bumper_enabled_, *this);
    loaded_successfully &= parse_param("bumper.distance_factor", bumper_distance_factor_, *this);
    loaded_successfully &= parse_param("bumper.min_replan_period", bumper_min_replan_period_, *this);

    // loaded from control_interface
    loaded_successfully &= parse_param("px4.altitude_acceptance_radius", altitude_acceptance_radius_, *this);

    if (!loaded_successfully)
    {
      const std::string str = "Could not load all non-optional parameters. Shutting down.";
      RCLCPP_ERROR_STREAM(get_logger(), str);
      rclcpp::shutdown();
      return;
    }
    //}

    // | ------------------ initialize publishers ----------------- |
    rclcpp::QoS qos(rclcpp::KeepLast(3));
    binary_tree_publisher_ = create_publisher<visualization_msgs::msg::Marker>("~/binary_tree_markers_out", qos);
    expansion_publisher_ = create_publisher<visualization_msgs::msg::Marker>("~/expansion_markers_out", qos);
    path_publisher_ = create_publisher<visualization_msgs::msg::Marker>("~/path_markers_out", qos);
    goal_publisher_ = create_publisher<visualization_msgs::msg::Marker>("~/goal_markers_out", qos);
    status_publisher_ = create_publisher<std_msgs::msg::String>("~/status_out", 1);
    future_trajectory_publisher_ = create_publisher<fog_msgs::msg::FutureTrajectory>("~/future_trajectory_out", qos);
    diagnostics_publisher_ = create_publisher<fog_msgs::msg::NavigationDiagnostics>("~/diagnostics_out", qos);

    // service clients
    waypoint_to_local_client_ = create_client<fog_msgs::srv::WaypointToLocal>("~/waypoint_to_local_out");
    path_to_local_client_ = create_client<fog_msgs::srv::PathToLocal>("~/path_to_local_out");

    // control interface action server
    control_ac_ptr_ = rclcpp_action::create_client<ControlAction>(get_node_base_interface(), get_node_graph_interface(), get_node_logging_interface(),
                                                                  get_node_waitables_interface(), "control_interface");

    // | ------------------ initialize callbacks ------------------ |
    rclcpp::SubscriptionOptions subopts;

    subopts.callback_group = new_cbk_grp();
    odometry_subscriber_ =
        create_subscription<nav_msgs::msg::Odometry>("~/odometry_in", rclcpp::SystemDefaultsQoS(), std::bind(&Navigation::odometryCallback, this, _1), subopts);

    subopts.callback_group = new_cbk_grp();
    cmd_pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>("~/cmd_pose_in", rclcpp::SystemDefaultsQoS(),
                                                                                std::bind(&Navigation::cmdPoseCallback, this, _1), subopts);

    subopts.callback_group = new_cbk_grp();
    goto_subscriber_ =
        create_subscription<nav_msgs::msg::Path>("~/goto_in", rclcpp::SystemDefaultsQoS(), std::bind(&Navigation::pathCallback, this, _1), subopts);

    subopts.callback_group = new_cbk_grp();
    control_diagnostics_subscriber_ = create_subscription<fog_msgs::msg::ControlInterfaceDiagnostics>(
        "~/control_diagnostics_in", rclcpp::SystemDefaultsQoS(), std::bind(&Navigation::controlDiagnosticsCallback, this, _1), subopts);

    subopts.callback_group = new_cbk_grp();
    octomap_subscriber_ = create_subscription<octomap_msgs::msg::Octomap>("~/octomap_in", rclcpp::SystemDefaultsQoS(),
                                                                          std::bind(&Navigation::octomapCallback, this, _1), subopts);

    subopts.callback_group = new_cbk_grp();
    bumper_subscriber_ = create_subscription<fog_msgs::msg::ObstacleSectors>("~/bumper_in", rclcpp::SystemDefaultsQoS(),
                                                                             std::bind(&Navigation::bumperCallback, this, _1), subopts);

    // service handlers
    const auto qos_profile = qos.get_rmw_qos_profile();
    const auto svc_grp_ptr = new_cbk_grp();
    hover_service_ = create_service<std_srvs::srv::Trigger>("~/hover_in", std::bind(&Navigation::hoverCallback, this, _1, _2), qos_profile, svc_grp_ptr);

    local_waypoint_service_ =
        create_service<fog_msgs::srv::Vec4>("~/local_waypoint_in", std::bind(&Navigation::localWaypointCallback, this, _1, _2), qos_profile, svc_grp_ptr);

    local_path_service_ =
        create_service<fog_msgs::srv::Path>("~/local_path_in", std::bind(&Navigation::localPathCallback, this, _1, _2), qos_profile, svc_grp_ptr);

    gps_waypoint_service_ =
        create_service<fog_msgs::srv::Vec4>("~/gps_waypoint_in", std::bind(&Navigation::gpsWaypointCallback, this, _1, _2), qos_profile, svc_grp_ptr);

    gps_path_service_ = create_service<fog_msgs::srv::Path>("~/gps_path_in", std::bind(&Navigation::gpsPathCallback, this, _1, _2), qos_profile, svc_grp_ptr);

    // timers
    execution_timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / main_update_rate_), std::bind(&Navigation::navigationRoutine, this), new_cbk_grp());

    diagnostics_timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / diagnostics_rate_), std::bind(&Navigation::diagnosticsRoutine, this), new_cbk_grp());

    action_server_ = rclcpp_action::create_server<NavigationAction>(
        get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(), get_node_waitables_interface(), "navigation",
        std::bind(&Navigation::actionServerHandleGoal, this, _1, _2), std::bind(&Navigation::actionServerHandleCancel, this, _1),
        std::bind(&Navigation::actionServerHandleAccepted, this, _1), rcl_action_server_get_default_options(), new_cbk_grp());

    if (max_waypoint_distance_ <= 0)
      max_waypoint_distance_ = replanning_distance_;

    is_initialized_ = true;
    RCLCPP_INFO(get_logger(), "Initialized");
  }
  //}

  // --------------------------------------------------------------
  // |                          Callbacks                         |
  // --------------------------------------------------------------

  // | --------------------- Input callbacks -------------------- |

  /* octomapCallback //{ */
  void Navigation::octomapCallback(const octomap_msgs::msg::Octomap::UniquePtr msg)
  {
    const auto tree_ptr = octomap_msgs::fullMsgToMap(*msg);
    if (tree_ptr)
    {
      const auto new_octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(tree_ptr));
      set_mutexed(octree_mutex_, std::make_tuple(new_octree, true, msg->header.frame_id), std::forward_as_tuple(octree_, getting_octomap_, octree_frame_));
      RCLCPP_INFO_ONCE(get_logger(), "Getting octomap");
    } else
    {
      RCLCPP_WARN(get_logger(), "Octomap message is empty!");
    }
  }
  //}

  /* odometryCallback //{ */
  void Navigation::odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg)
  {
    // ignore invalid messages
    if (has_nans(msg->pose.pose))
    {
      // only warn if we'd expect a valid message
      const auto state = get_mutexed(state_mutex_, state_);
      if (state != nav_state_t::not_ready)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Received uav pose contains NaNs, ignoring! Position: [%.2f, %.2f, %.2f], orientation [%.2f, %.2f, %.2f, %.2f].",
                             msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      }
      return;
    }

    const vec4_t uav_pose(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, quat2heading(msg->pose.pose.orientation));
    set_mutexed(uav_pose_mutex_, std::make_tuple(uav_pose, true), std::forward_as_tuple(uav_pose_, getting_uav_pose_));
    RCLCPP_INFO_ONCE(get_logger(), "Getting odometry");
  }
  //}

  /* cmdPoseCallback //{ */
  void Navigation::cmdPoseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr msg)
  {
    // ignore invalid messages
    if (has_nans(msg->pose))
    {
      // only warn if we'd expect a valid message
      const auto state = get_mutexed(state_mutex_, state_);
      if (state != nav_state_t::not_ready)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Received cmd pose contains NaNs, ignoring! Position: [%.2f, %.2f, %.2f], orientation [%.2f, %.2f, %.2f, %.2f].",
                             msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.x, msg->pose.orientation.y,
                             msg->pose.orientation.z, msg->pose.orientation.w);
      }
      return;
    }

    const vec4_t cmd_pose(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, quat2heading(msg->pose.orientation));
    set_mutexed(cmd_pose_mutex_, std::make_tuple(cmd_pose, true), std::forward_as_tuple(cmd_pose_, getting_cmd_pose_));
    RCLCPP_INFO_ONCE(get_logger(), "Getting cmd pose");
  }
  //}

  /* controlDiagnosticsCallback //{ */
  void Navigation::controlDiagnosticsCallback(const fog_msgs::msg::ControlInterfaceDiagnostics::UniquePtr msg)
  {
    const auto vehicle_state = control_interface::to_enum(msg->vehicle_state);
    const auto mission_state = control_interface::to_enum(msg->mission_state);
    set_mutexed(control_diags_mutex_, std::make_tuple(vehicle_state, mission_state, true, msg->mission_progress.mission_id),
                std::forward_as_tuple(control_vehicle_state_, control_mission_state_, getting_control_diagnostics_, control_response_id_));
    RCLCPP_INFO_ONCE(get_logger(), "Getting control_interface diagnostics");
  }
  //}

  /* bumperCallback //{ */
  void Navigation::bumperCallback(const fog_msgs::msg::ObstacleSectors::UniquePtr msg)
  {
    const auto bumper_msg = std::make_shared<fog_msgs::msg::ObstacleSectors>(*msg);
    set_mutexed(bumper_mutex_, bumper_msg, bumper_msg_);
    RCLCPP_INFO_ONCE(get_logger(), "Getting bumper msgs");
  }
  //}

  // | ----------------- Action server callbacks ---------------- |

  /* actionServerHandleGoal //{ */
  // note: to accept or reject goals sent to the server
  // callback must be non-blocking
  rclcpp_action::GoalResponse Navigation::actionServerHandleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const NavigationAction::Goal> goal)
  {
    const auto state = get_mutexed(state_mutex_, state_);

    if (state == nav_state_t::not_ready)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Goal " << rclcpp_action::to_string(uuid) << " rejected: node is not initialized: " << to_string(state));
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->path.poses.empty())
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Goal " << rclcpp_action::to_string(uuid) << " rejected: path input does not contain any waypoints");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  //}

  /* actionServerHandleCancel //{ */
  // note: to accept or reject requests to cancel a goal
  // callback must be non-blocking
  rclcpp_action::CancelResponse Navigation::actionServerHandleCancel(const std::shared_ptr<NavigationGoalHandle> goal_handle)
  {
    const auto state = get_mutexed(state_mutex_, state_);

    if (state == nav_state_t::idle)
    {
      RCLCPP_WARN(get_logger(), "No goal to cancel, vehicle is idle");
      return rclcpp_action::CancelResponse::REJECT;
    }

    if (state == nav_state_t::not_ready)
    {
      RCLCPP_WARN(get_logger(), "Cannot cancel goal, navigation is not initialized");
      return rclcpp_action::CancelResponse::REJECT;
    }

    std::scoped_lock lock(action_server_mutex_);
    // check if we received cancel of the goal that we are currently executing
    if (action_server_goal_handle_->get_goal_id() != goal_handle->get_goal_id())
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "The target goal is not active (active goal: "
                                                 << rclcpp_action::to_string(action_server_goal_handle_->get_goal_id())
                                                 << ", requested: " << rclcpp_action::to_string(goal_handle->get_goal_id()) << "). CANCEL rejected.");
      return rclcpp_action::CancelResponse::REJECT;
    }

    RCLCPP_WARN_STREAM(this->get_logger(), "CANCEL of goal " << rclcpp_action::to_string(goal_handle->get_goal_id()) << " accepted. Attempting hovering.");
    // let another thread handle the cancelling in parallel
    std::thread([this, goal_handle]() {
      std::scoped_lock lock(action_server_mutex_, state_mutex_, waypoints_mutex_, control_ac_mutex_);
      std::string reason;
      if (cancel_goal(goal_handle, reason))
        RCLCPP_INFO_STREAM(get_logger(), "Goal " << rclcpp_action::to_string(goal_handle->get_goal_id()) << " canceled. Hovering");
      else
        RCLCPP_WARN_STREAM(get_logger(), "Goal " << rclcpp_action::to_string(goal_handle->get_goal_id()) << " cannot be canceled: " << reason);
    }).detach();
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  //}

  /* actionServerHandleAccepted //{ */
  // note: to receive a goal handle after a goal has been accepted
  // callback must be non-blocking
  void Navigation::actionServerHandleAccepted(const std::shared_ptr<NavigationGoalHandle> goal_handle)
  {
    std::scoped_lock lock(action_server_mutex_);

    // if the goal is not global, first request from ControlInterface to transform it to local coordinates
    if (!goal_handle->get_goal()->is_local)
    {
      auto convert_path_req = std::make_shared<fog_msgs::srv::PathToLocal::Request>();
      convert_path_req->path = goal_handle->get_goal()->path;
      rclcpp::Client<fog_msgs::srv::PathToLocal>::CallbackType cbk = std::bind(&Navigation::pathFutureCallback, this, std::placeholders::_1, goal_handle);
      path_to_local_client_->async_send_request(convert_path_req, cbk);
      RCLCPP_INFO_STREAM(get_logger(), "Requesting transformation of goal path from GPS to local coordinates");
      return;
    }

    // if it's already local, attempt to start the new mission
    std::string reason;
    const size_t added = addWaypoints(goal_handle->get_goal()->path.poses, goal_handle, override_previous_commands_, reason);
    if (added == 0)
    {
      reason = std::to_string(goal_handle->get_goal()->path.poses.size()) + " new waypoints not set: " + reason;
      abort_goal(goal_handle, reason);
      return;
    }

    RCLCPP_INFO_STREAM(get_logger(), "Queued " << added << " new waypoints for planning as goal"
                                               << rclcpp_action::to_string(action_server_goal_handle_->get_goal_id()) << ".");
  }
  //}

  // | -------------------- Command callbacks ------------------- |

  /* addWaypoints() method //{ */
  // the caller must NOT lock the following muteces or else a deadlock may happen:
  // state_mutex_
  // waypoints_mutex_
  // control_ac_mutex_
  template <typename T>
  size_t Navigation::addWaypoints(const T& path, const std::shared_ptr<NavigationGoalHandle> goal_handle, const bool override, std::string& fail_reason_out)
  {
    std::scoped_lock lock(state_mutex_, waypoints_mutex_, control_ac_mutex_);
    size_t added = 0;
    if (state_ != nav_state_t::idle)
    {
      if (override && state_ != nav_state_t::not_ready)
      {
        RCLCPP_INFO(get_logger(), "Overriding previous navigation commands");
        hover();  // clears previous waypoints and commands the drone to the cmd_pose_, which should stop it
      } else
      {
        fail_reason_out = "not idle! Current state is: " + to_string(state_);
        return added;
      }
    }

    // abort any current goal
    if (action_server_goal_handle_ != goal_handle && action_server_goal_handle_)
    {
      if (action_server_goal_handle_->is_active())
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "Previous goal is not terminated yet. ABORTING previous goal "
                                                   << rclcpp_action::to_string(action_server_goal_handle_->get_goal_id()) << ".");
        abort_goal(action_server_goal_handle_, "New goal received.");
      }
    }
    // and update the current active goal handle
    action_server_goal_handle_ = goal_handle;
    mission_id_++;

    const vec4_t cmd_pose = get_mutexed(cmd_pose_mutex_, cmd_pose_);

    RCLCPP_INFO(get_logger(), "Recieved %ld waypoints", path.size());
    for (const auto& p : path)
    {
      // convert the point to eigen and ensure that the fourth element (heading) is in the range of [-pi, pi]
      const vec4_t point = wrap_heading(to_eigen(p));

      if (point.z() < min_altitude_)
      {
        fail_reason_out = "path is below min. altitude";
        RCLCPP_WARN(get_logger(), "Goal Z coordinate (%.2f) is below the minimum allowed altitude (%.2f), not adding further waypoints", point.z(),
                    min_altitude_);
        break;
      }

      if (point.z() > max_altitude_)
      {
        fail_reason_out = "path is above max. altitude";
        RCLCPP_WARN(get_logger(), "Goal Z coordinate (%.2f) is above the maximum allowed altitude (%.2f), not adding further waypoints", point.z(),
                    max_altitude_);
        break;
      }

      if ((point.head<3>() - cmd_pose.head<3>()).norm() > max_goal_distance_)
      {
        fail_reason_out = "path beyond the max. goal distance";
        RCLCPP_WARN(get_logger(), "Distance to goal (%.2f) exceeds the maximum allowed distance (%.2f m), not adding further waypoints",
                    (point.head<3>() - cmd_pose.head<3>()).norm(), max_goal_distance_);
        break;
      }

      waypoints_in_.push_back(point);
      added++;
    }

    return added;
  }
  //}

  /* pathCallback //{ */
  void Navigation::pathCallback(const nav_msgs::msg::Path::UniquePtr msg)
  {
    if (msg->poses.empty())
    {
      RCLCPP_ERROR(get_logger(), "Path rejected: input does not contain any waypoints");
      return;
    }

    std::string reason;
    const size_t added = addWaypoints(msg->poses, no_goal, override_previous_commands_, reason);
    if (added == 0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Path rejected: " << reason);
      return;
    }

    RCLCPP_INFO(get_logger(), "Queued %ld new waypoints for planning.", added);
  }
  //}

  /* localPathCallback //{ */
  bool Navigation::localPathCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::Path::Request> request,
                                     std::shared_ptr<fog_msgs::srv::Path::Response> response)
  {
    if (request->path.poses.empty())
    {
      response->message = "Path rejected: path input does not contain any waypoints";
      response->success = false;
      RCLCPP_ERROR_STREAM(get_logger(), response->message);
      return true;
    }

    std::string reason;
    const size_t added = addWaypoints(request->path.poses, no_goal, override_previous_commands_, reason);
    if (added == 0)
    {
      response->message = "Path rejected: " + reason;
      response->success = false;
      RCLCPP_ERROR_STREAM(get_logger(), response->message);
      return true;
    }

    response->message = "Queued " + std::to_string(request->path.poses.size()) + " new waypoints for planning.";
    response->success = true;
    RCLCPP_INFO_STREAM(get_logger(), response->message);
    return true;
  }
  //}

  /* gpsPathCallback //{ */
  bool Navigation::gpsPathCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::Path::Request> request,
                                   std::shared_ptr<fog_msgs::srv::Path::Response> response)
  {
    const auto state = get_mutexed(state_mutex_, state_);

    if (state == nav_state_t::not_ready)
    {
      response->message = "Path rejected: node is not initialized";
      response->success = false;
      RCLCPP_ERROR_STREAM(get_logger(), response->message);
      return true;
    }

    if (request->path.poses.empty())
    {
      response->message = "Path rejected: path input does not contain any waypoints";
      response->success = false;
      RCLCPP_ERROR_STREAM(get_logger(), response->message);
      return true;
    }

    auto convert_path_req = std::make_shared<fog_msgs::srv::PathToLocal::Request>();
    convert_path_req->path = request->path;
    rclcpp::Client<fog_msgs::srv::PathToLocal>::CallbackType cbk = std::bind(&Navigation::pathFutureCallback, this, std::placeholders::_1, no_goal);
    path_to_local_client_->async_send_request(convert_path_req, cbk);

    response->message = "Requesting transformation of path from GPS to local coordinates";
    response->success = true;
    RCLCPP_INFO_STREAM(get_logger(), response->message);
    return true;
  }
  //}

  /* localWaypointCallback //{ */
  bool Navigation::localWaypointCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
                                         std::shared_ptr<fog_msgs::srv::Vec4::Response> response)
  {
    const auto state = get_mutexed(state_mutex_, state_);
    if (state == nav_state_t::not_ready)
    {
      response->message = "Waypoint rejected: node is not initialized";
      response->success = false;
      RCLCPP_ERROR_STREAM(get_logger(), response->message);
      return true;
    }

    const vec4_t point(request->goal.at(0), request->goal.at(1), request->goal.at(2), request->goal.at(3));
    const std::vector<vec4_t> path = {point};

    std::string reason;
    const size_t added = addWaypoints(path, no_goal, override_previous_commands_, reason);
    if (added == 0)
    {
      response->message = "Waypoint rejected: " + reason;
      response->success = false;
      RCLCPP_ERROR_STREAM(get_logger(), response->message);
      return true;
    }

    std::stringstream ss;
    ss << "Waypoint set: " << point.transpose() << ".";
    response->message = ss.str();
    response->success = true;
    RCLCPP_INFO_STREAM(get_logger(), response->message);
    return true;
  }
  //}

  /* gpsWaypointCallback //{ */
  bool Navigation::gpsWaypointCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
                                       std::shared_ptr<fog_msgs::srv::Vec4::Response> response)
  {
    const auto state = get_mutexed(state_mutex_, state_);
    if (state == nav_state_t::not_ready)
    {
      response->message = "Waypoint rejected: node is not initialized";
      response->success = false;
      RCLCPP_ERROR_STREAM(get_logger(), response->message);
      return true;
    }

    auto waypoint_convert_req = std::make_shared<fog_msgs::srv::WaypointToLocal::Request>();
    waypoint_convert_req->latitude_deg = request->goal.at(0);
    waypoint_convert_req->longitude_deg = request->goal.at(1);
    waypoint_convert_req->relative_altitude_m = request->goal.at(2);
    waypoint_convert_req->heading = sradians::wrap(request->goal.at(3));  // ensure that the heading is in the range [-pi, pi]
    waypoint_to_local_client_->async_send_request(waypoint_convert_req, std::bind(&Navigation::waypointFutureCallback, this, std::placeholders::_1));

    response->message = "Requesting transformation of waypoint from GPS to local coordinates";
    response->success = true;
    RCLCPP_INFO_STREAM(get_logger(), response->message);
    return true;
  }
  //}

  /* hoverCallback //{ */
  bool Navigation::hoverCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const auto state = get_mutexed(state_mutex_, state_);

    if (state == nav_state_t::idle)
    {
      response->message = "Hover not necessary, vehicle is already idle";
      response->success = false;
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (state == nav_state_t::not_ready)
    {
      response->message = "Hover not called, navigation is not initialized";
      response->success = false;
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return true;
    }

    std::scoped_lock lock(state_mutex_, waypoints_mutex_, control_ac_mutex_);
    hover();
    abort_goal(action_server_goal_handle_, "Hover commanded.");
    response->message = "Navigation stopped. Hovering";
    response->success = true;
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    return true;
  }
  //}

  // | -------------------- Future callbacks -------------------- |

  /* waypointFutureCallback //{ */
  void Navigation::waypointFutureCallback(rclcpp::Client<fog_msgs::srv::WaypointToLocal>::SharedFuture future)
  {
    if (!future_ready(future))
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to call service to transform waypoint.");
      return;
    }
    const std::shared_ptr<fog_msgs::srv::WaypointToLocal_Response> result = future.get();
    if (result == nullptr)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to call service to transform waypoint.");
      return;
    }
    if (!result->success)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to transform waypoint: " << result->message);
      return;
    }

    RCLCPP_INFO(get_logger(), "Coordinate transform returned: %.2f, %.2f", result->local_x, result->local_y);
    const vec4_t point(result->local_x, result->local_y, result->local_z,
                       sradians::wrap(result->heading));  // ensure that the heading is in the range [-pi, pi]
    const std::vector<vec4_t> path = {point};

    std::string reason;
    const size_t added = addWaypoints(path, no_goal, override_previous_commands_, reason);
    if (added == 0)
      RCLCPP_ERROR_STREAM(get_logger(), "Waypoint rejected: " << reason);
    else
      RCLCPP_INFO(get_logger(), "Waypoint added (LOCAL): %.2f, %.2f, %.2f, %.2f", point.x(), point.y(), point.z(), point.w());
  }
  //}

  /* pathFutureCallback //{ */
  void Navigation::pathFutureCallback(rclcpp::Client<fog_msgs::srv::PathToLocal>::SharedFuture future, const std::shared_ptr<NavigationGoalHandle> goal_handle)
  {
    auto goal_result = std::make_shared<NavigationAction::Result>();

    if (!future_ready(future))
    {
      if (goal_handle)
        abort_goal(goal_handle, "Failed to call service to transform path.");
      else
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to call service to transform path.");
      return;
    }

    const std::shared_ptr<fog_msgs::srv::PathToLocal_Response> fut_result = future.get();
    if (fut_result == nullptr)
    {
      if (goal_handle)
        abort_goal(goal_handle, "Failed to call service to transform path.");
      else
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to call service to transform path.");
      return;
    }

    if (!fut_result->success)
    {
      if (goal_handle)
        abort_goal(goal_handle, "Failed to transform path: " + fut_result->message);
      else
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to transform path: " << fut_result->message);
      return;
    }

    RCLCPP_INFO(get_logger(), "Coordinate transform returned %ld points", fut_result->path.poses.size());

    std::string reason;
    const size_t added = addWaypoints(fut_result->path.poses, goal_handle, override_previous_commands_, reason);
    if (added == 0)
    {
      if (goal_handle)
        abort_goal(goal_handle, std::to_string(goal_handle->get_goal()->path.poses.size()) + " new waypoints not set: " + reason);
      else
        RCLCPP_ERROR_STREAM(get_logger(), "Path rejected: " << reason);
      return;
    }

    // finally, hopefully all went well!
    RCLCPP_INFO_STREAM(get_logger(), "Queued " << fut_result->path.poses.size() << " new waypoints for planning.");
  }
  //}

  // --------------------------------------------------------------
  // |                          Routines                          |
  // --------------------------------------------------------------

  /* navigationRoutine //{ */
  void Navigation::navigationRoutine()
  {
    std::scoped_lock lck(state_mutex_, waypoints_mutex_, control_ac_mutex_);

    state_navigation_common();

    switch (state_)
    {
      case nav_state_t::not_ready:
        state_navigation_not_ready();
        break;

      case nav_state_t::idle:
        state_navigation_idle();
        break;

      case nav_state_t::planning:
        state_navigation_planning();
        break;

      case nav_state_t::commanding:
        state_navigation_commanding();
        break;

      case nav_state_t::moving:
        state_navigation_moving();
        break;

      case nav_state_t::avoiding:
        state_navigation_avoiding();
        break;

      default:
        assert(false && "Invalid state, this should never happen!");
        RCLCPP_ERROR(get_logger(), "Invalid state, this should never happen!");
        return;
    }

    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Navigation state: " << to_string(state_));

    std_msgs::msg::String msg;
    msg.data = toupper(to_string(state_));
    status_publisher_->publish(msg);
  }
  //}

  /* diagnosticsRoutine //{ */
  void Navigation::diagnosticsRoutine()
  {
    std::scoped_lock lck(state_mutex_, bumper_mutex_, waypoints_mutex_, action_server_mutex_);
    publishDiagnostics();
    update_goal(action_server_goal_handle_, waypoint_current_it_, waypoints_in_.size());
  }
  //}

  // | ------------- State function implementations ------------- |

  /* state_navigation_common() method //{ */
  void Navigation::state_navigation_common()
  {
    // bumper is always checked even when the vehicle is on the ground
    const auto bumper_msg = get_mutexed(bumper_mutex_, bumper_msg_);
    if (bumper_enabled_)
    {
      const bool bumper_data_old = bumperDataOld(bumper_msg);
      const bool obstacle_detected = bumperCheckObstacles(bumper_msg);

      // if the data is old and navigation is doing something, abort it - flight may be dangerous
      if (state_ != nav_state_t::not_ready && bumper_data_old)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Missing fresh bumper data calling hover and switching to not_ready.");
        hover();
        abort_goal(action_server_goal_handle_, "Missing fresh bumper data.");
        state_ = nav_state_t::not_ready;
        return;
      }

      // if applicable, check for obstacles and switch to avoiding
      if (state_ != nav_state_t::not_ready && state_ != nav_state_t::avoiding && !bumper_data_old && obstacle_detected)
      {
        RCLCPP_WARN(get_logger(), "Obstacle detected! Switching to avoiding.");
        state_ = nav_state_t::avoiding;
      }

      set_mutexed(bumper_mutex_, std::make_tuple(!bumper_data_old, obstacle_detected), std::forward_as_tuple(bumper_active_, bumper_obstacle_detected_));
    }

    // check that the vehicle is still autonomously flying and switch to not_ready if applicable
    const vehicle_state_t control_vehicle_state = get_mutexed(control_diags_mutex_, control_vehicle_state_);
    if (state_ != nav_state_t::not_ready && control_vehicle_state != vehicle_state_t::autonomous_flight)
    {
      RCLCPP_INFO(get_logger(), "Vehicle no longer in autonomous flight. Clearing waypoints and switching to not_ready.");
      waypoints_in_.clear();
      waypoint_current_it_ = 0;
      waypoint_state_ = waypoint_state_t::empty;
      state_ = nav_state_t::not_ready;
      abort_goal(action_server_goal_handle_, "Vehicle no longer in autonomous flight.");
      return;
    }
  }
  //}

  /* state_navigation_not_ready() method //{ */
  void Navigation::state_navigation_not_ready()
  {
    const bool getting_octomap = get_mutexed(octree_mutex_, getting_octomap_);
    const bool getting_uav_pose = get_mutexed(uav_pose_mutex_, getting_uav_pose_);
    const bool getting_cmd_pose = get_mutexed(cmd_pose_mutex_, getting_cmd_pose_);
    const auto bumper_msg = get_mutexed(bumper_mutex_, bumper_msg_);
    const bool bumper_ok = !bumper_enabled_ || !bumperDataOld(bumper_msg);
    const auto [control_vehicle_state, getting_control_diagnostics] = get_mutexed(control_diags_mutex_, control_vehicle_state_, getting_control_diagnostics_);

    if (is_initialized_ && getting_octomap && getting_control_diagnostics && getting_uav_pose && getting_cmd_pose && bumper_ok
        && control_vehicle_state == vehicle_state_t::autonomous_flight)
    {
      state_ = nav_state_t::idle;
      RCLCPP_INFO(get_logger(), "Navigation is now ready! Switching state to idle.");
    } else
    {
      std::string reasons;
      add_reason_if("missing octomap", !getting_octomap, reasons);
      add_reason_if("missing control diagnostics", !getting_control_diagnostics, reasons);
      add_reason_if("missing uav pose", !getting_uav_pose, reasons);
      add_reason_if("missing cmd pose", !getting_cmd_pose, reasons);
      add_reason_if("missing bumper data", !bumper_ok, reasons);
      add_reason_if("vehicle not in autonomous mode", control_vehicle_state != vehicle_state_t::autonomous_flight, reasons);
      RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Not initialized: " << reasons);
    }
  }
  //}

  /* state_navigation_idle() method //{ */
  void Navigation::state_navigation_idle()
  {
    if (!waypoints_in_.empty() && waypoint_current_it_ < waypoints_in_.size())
    {
      RCLCPP_WARN(get_logger(), "New navigation goals available. Switching to planning");
      state_ = nav_state_t::planning;
    }
  }
  //}

  /* state_navigation_planning() method //{ */
  void Navigation::state_navigation_planning()
  {
    /* initial checks //{ */

    if (octree_ == nullptr || octree_->size() < 1)
    {
      RCLCPP_WARN(get_logger(), "Octomap is nullptr or empty! Aborting planning and swiching to idle");
      abort_goal(action_server_goal_handle_, "Octomap is nullptr or empty!");
      state_ = nav_state_t::idle;
      waypoint_state_ = waypoint_state_t::empty;
      return;
    }

    if (waypoints_in_.empty() || waypoint_current_it_ >= waypoints_in_.size())
    {
      RCLCPP_INFO(get_logger(), "No more navigation goals available. Switching to idle");
      waypoints_in_.clear();
      waypoint_current_it_ = 0;
      finish_goal(action_server_goal_handle_);
      state_ = nav_state_t::idle;
      return;
    }

    //}

    // if we got here, let's plan!
    const vec4_t goal = waypoints_in_.at(waypoint_current_it_);

    RCLCPP_INFO_STREAM(get_logger(),
                       "Waypoint " << goal.transpose() << " set as the next goal #" << waypoint_current_it_ << "/" << waypoints_in_.size() << ".");

    visualizeGoals(waypoints_in_);

    const vec4_t uav_pose = get_mutexed(uav_pose_mutex_, uav_pose_);
    RCLCPP_INFO(get_logger(), "Current position: [%7.2f, %7.2f, %7.2f, %7.2f]", uav_pose.x(), uav_pose.y(), uav_pose.z(), uav_pose.w());
    RCLCPP_INFO(get_logger(), "Current nav_goal: [%7.2f, %7.2f, %7.2f, %7.2f]", goal.x(), goal.y(), goal.z(), goal.w());

    // start the actual nav_state_t::planning
    const auto [planned_path, goal_reached] = planPath(goal, octree_);
    // evaluate the result of path planning
    // if no path was found, check the cause
    if (planned_path.empty())
    {
      // the current goal is already reached!
      if (goal_reached)
      {
        replanning_counter_ = 0;  // planning for this goal is done, reset the retry counter
        waypoint_state_ = waypoint_state_t::reached;
        // and it was the final goal that we've got
        if (waypoint_current_it_ >= waypoints_in_.size())
        {
          RCLCPP_INFO(get_logger(), "The last provided navigation goal has been visited. Switching to idle");
          waypoints_in_.clear();
          waypoint_current_it_ = 0;
          finish_goal(action_server_goal_handle_);
          state_ = nav_state_t::idle;
        }
        // if it was not the final goal, let's go to the next goal
        else
        {
          RCLCPP_INFO(get_logger(), "Navigation goal #%lu/%lu visited, continuing.", waypoint_current_it_, waypoints_in_.size());
          waypoint_current_it_++;
          update_goal(action_server_goal_handle_, waypoint_current_it_, waypoints_in_.size());
        }
      }
      // the current goal is not reached, yet the path is empty. this means that the planning failed
      else
      {
        RCLCPP_INFO(get_logger(), "A path to the current goal was not found.");
        replanning_counter_++;
        // check if the number of retries is too high
        if (replanning_counter_ >= replanning_limit_)
        {
          RCLCPP_ERROR(get_logger(), "No path produced after %d repeated attempts. Please provide a new waypoint. Clearing waypoints and switching to idle.",
                       replanning_counter_);
          abort_goal(action_server_goal_handle_,
                     "Waypoint #" + std::to_string(waypoint_current_it_) + "/" + std::to_string(waypoints_in_.size()) + " is unreachable.");
          waypoints_in_.clear();
          waypoint_current_it_ = 0;
          replanning_counter_ = 0;
          state_ = nav_state_t::idle;
          waypoint_state_ = waypoint_state_t::unreachable;
        }
      }
    }
    // a path was found, let's follow it
    else
    {
      replanning_counter_ = 0;                                       // planning was successful, so reset the retry counter
      control_goal_handle_future_ = commandWaypoints(planned_path);  // send the waypoints to the command interface as a new action goal
      // change the state accordingly
      state_ = nav_state_t::commanding;
      waypoint_state_ = waypoint_state_t::ongoing;
    }
  }
  //}

  /* state_navigation_commanding() method //{ */
  void Navigation::state_navigation_commanding()
  {
    const auto command_id = get_mutexed(control_diags_mutex_, control_command_id_);

    // check if the goal was processed by control interface
    if (!future_ready(control_goal_handle_future_))
      return;

    // check if the goal was accepted
    const auto goal_handle = control_goal_handle_future_.get();
    if (goal_handle == nullptr)
    {
      RCLCPP_WARN_STREAM(get_logger(), "Current segment goal not accepted (mission #" << command_id << "), switching to planning.");
      state_ = nav_state_t::planning;
      return;
    }

    // if the goal was accepted, set the result future and continue to the next state
    control_goal_result_future_ = control_ac_ptr_->async_get_result(goal_handle);
    state_ = nav_state_t::moving;
  }
  //}

  /* state_navigation_moving() method //{ */
  void Navigation::state_navigation_moving()
  {
    const auto command_id = get_mutexed(control_diags_mutex_, control_command_id_);

    replanning_counter_ = 0;

    // check if the result is already available
    if (!future_ready(control_goal_result_future_))
      return;
    const auto result = control_goal_result_future_.get();

    switch (result.code)
    {
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN_STREAM(get_logger(), "Current segment goal aborted (mission #" << command_id << "), switching to planning.");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN_STREAM(get_logger(), "Current segment goal canceled (mission #" << command_id << "), switching to planning.");
        break;
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO_STREAM(get_logger(), "Current segment goal reached (mission #" << command_id << "), switching to planning.");
        break;
      default:
        RCLCPP_ERROR_STREAM(get_logger(), "Unknown goal result code: " << int8_t(result.code) << " (mission #" << command_id << "), switching to planning.");
        break;
    }
    state_ = nav_state_t::planning;
  }
  //}

  /* state_navigation_avoiding() method //{ */
  void Navigation::state_navigation_avoiding()
  {
    const bool waypoints_left = waypoints_in_.empty() && waypoint_current_it_ < waypoints_in_.size();
    // to which state to revert if avoidance is done
    const nav_state_t reset_state = waypoints_left ? nav_state_t::idle : nav_state_t::planning;

    // only replan if at least bumper_min_replan_period_ duration has elapsed since the last replan
    static rclcpp::Time last_replan = get_clock()->now() - bumper_min_replan_period_;
    const rclcpp::Time now = get_clock()->now();
    if (now - last_replan < bumper_min_replan_period_)
      return;
    last_replan = get_clock()->now();

    const auto bumper_msg = get_mutexed(bumper_mutex_, bumper_msg_);
    const vec3_t avoidance_vector = bumperGetAvoidanceVector(bumper_msg);
    if (avoidance_vector.norm() == 0)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Nothing to avoid, switching to " << to_string(reset_state) << ".");
      state_ = reset_state;
      return;
    }

    const vec4_t uav_pose = get_mutexed(uav_pose_mutex_, uav_pose_);
    const vec4_t cmd_pose = get_mutexed(cmd_pose_mutex_, cmd_pose_);
    const vec4_t new_goal = cmd_pose + vec4_t(avoidance_vector.x(), avoidance_vector.y(), avoidance_vector.z(), 0.0);
    commandWaypoints({new_goal});
    RCLCPP_INFO(get_logger(), "[Bumper]: Avoiding obstacle by moving from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]", uav_pose.x(), uav_pose.y(), uav_pose.z(),
                new_goal.x(), new_goal.y(), new_goal.z());
  }
  //}

  // --------------------------------------------------------------
  // |                       Helper methods                       |
  // --------------------------------------------------------------

  /* planPath() method //{ */
  // returns a path to the goal (if found) and a bool, indicating if the UAV is already in the goal (if the path is empty)
  // if the path is empty and the bool is false, then the path could not be found or the found path is invalid
  std::pair<std::vector<vec4_t>, bool> Navigation::planPath(const vec4_t& goal, std::shared_ptr<octomap::OcTree> mapping_tree)
  {
    navigation::AstarPlanner planner = navigation::AstarPlanner(
        safe_obstacle_distance_, euclidean_distance_cutoff_, planning_tree_resolution_, distance_penalty_, greedy_penalty_, min_altitude_, max_altitude_,
        ground_cutoff_, planning_timeout_, max_waypoint_distance_, altitude_acceptance_radius_, unknown_is_occupied_, get_logger());
    const vec4_t uav_pose = get_mutexed(uav_pose_mutex_, uav_pose_);
    const vec4_t cmd_pose = get_mutexed(cmd_pose_mutex_, cmd_pose_);

    const octomap::point3d planning_goal = toPoint3d(goal);
    octomap::point3d planning_start;
    if ((cmd_pose.head<3>() - uav_pose.head<3>()).norm() <= navigation_tolerance_)
      planning_start = toPoint3d(cmd_pose);
    else
      planning_start = toPoint3d(uav_pose);

    const auto [path, result] =
        planner.findPath(planning_start, planning_goal, mapping_tree, planning_timeout_, std::bind(&Navigation::visualizeTree, this, _1),
                         std::bind(&Navigation::visualizeExpansions, this, _1, _2, _3));

    RCLCPP_INFO(get_logger(), "Planner returned %ld waypoints", path.size());

    bool path_valid;
    bool goal_reached;
    bool append_goal;
    /* evaluate based on the result and the current UAV position whether the cmd goal is reached and whether to append the goal to the output //{ */
    switch (result)
    {
      case GOAL_REACHED:
        /*  //{ */
        // if the current UAV heading is different from the desired, just turn
        // use sradians::dist instead of std::abs to correctly compare two cyclic values in range [-pi, pi]
        if (sradians::dist(uav_pose.w(), goal.w()) > max_heading_step_)
        {
          goal_reached = false;
          append_goal = true;
          path_valid = true;
          RCLCPP_INFO(get_logger(), "turning in one spot");
        }
        // otherwise, we're in position
        else
        {
          goal_reached = true;
          append_goal = false;
          path_valid = false;
        }
        //}
        break;

      case COMPLETE:
        goal_reached = false;
        append_goal = true;
        path_valid = true;
        break;

      case INCOMPLETE:
        /*  //{ */
        goal_reached = false;
        append_goal = false;
        path_valid = true;
        if (path.size() > 1)
        {
          double path_dist = 0.0;
          for (size_t i = 1; i < path.size(); i++)
            path_dist += (path[i] - path[i - 1]).norm();
          // use sradians::dist instead of std::abs to correctly compare two cyclic values in range [-pi, pi]
          if (path_dist < planning_tree_resolution_)
          {
            if (sradians::dist(uav_pose.w(), goal.w()) > max_heading_step_)
            {
              append_goal = true;
              RCLCPP_INFO(get_logger(), "turning in one spot");
            }
          }
        }
        //}
        break;

      case GOAL_IN_OBSTACLE:
        goal_reached = false;
        append_goal = false;
        path_valid = false;
        RCLCPP_WARN_STREAM(get_logger(), "Goal " << goal.transpose() << " is inside an inflated obstacle");
        break;

      case FAILURE:
      default:
        goal_reached = false;
        append_goal = false;
        path_valid = false;
        break;
    }

    //}

    // if path is not valid, just return an empty path
    if (!path_valid)
      return {{}, goal_reached};

    // resample path and add heading
    const std::vector<vec4_t> resampled = resamplePath(path, goal.w());

    // finally, start filling the buffer up to the maximal replanning distance
    std::vector<vec4_t> planned_path;
    planned_path.reserve(resampled.size() + 1);
    for (const auto& w : resampled)
    {
      if ((w.head<3>() - cmd_pose.head<3>()).norm() <= replanning_distance_)
      {
        planned_path.push_back(w);
      } else
      {
        RCLCPP_INFO(get_logger(), "Path exceeding replanning distance");
        append_goal = false;
        break;
      }
    }

    if (append_goal)
    {
      planned_path.push_back(goal);
      RCLCPP_INFO(get_logger(), "Appended current goal waypoint to the path.");
    }

    return {planned_path, goal_reached};
  }
  //}

  /* bumperDataOld() method //{ */
  bool Navigation::bumperDataOld(const fog_msgs::msg::ObstacleSectors::SharedPtr bumper_msg)
  {
    return bumper_msg == nullptr || (get_clock()->now() - bumper_msg->header.stamp).seconds() > 1.0;
  }
  //}

  /* bumperCheckObstacles //{ */
  bool Navigation::bumperCheckObstacles(const fog_msgs::msg::ObstacleSectors::SharedPtr bumper_msg)
  {
    // no info available
    if (bumper_msg == nullptr)
      return true;

    for (int i = 0; i < int(bumper_msg->n_horizontal_sectors); i++)
    {
      // no reading available - skip it
      if (bumper_msg->sectors.at(i) < 0)
        continue;

      // if the sector is below the safe distance
      if (bumper_msg->sectors.at(i) <= safe_obstacle_distance_ * bumper_distance_factor_)
        return true;
    }
    return false;
  }
  //}

  /* bumperGetAvoidanceVector//{ */
  vec3_t Navigation::bumperGetAvoidanceVector(const fog_msgs::msg::ObstacleSectors::SharedPtr bumper_msg)
  {
    // no info available
    if (bumper_msg == nullptr)
      return vec3_t::Zero();

    const vec4_t uav_pose = get_mutexed(uav_pose_mutex_, uav_pose_);
    const double sector_size = (2.0 * M_PI) / double(bumper_msg->n_horizontal_sectors);
    const vec3_t forward = vec3_t::UnitX();

    int n_sec = (int)bumper_msg->sectors.size() - 2;  // remove 2 (up and down rangefinders)

    std::set<int> free_sec;
    std::set<int> occ_sec;

    auto cmp = [](std::pair<int, int> a, std::pair<int, int> b) { return a.second > b.second; };
    std::set<std::pair<int, int>, decltype(cmp)> opposite_sec(cmp);

    for (int i = 0; i < int(bumper_msg->n_horizontal_sectors); i++)
    {
      if (bumper_msg->sectors.at(i) < 0)
        continue;

      if (bumper_msg->sectors.at(i) <= safe_obstacle_distance_ * bumper_distance_factor_)
      {
        occ_sec.insert(i);
      } else
      {
        free_sec.insert(i);
      }
    }

    for (auto& s : occ_sec)
    {
      std::pair<int, int> sec;
      sec.first = ((s + (n_sec / 2)) % n_sec);
      sec.second = 0;
      for (auto& o : occ_sec)
      {
        int d = std::abs(o - sec.first) % n_sec;
        sec.second += d;
        opposite_sec.insert(sec);
      }
    }

    // no obstacle
    if (occ_sec.empty())
      return vec3_t::Zero();

    for (auto& s : opposite_sec)
    {
      if (free_sec.find(s.first) != free_sec.end())
      {
        RCLCPP_INFO(this->get_logger(), "[Navigation]: Selected sector %d (free and opposite of obstacle), dist: %d", s.first, s.second);
        const anax_t rot(sector_size * s.first + uav_pose.w(), vec3_t::UnitZ());
        const vec3_t avoidance_vector = rot * ((planning_tree_resolution_)*forward);
        return avoidance_vector;
      }
    }

    if (!free_sec.empty())
    {
      RCLCPP_INFO(this->get_logger(), "[Navigation]: Selected sector %d (free)", (*free_sec.begin()));
      const anax_t rot(sector_size * (*free_sec.begin()) + uav_pose.w(), vec3_t::UnitZ());
      const vec3_t avoidance_vector = rot * ((planning_tree_resolution_)*forward);
      return avoidance_vector;
    }

    return vec3_t::Zero();
  }
  //}

  /* resamplePath //{ */
  // resamples the path to limit the maximal distance between subsequent points and adds heading to the waypoints
  std::vector<vec4_t> Navigation::resamplePath(const std::vector<octomap::point3d>& waypoints, const double end_heading) const
  {
    std::vector<vec4_t> ret;

    if (waypoints.size() < 2)
    {
      for (const auto& w : waypoints)
        ret.push_back(to_eigen(w, end_heading));
      return ret;
    }

    ret.push_back(to_eigen(waypoints.front(), end_heading));

    size_t i = 1;
    while (i < waypoints.size())
    {
      const vec3_t& prev_pt = ret.back().head<3>();
      const vec3_t wp = to_eigen(waypoints.at(i));
      const vec3_t diff_vec = wp - prev_pt;
      const double dist = diff_vec.norm();
      if (dist > max_waypoint_distance_)
      {
        const vec3_t dir_vec = max_waypoint_distance_ * diff_vec.normalized();
        const vec3_t padded = prev_pt + dir_vec;
        const vec4_t padded_heading(padded.x(), padded.y(), padded.z(), end_heading);
        ret.push_back(padded_heading);
      } else
      {
        ret.push_back(vec4_t(wp.x(), wp.y(), wp.z(), end_heading));
        // let's move to processing the next point in the path
        i++;
      }
    }

    RCLCPP_INFO(get_logger(), "Padded %lu original waypoints to %lu points", waypoints.size(), ret.size());

    return ret;
  }
  //}

  /* waypointsToGoal //{ */
  ControlAction::Goal Navigation::waypointsToGoal(const std::vector<vec4_t>& waypoints)
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = get_mutexed(octree_mutex_, octree_frame_);
    for (const auto& wp : waypoints)
    {
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = wp.x();
      p.pose.position.y = wp.y();
      p.pose.position.z = wp.z();
      p.pose.orientation = heading2quat(wp.w());
      msg.poses.push_back(p);
    }
    ControlAction::Goal goal;
    goal.path = msg;
    // read and update the control_command_id_ atomically
    {
      std::scoped_lock lck(control_diags_mutex_);
      goal.mission_id = ++control_command_id_;
    }
    return goal;
  }
  //}

  // | ----------------- Command-sending methods ---------------- |

  /* commandWaypoints() method //{ */
  // the caller must lock the following muteces:
  // control_ac_mutex_
  std::shared_future<ControlGoalHandle::SharedPtr> Navigation::commandWaypoints(const std::vector<vec4_t>& waypoints)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Sending waypoints to control_interface:");
    for (const auto& w : waypoints)
      RCLCPP_INFO(get_logger(), "       [%7.2f, %7.2f, %7.2f, %7.2f]", w.x(), w.y(), w.z(), w.w());
    visualizePath(waypoints);
    publishFutureTrajectory(waypoints);
    const auto goal = waypointsToGoal(waypoints);
    RCLCPP_INFO_STREAM(get_logger(), "Sending mission #" << goal.mission_id << " with " << goal.path.poses.size() << " waypoints to the control interface:");

    // finally, send the goal to control interface
    return control_ac_ptr_->async_send_goal(goal);
  }
  //}

  /* hover //{ */
  // the caller must lock the following muteces:
  // state_mutex_
  // waypoints_mutex_
  // control_ac_mutex_
  void Navigation::hover()
  {
    waypoints_in_.clear();
    waypoint_current_it_ = 0;
    waypoint_state_ = waypoint_state_t::empty;
    const vec4_t cmd_pose = get_mutexed(cmd_pose_mutex_, cmd_pose_);
    commandWaypoints({cmd_pose});
    state_ = nav_state_t::idle;
  }
  //}

  /* cancel_goal() method //{ */
  // the caller must lock the following muteces:
  // action_server_mutex_
  // state_mutex_
  // waypoints_mutex_
  // control_ac_mutex_
  bool Navigation::cancel_goal(const std::shared_ptr<NavigationGoalHandle> goal_handle, std::string& fail_reason_out)
  {
    if (goal_handle == nullptr || !goal_handle->is_active())
    {
      fail_reason_out = "Passed goal handle to cancel is a nullptr or not active! Ignoring.";
      return false;
    }

    auto result = std::make_shared<NavigationAction::Result>();
    if (action_server_goal_handle_->get_goal_id() != goal_handle->get_goal_id())
    {
      fail_reason_out = "Attempting to cancel goal " + rclcpp_action::to_string(goal_handle->get_goal_id()) + ", but goal "
                        + rclcpp_action::to_string(action_server_goal_handle_->get_goal_id()) + " is active! Ignoring.";
      result->message = fail_reason_out;
      goal_handle->canceled(result);
      return false;
    }

    hover();
    result->message = "Goal " + rclcpp_action::to_string(goal_handle->get_goal_id()) + " canceled.";
    goal_handle->canceled(result);
    return true;
  }
  //}

  /* finish_goal() method //{ */
  void Navigation::finish_goal(const std::shared_ptr<NavigationGoalHandle> goal_handle)
  {
    if (goal_handle == nullptr || !goal_handle->is_active())
      return;

    auto result = std::make_shared<NavigationAction::Result>();
    result->message = "Goal " + rclcpp_action::to_string(goal_handle->get_goal_id()) + " finished: All waypoints visited";
    goal_handle->succeed(result);
    RCLCPP_INFO_STREAM(get_logger(), result->message);
  }
  //}

  /* abort_goal() method //{ */
  void Navigation::abort_goal(const std::shared_ptr<NavigationGoalHandle> goal_handle, const std::string& reason)
  {
    if (goal_handle == nullptr || !goal_handle->is_active())
      return;

    auto result = std::make_shared<NavigationAction::Result>();
    result->message = "Goal " + rclcpp_action::to_string(goal_handle->get_goal_id()) + " ABORTED: " + reason;
    goal_handle->abort(result);
    RCLCPP_ERROR_STREAM(get_logger(), result->message);
  }
  //}

  /* update_goal() method //{ */
  void Navigation::update_goal(const std::shared_ptr<NavigationGoalHandle> goal_handle, const int current_waypoint, const int waypoints)
  {
    if (goal_handle == nullptr || !goal_handle->is_active())
      return;

    auto feedback = std::make_shared<NavigationAction::Feedback>();
    feedback->mission_progress.current_waypoint = current_waypoint;
    feedback->mission_progress.size = waypoints;
    feedback->mission_progress.mission_id = mission_id_;
    goal_handle->publish_feedback(feedback);
  }
  //}

  // | -------------- Publish/visualization methods ------------- |

  /* publishDiagnostics //{ */
  // the caller must lock the following muteces:
  // state_mutex_
  // bumper_mutex_
  // octree_mutex_
  // waypoints_mutex_
  void Navigation::publishDiagnostics()
  {
    fog_msgs::msg::NavigationDiagnostics msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = octree_frame_;
    msg.state = to_msg(state_);

    msg.mission_progress.current_waypoint = waypoint_current_it_;
    msg.mission_progress.size = waypoints_in_.size();
    msg.waypoint_state = to_msg(waypoint_state_);
    if (waypoints_in_.size() > waypoint_current_it_)
    {
      msg.waypoint.x = waypoints_in_.at(waypoint_current_it_).x();
      msg.waypoint.y = waypoints_in_.at(waypoint_current_it_).y();
      msg.waypoint.z = waypoints_in_.at(waypoint_current_it_).z();
    } else
    {
      msg.waypoint.x = nand;
      msg.waypoint.y = nand;
      msg.waypoint.z = nand;
    }

    msg.bumper_active = bumper_active_;
    msg.obstacle_detected = bumper_obstacle_detected_;
    diagnostics_publisher_->publish(msg);
  }
  //}

  /* publishFutureTrajectory //{ */
  void Navigation::publishFutureTrajectory(const std::vector<vec4_t>& waypoints)
  {
    const std::string octree_frame = get_mutexed(octree_mutex_, octree_frame_);
    fog_msgs::msg::FutureTrajectory msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = octree_frame;
    for (const auto& w : waypoints)
    {
      fog_msgs::msg::Vector4Stamped v;
      v.x = w.x();
      v.y = w.y();
      v.z = w.z();
      v.w = w.w();
      v.header.frame_id = octree_frame;
      msg.poses.push_back(v);
    }
    future_trajectory_publisher_->publish(msg);
  }
  //}

  /* visualization //{ */

  /* visualizeTree //{ */
  void Navigation::visualizeTree(const std::shared_ptr<octomap::OcTree> tree)
  {
    RCLCPP_INFO_ONCE(get_logger(), "Visualizing tree");
    visualization_msgs::msg::Marker msg;
    msg.header.frame_id = get_mutexed(octree_mutex_, octree_frame_);
    msg.header.stamp = get_clock()->now();
    msg.ns = "tree";
    msg.type = visualization_msgs::msg::Marker::POINTS;
    msg.id = 8;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = tree_points_scale_;
    msg.scale.y = tree_points_scale_;

    for (auto it = tree->begin(); it != tree->end(); it++)
    {
      if (tree->isNodeOccupied(*it))
      {
        geometry_msgs::msg::Point gp;
        auto color = generateColor(0, 0, 0, 1.0);
        gp.x = it.getX();
        gp.y = it.getY();
        gp.z = it.getZ();
        msg.points.push_back(gp);
        msg.colors.push_back(color);
      } else if (show_unoccupied_)
      {
        geometry_msgs::msg::Point gp;
        auto color = generateColor(0.7, 0.7, 0.7, 0.7);
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
  void Navigation::visualizeExpansions(const std::unordered_set<navigation::Node, HashFunction> open,
                                       const std::unordered_set<navigation::Node, HashFunction>& closed, const std::shared_ptr<octomap::OcTree> tree)
  {
    RCLCPP_INFO_ONCE(get_logger(), "Visualizing open planning expansions");
    visualization_msgs::msg::Marker msg;
    msg.header.frame_id = get_mutexed(octree_mutex_, octree_frame_);
    msg.header.stamp = get_clock()->now();
    msg.ns = "expansions";
    msg.type = visualization_msgs::msg::Marker::POINTS;
    msg.id = 8;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = expansions_points_scale_;
    msg.scale.y = expansions_points_scale_;

    double max_cost = 0;
    double min_cost = 1e6;

    for (auto it = open.begin(); it != open.end(); it++)
    {
      if (it->total_cost > max_cost)
      {
        max_cost = it->total_cost;
      }
      if (it->total_cost < min_cost)
      {
        min_cost = it->total_cost;
      }
    }

    for (auto it = open.begin(); it != open.end(); it++)
    {
      auto coords = tree->keyToCoord(it->key);
      geometry_msgs::msg::Point gp;
      double brightness = (it->total_cost - min_cost) / (max_cost - min_cost);
      auto color = generateColor(0.0, brightness, 0.3, 0.8);
      gp.x = coords.x();
      gp.y = coords.y();
      gp.z = coords.z();
      msg.points.push_back(gp);
      msg.colors.push_back(color);
    }

    for (auto it = closed.begin(); it != closed.end(); it++)
    {
      auto coords = tree->keyToCoord(it->key);
      geometry_msgs::msg::Point gp;
      auto color = generateColor(0.8, 0.0, 0, 0.8);
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
  void Navigation::visualizePath(const std::vector<vec4_t>& waypoints)
  {
    RCLCPP_INFO_ONCE(get_logger(), "Visualizing path");
    visualization_msgs::msg::Marker msg;
    msg.header.frame_id = get_mutexed(octree_mutex_, octree_frame_);
    msg.header.stamp = get_clock()->now();
    msg.ns = "path";
    msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    msg.id = 8;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = path_points_scale_;

    for (size_t i = 1; i < waypoints.size(); i++)
    {
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
  void Navigation::visualizeGoals(const std::deque<vec4_t>& waypoints)
  {
    RCLCPP_INFO_ONCE(get_logger(), "Visualizing goals");
    visualization_msgs::msg::Marker msg;
    msg.header.frame_id = get_mutexed(octree_mutex_, octree_frame_);
    msg.header.stamp = get_clock()->now();
    msg.ns = "goals";
    msg.type = visualization_msgs::msg::Marker::POINTS;
    msg.id = 8;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = goal_points_scale_;
    msg.scale.y = goal_points_scale_;

    for (size_t id = 0; id < waypoints.size(); id++)
    {
      const vec4_t w = waypoints[id];
      std_msgs::msg::ColorRGBA c;

      if (id == waypoint_current_it_)
      {
        c = generateColor(0.3, 0.5, 1.0, 1.0);
      } else
      {
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
  std_msgs::msg::ColorRGBA Navigation::generateColor(const double r, const double g, const double b, const double a)
  {
    std_msgs::msg::ColorRGBA c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = a;
    return c;
  }
  //}

  //}

  // | -------------------------- Utils ------------------------- |

  /* future_ready //{ */
  // just a simple helper function
  template <typename T>
  bool future_ready(const std::shared_future<T>& f)
  {
    assert(f.valid());
    const auto result = f.wait_for(std::chrono::seconds(0));
    return result == std::future_status::ready || result == std::future_status::deferred;
  }
  //}

  /* to_eigen //{ */
  vec4_t to_eigen(const vec4_t& vec)
  {
    return vec;
  }

  vec3_t to_eigen(const octomath::Vector3& vec)
  {
    return {vec.x(), vec.y(), vec.z()};
  }

  vec4_t to_eigen(const octomath::Vector3& vec, const double heading)
  {
    return {vec.x(), vec.y(), vec.z(), heading};
  }

  vec4_t to_eigen(const geometry_msgs::msg::PoseStamped& pose)
  {
    return {pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, quat2heading(pose.pose.orientation)};
  }
  //}

  /* toPoint3d() method //{ */
  octomap::point3d toPoint3d(const vec4_t& vec)
  {
    octomap::point3d p;
    p.x() = (float)vec.x();
    p.y() = (float)vec.y();
    p.z() = (float)vec.z();
    return p;
  }
  //}

  /* new_cbk_grp() method //{ */
  // just a util function that returns a new mutually exclusive callback group to shorten the call
  rclcpp::CallbackGroup::SharedPtr Navigation::new_cbk_grp()
  {
    const rclcpp::CallbackGroup::SharedPtr new_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_groups_.push_back(new_group);
    return new_group;
  }
  //}

}  // namespace navigation
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(navigation::Navigation)

