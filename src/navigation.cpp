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
#include <limits>
#include <tuple>

#include <control_interface/enums.h>
#include <fog_lib/mutex_utils.h>
#include <fog_lib/params.h>
#include <fog_lib/misc.h>
#include <fog_lib/geometry/cyclic.h>
#include <fog_lib/geometry/misc.h>

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

  /* class Navigation //{ */
  class Navigation : public rclcpp::Node
  {
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

    // bumper-related variables
    std::shared_mutex bumper_mutex_;
    std::shared_ptr<fog_msgs::msg::ObstacleSectors> bumper_msg_ = nullptr;

    std::shared_mutex state_mutex_;
    enum nav_state_t
    {
      not_ready,
      idle,
      planning,
      commanding,
      moving,
      avoiding
    } state_ = nav_state_t::not_ready;

    std::mutex waypoints_mutex_;
    std::deque<vec4_t> waypoints_in_;
    size_t waypoint_current_it_ = 0;
    vec4_t waypoint_current_ = vec4_t(nand, nand, nand, nand);
    enum waypoint_state_t
    {
      empty = 0,
      ongoing,
      reached,
      unreachable
    } waypoint_state_ = waypoint_state_t::empty;
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

    // params
    double euclidean_distance_cutoff_;
    double safe_obstacle_distance_;
    double navigation_tolerance_;
    bool unknown_is_occupied_;
    double min_altitude_;
    double max_altitude_;
    double max_goal_distance_;
    double distance_penalty_;
    double greedy_penalty_;
    double planning_tree_resolution_;
    double max_waypoint_distance_;
    double max_heading_step_;
    double planning_timeout_;
    int replanning_limit_;
    double replanning_distance_;
    double main_update_rate_;

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
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr goto_trigger_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hover_service_;

    rclcpp::Service<fog_msgs::srv::Vec4>::SharedPtr local_waypoint_service_;
    rclcpp::Service<fog_msgs::srv::Path>::SharedPtr local_path_service_;
    rclcpp::Service<fog_msgs::srv::Vec4>::SharedPtr gps_waypoint_service_;
    rclcpp::Service<fog_msgs::srv::Path>::SharedPtr gps_path_service_;


    rclcpp::Client<fog_msgs::srv::Path>::SharedPtr local_path_client_;
    rclcpp::Client<fog_msgs::srv::Path>::SharedFuture local_path_future_;

    rclcpp::Client<fog_msgs::srv::WaypointToLocal>::SharedPtr waypoint_to_local_client_;
    rclcpp::Client<fog_msgs::srv::PathToLocal>::SharedPtr path_to_local_client_;

    template<typename T>
    size_t addWaypoints(const T& path, const bool override, std::string& fail_reason_out);
    void pathCallback(const nav_msgs::msg::Path::UniquePtr msg);

    // service callbacks
    bool hoverCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    bool localWaypointCallback(const std::shared_ptr<fog_msgs::srv::Vec4::Request> request, std::shared_ptr<fog_msgs::srv::Vec4::Response> response);
    bool localPathCallback(const std::shared_ptr<fog_msgs::srv::Path::Request> request, std::shared_ptr<fog_msgs::srv::Path::Response> response);
    bool gpsWaypointCallback(const std::shared_ptr<fog_msgs::srv::Vec4::Request> request, std::shared_ptr<fog_msgs::srv::Vec4::Response> response);
    bool gpsPathCallback(const std::shared_ptr<fog_msgs::srv::Path::Request> request, std::shared_ptr<fog_msgs::srv::Path::Response> response);

    // future callback
    void waypointFutureCallback(rclcpp::Client<fog_msgs::srv::WaypointToLocal>::SharedFuture future);
    void pathFutureCallback(rclcpp::Client<fog_msgs::srv::PathToLocal>::SharedFuture future);

    // visualization
    void visualizeTree(const octomap::OcTree& tree);
    void visualizeExpansions(const std::unordered_set<navigation::Node, HashFunction> open, const std::unordered_set<navigation::Node, HashFunction>& closed,
                             const octomap::OcTree& tree);
    void visualizePath(const std::vector<vec4_t>& waypoints);
    void visualizeGoals(const std::deque<vec4_t>& waypoints);

    std::vector<vec4_t> resamplePath(const std::vector<octomap::point3d>& waypoints, const double end_heading) const;
    std::shared_ptr<fog_msgs::srv::Path::Request> waypointsToPath(const std::vector<vec4_t>& waypoints);
    void startSendingWaypoints(const std::vector<vec4_t>& waypoints);
    void hover();

    void publishDiagnostics();
    void publishFutureTrajectory(const std::vector<vec4_t>& waypoints);

    // bumper
    bool bumperCheckObstacles(const fog_msgs::msg::ObstacleSectors& bumper_msg);
    vec3_t bumperGetAvoidanceVector(const fog_msgs::msg::ObstacleSectors& bumper_msg);

    std_msgs::msg::ColorRGBA generateColor(const double r, const double g, const double b, const double a);

    std::string to_string(const nav_state_t state) const;
    std::string to_string(const waypoint_state_t state) const;

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

    loaded_successfully &= parse_param("visualization.visualize_planner", visualize_planner_, *this);
    loaded_successfully &= parse_param("visualization.show_unoccupied", show_unoccupied_, *this);
    loaded_successfully &= parse_param("visualization.tree_points_scale", tree_points_scale_, *this);
    loaded_successfully &= parse_param("visualization.expansions_points_scale", expansions_points_scale_, *this);
    loaded_successfully &= parse_param("visualization.path_points_scale", path_points_scale_, *this);
    loaded_successfully &= parse_param("visualization.goal_points_scale", goal_points_scale_, *this);

    loaded_successfully &= parse_param("bumper.enabled", bumper_enabled_, *this);
    loaded_successfully &= parse_param("bumper.distance_factor", bumper_distance_factor_, *this);
    loaded_successfully &= parse_param("bumper.min_replan_period", bumper_min_replan_period_, *this);

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
    local_path_client_ = create_client<fog_msgs::srv::Path>("~/local_path_out");
    waypoint_to_local_client_ = create_client<fog_msgs::srv::WaypointToLocal>("~/waypoint_to_local_out");
    path_to_local_client_ = create_client<fog_msgs::srv::PathToLocal>("~/path_to_local_out");

    // | ------------------ initialize callbacks ------------------ |
    rclcpp::SubscriptionOptions subopts;

    subopts.callback_group = new_cbk_grp();
    odometry_subscriber_ = create_subscription<nav_msgs::msg::Odometry>("~/odometry_in",
        rclcpp::SystemDefaultsQoS(), std::bind(&Navigation::odometryCallback, this, _1), subopts);

    subopts.callback_group = new_cbk_grp();
    cmd_pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>("~/cmd_pose_in",
        rclcpp::SystemDefaultsQoS(), std::bind(&Navigation::cmdPoseCallback, this, _1), subopts);

    subopts.callback_group = new_cbk_grp();
    goto_subscriber_ = create_subscription<nav_msgs::msg::Path>("~/goto_in",
        rclcpp::SystemDefaultsQoS(), std::bind(&Navigation::pathCallback, this, _1), subopts);

    subopts.callback_group = new_cbk_grp();
    control_diagnostics_subscriber_ = create_subscription<fog_msgs::msg::ControlInterfaceDiagnostics>("~/control_diagnostics_in",
        rclcpp::SystemDefaultsQoS(), std::bind(&Navigation::controlDiagnosticsCallback, this, _1), subopts);

    subopts.callback_group = new_cbk_grp();
    octomap_subscriber_ = create_subscription<octomap_msgs::msg::Octomap>("~/octomap_in",
        rclcpp::SystemDefaultsQoS(), std::bind(&Navigation::octomapCallback, this, _1), subopts);

    subopts.callback_group = new_cbk_grp();
    bumper_subscriber_ = create_subscription<fog_msgs::msg::ObstacleSectors>("~/bumper_in",
        rclcpp::SystemDefaultsQoS(), std::bind(&Navigation::bumperCallback, this, _1), subopts);

    // service handlers
    const auto qos_profile = qos.get_rmw_qos_profile();
    const auto svc_grp_ptr = new_cbk_grp();
    hover_service_ = create_service<std_srvs::srv::Trigger>("~/hover_in",
        std::bind(&Navigation::hoverCallback, this, _1, _2), qos_profile, svc_grp_ptr);

    local_waypoint_service_ = create_service<fog_msgs::srv::Vec4>("~/local_waypoint_in",
        std::bind(&Navigation::localWaypointCallback, this, _1, _2), qos_profile, svc_grp_ptr);

    local_path_service_ = create_service<fog_msgs::srv::Path>("~/local_path_in",
        std::bind(&Navigation::localPathCallback, this, _1, _2), qos_profile, svc_grp_ptr);

    gps_waypoint_service_ = create_service<fog_msgs::srv::Vec4>("~/gps_waypoint_in",
        std::bind(&Navigation::gpsWaypointCallback, this, _1, _2), qos_profile, svc_grp_ptr);

    gps_path_service_ = create_service<fog_msgs::srv::Path>("~/gps_path_in",
        std::bind(&Navigation::gpsPathCallback, this, _1, _2), qos_profile, svc_grp_ptr);

    // timers
    execution_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / main_update_rate_),
        std::bind(&Navigation::navigationRoutine, this), new_cbk_grp());

    diagnostics_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / main_update_rate_),
        std::bind(&Navigation::diagnosticsRoutine, this), new_cbk_grp());

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
    }
    else
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
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Received uav pose contains NaNs, ignoring! Position: [%.2f, %.2f, %.2f], orientation [%.2f, %.2f, %.2f, %.2f].",
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
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
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Received cmd pose contains NaNs, ignoring! Position: [%.2f, %.2f, %.2f], orientation [%.2f, %.2f, %.2f, %.2f].",
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
            msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
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
    set_mutexed(control_diags_mutex_,
        std::make_tuple(vehicle_state, mission_state, true, msg->mission_id),
        std::forward_as_tuple(control_vehicle_state_, control_mission_state_, getting_control_diagnostics_, control_response_id_)
      );
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

  // | -------------------- Command callbacks ------------------- |

  /* addWaypoints() method //{ */
  // the caller must NOT lock the following muteces or else a deadlock may happen:
  // state_mutex_
  // waypoints_mutex_
  template<typename T>
  size_t Navigation::addWaypoints(const T& path, const bool override, std::string& fail_reason_out)
  {
    std::scoped_lock lock(state_mutex_, waypoints_mutex_);
    size_t added = 0;
    if (state_ != nav_state_t::idle)
    {
      if (override && state_ != nav_state_t::not_ready)
      {
        RCLCPP_INFO(get_logger(), "Overriding previous navigation commands");
        hover(); // clears previous waypoints and commands the drone to the cmd_pose_, which should stop it
      }
      else
      {
        fail_reason_out = "not idle! Current state is: " + to_string(state_);
        return added;
      }
    }

    const vec4_t cmd_pose = get_mutexed(cmd_pose_mutex_, cmd_pose_);
  
    RCLCPP_INFO(get_logger(), "Recieved %ld waypoints", path.size());
    for (const auto& p : path)
    {
      // convert the point to eigen and ensure that the fourth element (heading) is in the range of [-pi, pi]
      const vec4_t point = wrap_heading(to_eigen(p));

      if (point.z() < min_altitude_)
      {
        RCLCPP_WARN(get_logger(), "Goal Z coordinate (%.2f) is below the minimum allowed altitude (%.2f), not adding further waypoints", point.z(), min_altitude_);
        break;
      }

      if (point.z() > max_altitude_)
      {
        RCLCPP_WARN(get_logger(), "Goal Z coordinate (%.2f) is above the maximum allowed altitude (%.2f), not adding further waypoints", point.z(), max_altitude_);
        break;
      }

      if ((point.head<3>() - cmd_pose.head<3>()).norm() > max_goal_distance_)
      {
        RCLCPP_WARN(get_logger(), "Distance to goal (%.2f) exceeds the maximum allowed distance (%.2f m), not adding further waypoints", (point.head<3>() - cmd_pose.head<3>()).norm(), max_goal_distance_);
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
    const size_t added = addWaypoints(msg->poses, override_previous_commands_, reason);
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
    const size_t added = addWaypoints(request->path.poses, override_previous_commands_, reason);
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
    path_to_local_client_->async_send_request(convert_path_req, std::bind(&Navigation::pathFutureCallback, this, std::placeholders::_1));

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
    if (request->goal.size() != 4)
    {
      response->message = "The waypoint must have 4 coordinates (x, y, z, heading)! Ignoring request.";
      response->success = false;
      RCLCPP_ERROR_STREAM(get_logger(), response->message);
      return true;
    }

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
    const size_t added = addWaypoints(path, override_previous_commands_, reason);
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
    if (request->goal.size() != 4)
    {
      response->message = "The waypoint must have 4 coordinates (x, y, z, heading)! Ignoring request.";
      response->success = false;
      RCLCPP_ERROR_STREAM(get_logger(), response->message);
      return true;
    }

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
    waypoint_convert_req->heading = sradians::wrap(request->goal.at(3)); // ensure that the heading is in the range [-pi, pi]
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

    std::scoped_lock lock(state_mutex_, waypoints_mutex_);
    hover();
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
    const vec4_t point(result->local_x, result->local_y, result->local_z, sradians::wrap(result->heading)); // ensure that the heading is in the range [-pi, pi]
    const std::vector<vec4_t> path = {point};

    std::string reason;
    const size_t added = addWaypoints(path, override_previous_commands_, reason);
    if (added == 0)
      RCLCPP_ERROR_STREAM(get_logger(), "Waypoint rejected: " << reason);
    else
      RCLCPP_INFO(get_logger(), "Waypoint added (LOCAL): %.2f, %.2f, %.2f, %.2f", point.x(), point.y(), point.z(), point.w());
  }
  //}

  /* pathFutureCallback //{ */
  void Navigation::pathFutureCallback(rclcpp::Client<fog_msgs::srv::PathToLocal>::SharedFuture future)
  {
    const std::shared_ptr<fog_msgs::srv::PathToLocal_Response> result = future.get();
    if (result == nullptr)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to call service to transform path.");
      return;
    }
    if (!result->success)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to transform path: " << result->message);
      return;
    }

    RCLCPP_INFO(get_logger(), "Coordinate transform returned %ld points", result->path.poses.size());

    std::string reason;
    const size_t added = addWaypoints(result->path.poses, override_previous_commands_, reason);
    if (added == 0)
      RCLCPP_ERROR_STREAM(get_logger(), "Path rejected: " << reason);
    else
      RCLCPP_INFO_STREAM(get_logger(), "Queued " << result->path.poses.size() << " new waypoints for planning.");
  }
  //}

  // --------------------------------------------------------------
  // |                          Routines                          |
  // --------------------------------------------------------------

  /* navigationRoutine //{ */
  void Navigation::navigationRoutine()
  {
    std::scoped_lock lck(state_mutex_, waypoints_mutex_);

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
    }

    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Navigation state: " << to_string(state_));

    std_msgs::msg::String msg;
    msg.data = to_string(state_);
    status_publisher_->publish(msg);
  }
  //}

  /* diagnosticsRoutine //{ */
  void Navigation::diagnosticsRoutine()
  {
    std::scoped_lock lck(state_mutex_, waypoints_mutex_);
    publishDiagnostics();
  }
  //}

  // | ------------- State function implementations ------------- |

  /* state_navigation_common() method //{ */
  void Navigation::state_navigation_common()
  {
    if (state_ == nav_state_t::not_ready)
      return;
  
    // common checks for all states except unitialized
    const vehicle_state_t control_vehicle_state = get_mutexed(control_diags_mutex_, control_vehicle_state_);
    if (control_vehicle_state != vehicle_state_t::autonomous_flight)
    {
      RCLCPP_INFO(get_logger(), "Vehicle no longer in autonomous flight. Clearing waypoints and switching to not_ready.");
      waypoints_in_.clear();
      waypoint_current_it_ = 0;
      waypoint_state_ = waypoint_state_t::empty;
      state_ = nav_state_t::not_ready;
      return;
    }

    const auto bumper_msg = get_mutexed(bumper_mutex_, bumper_msg_);
    if (bumper_enabled_)
    {
      const bool bumper_data_old = bumper_msg == nullptr || (get_clock()->now() - bumper_msg->header.stamp).seconds() > 1.0;
      if (state_ != nav_state_t::idle && bumper_data_old)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Missing fresh bumper data calling hover and switching to idle.");
        hover();
        state_ = nav_state_t::idle;
        return;
      }

      if (state_ != nav_state_t::avoiding)
      {
        const bool obstacle_detected = bumperCheckObstacles(*bumper_msg);
        if (obstacle_detected)
        {
          RCLCPP_WARN(get_logger(), "Obstacle detected! Switching to avoiding.");
          state_ = nav_state_t::avoiding;
        }
      }
    }
  }
  //}

  /* state_navigation_not_ready() method //{ */
  void Navigation::state_navigation_not_ready()
  {
    const bool getting_octomap = get_mutexed(octree_mutex_, getting_octomap_);
    const bool getting_uav_pose = get_mutexed(uav_pose_mutex_, getting_uav_pose_);
    const bool getting_cmd_pose = get_mutexed(cmd_pose_mutex_, getting_cmd_pose_);
    const auto [control_vehicle_state, getting_control_diagnostics] = get_mutexed(control_diags_mutex_, control_vehicle_state_, getting_control_diagnostics_);

    if (is_initialized_
     && getting_octomap
     && getting_control_diagnostics
     && getting_uav_pose
     && getting_cmd_pose
     && control_vehicle_state == vehicle_state_t::autonomous_flight)
    {
      state_ = nav_state_t::idle;
      RCLCPP_INFO(get_logger(), "Navigation is now ready! Switching state to idle.");
    }
    else
    {
      std::string reasons;
      add_reason_if("missing octomap", !getting_octomap, reasons);
      add_reason_if("missing control diagnostics", !getting_control_diagnostics, reasons);
      add_reason_if("missing uav pose", !getting_uav_pose, reasons);
      add_reason_if("missing cmd pose", !getting_cmd_pose, reasons);
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
      state_ = nav_state_t::idle;
      waypoint_state_ = waypoint_state_t::empty;
      return;
    }
    
    if (waypoints_in_.empty() || waypoint_current_it_ >= waypoints_in_.size())
    {
      RCLCPP_INFO(get_logger(), "No more navigation goals available. Switching to idle");
      state_ = nav_state_t::idle;
      return;
    }
    
    //}
  
    // if we got here, let's plan!
    const vec4_t goal = waypoints_in_.at(waypoint_current_it_);
    waypoint_current_ = goal;
  
    RCLCPP_INFO_STREAM(get_logger(), "Waypoint " << goal.transpose() << " set as the next goal #" << waypoint_current_it_ << "/" << waypoints_in_.size() << ".");
  
    visualizeGoals(waypoints_in_);

    // start the actual nav_state_t::planning
    const auto [planned_path, goal_reached] = planPath(goal, octree_);
    // evaluate the result of path planning
    // if no path was found, check the cause
    if (planned_path.empty())
    {
      // the current goal is already reached!
      if (goal_reached)
      {
        replanning_counter_ = 0; // planning for this goal is done, reset the retry counter
        waypoint_state_ = waypoint_state_t::reached;
        // and it was the final goal that we've got
        if (waypoint_current_it_ >= waypoints_in_.size())
        {
          RCLCPP_INFO(get_logger(), "The last provided navigation goal has been visited. Switching to idle");
          waypoints_in_.clear();
          waypoint_current_it_ = 0;
          state_ = nav_state_t::idle;
        }
        // if it was not the final goal, let's go to the next goal
        else
        {
          RCLCPP_INFO(get_logger(), "Navigation goal #%lu/%lu visited, continuing.", waypoint_current_it_, waypoints_in_.size());
          waypoint_current_it_++;
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
          RCLCPP_ERROR(get_logger(), "No path produced after %d repeated attempts. Please provide a new waypoint. Clearing waypoints and switching to idle.", replanning_counter_);
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
      replanning_counter_ = 0; // planning was successful, so reset the retry counter
      startSendingWaypoints(planned_path); // start sending the waypoints to the command interface
      // change the state accordingly
      state_ = nav_state_t::commanding;
      waypoint_state_ = waypoint_state_t::ongoing;
    }
  }
  //}

  /* state_navigation_commanding() method //{ */
  void Navigation::state_navigation_commanding()
  {
    if (future_ready(local_path_future_))
    {
      const auto resp_ptr = local_path_future_.get();
      // not sure if this can even happen - ROS2 documentation on this is empty...
      if (resp_ptr == nullptr)
      {
        RCLCPP_WARN(get_logger(), "Failed to call local path service of control interface! Deleting waypoints and switching state to idle.");
        hover();
        state_ = nav_state_t::idle;
      }
      // this may happen, but we can't do much - control interface is probably not ready or something
      else if (!resp_ptr->success)
      {
        RCLCPP_WARN_STREAM(get_logger(), "Failed to set local path to control interface! Deleting waypoints and switching state to idle. Reason: " << resp_ptr->message);
        hover();
        state_ = nav_state_t::idle;
      }
      // otherwise, all went well and the vehicle should be moving by this point
      else
      {
        RCLCPP_INFO(get_logger(), "Set local path to control interface. Switching state to moving.");
        state_ = nav_state_t::moving;
      }
    }
  }
  //}

  /* state_navigation_moving() method //{ */
  void Navigation::state_navigation_moving()
  {
    const auto [control_mission_state, command_id, response_id] = get_mutexed(control_diags_mutex_, control_mission_state_, control_command_id_, control_response_id_);

    replanning_counter_ = 0;
    if (control_mission_state == mission_state_t::finished)
    {
      if (response_id == command_id)
      {
        RCLCPP_INFO_STREAM(get_logger(), "End of current segment reached (mission #" << command_id << "), switching to planning");
        state_ = nav_state_t::planning;
      }
      /* else */
      /*   RCLCPP_WARN_STREAM(get_logger(), "Different mission ended (" << response_id << ", expected " << command_id << "), ignoring"); */
    }
  }
  //}

  /* state_navigation_avoiding() method //{ */
  void Navigation::state_navigation_avoiding()
  {
    // if there is already a request sent, check if it was processed already
    if (local_path_future_.valid())
    {
      if (future_ready(local_path_future_))
      {
        const auto resp_ptr = local_path_future_.get();
        // not sure if this can even happen - ROS2 documentation on this is empty...
        if (resp_ptr == nullptr)
        {
          RCLCPP_WARN(get_logger(), "Failed to call local path service of control interface! Deleting waypoints and switching state to idle.");
          hover();
          state_ = nav_state_t::idle;
        }
        // this may happen, but we can't do much - control interface is probably not ready or something
        else if (!resp_ptr->success)
        {
          RCLCPP_WARN_STREAM(get_logger(), "Failed to set local path to control interface! Deleting waypoints and switching state to idle. Reason: " << resp_ptr->message);
          hover();
          state_ = nav_state_t::idle;
        }
      }
      // if the previous service call is not processed yet, do nothing
      else
        return;
    }

    // only replan if at least bumper_min_replan_period_ duration has elapsed since the last replan
    static rclcpp::Time last_replan = get_clock()->now() - bumper_min_replan_period_;
    const rclcpp::Time now = get_clock()->now();
    if (now - last_replan < bumper_min_replan_period_)
      return;
    last_replan = get_clock()->now();

    const auto bumper_msg = get_mutexed(bumper_mutex_, bumper_msg_);
    const vec3_t avoidance_vector = bumperGetAvoidanceVector(*bumper_msg);
    if (avoidance_vector.norm() == 0)
    {
      RCLCPP_INFO(get_logger(), "Nothing to avoid, switching to idle");
      state_ = nav_state_t::idle;
      return;
    }

    const vec4_t uav_pose = get_mutexed(uav_pose_mutex_, uav_pose_);
    const vec4_t cmd_pose = get_mutexed(cmd_pose_mutex_, cmd_pose_);
    const vec4_t new_goal = cmd_pose + vec4_t(avoidance_vector.x(), avoidance_vector.y(), avoidance_vector.z(), 0.0);
    startSendingWaypoints({new_goal});
    RCLCPP_INFO(get_logger(), "[Bumper]: Avoiding obstacle by moving from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]", uav_pose.x(), uav_pose.y(),
                uav_pose.z(), new_goal.x(), new_goal.y(), new_goal.z());
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
        safe_obstacle_distance_, euclidean_distance_cutoff_, planning_tree_resolution_, distance_penalty_, greedy_penalty_,
        min_altitude_, max_altitude_, planning_timeout_, max_waypoint_distance_, unknown_is_occupied_
      );
    const vec4_t uav_pose = get_mutexed(uav_pose_mutex_, uav_pose_);
    const vec4_t cmd_pose = get_mutexed(cmd_pose_mutex_, cmd_pose_);
  
    const octomap::point3d planning_goal = toPoint3d(goal);
    octomap::point3d planning_start;
    if ((cmd_pose.head<3>() - uav_pose.head<3>()).norm() <= navigation_tolerance_)
      planning_start = toPoint3d(cmd_pose);
    else
      planning_start = toPoint3d(uav_pose);
  
    const auto [path, result] = planner.findPath(
        planning_start, planning_goal, mapping_tree, planning_timeout_, std::bind(&Navigation::visualizeTree, this, _1),
        std::bind(&Navigation::visualizeExpansions, this, _1, _2, _3)
      );
  
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
          const vec3_t w_start = to_eigen(path.front());
          const vec3_t w_end = to_eigen(path.back());
          const double path_start_end_dist = (w_end - w_start).norm();
          // use sradians::dist instead of std::abs to correctly compare two cyclic values in range [-pi, pi]
          if (path_start_end_dist < planning_tree_resolution_)
          {
            if (sradians::dist(uav_pose.w(), goal.w()) > max_heading_step_)
            {
              append_goal = true;
              RCLCPP_INFO(get_logger(), "turning in one spot");
            }
            else
            {
              path_valid = false;
              RCLCPP_WARN(get_logger(), "Found a path that is too short.");
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
    planned_path.reserve(resampled.size()+1);
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
      planned_path.push_back(goal);
  
    return {planned_path, goal_reached};
  }
  //}

  /* bumperCheckObstacles //{ */
  bool Navigation::bumperCheckObstacles(const fog_msgs::msg::ObstacleSectors& bumper_msg)
  {
    for (int i = 0; i < int(bumper_msg.n_horizontal_sectors); i++)
    {
      // no reading available - skip it
      if (bumper_msg.sectors.at(i) < 0)
        continue;

      // if the sector is below the safe distance
      if (bumper_msg.sectors.at(i) <= safe_obstacle_distance_ * bumper_distance_factor_)
        return true;
    }
    return false;
  }
  //}

  /* bumperGetAvoidanceVector//{ */
  vec3_t Navigation::bumperGetAvoidanceVector(const fog_msgs::msg::ObstacleSectors& bumper_msg)
  {
    const vec4_t uav_pose = get_mutexed(uav_pose_mutex_, uav_pose_);
    const double sector_size = (2.0 * M_PI) / double(bumper_msg.n_horizontal_sectors);
    const vec3_t forward = vec3_t::UnitX();
    for (int i = 0; i < int(bumper_msg.n_horizontal_sectors); i++)
    {
      if (bumper_msg.sectors.at(i) < 0)
        continue;

      if (bumper_msg.sectors.at(i) <= safe_obstacle_distance_ * bumper_distance_factor_)
      {
        const anax_t rot(sector_size*i + M_PI + uav_pose.w(), vec3_t::UnitZ());
        const vec3_t avoidance_vector = rot * ((safe_obstacle_distance_ - bumper_msg.sectors.at(i) + planning_tree_resolution_) * forward);
        return avoidance_vector;
      }
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
        const vec3_t dir_vec = max_waypoint_distance_*diff_vec.normalized();
        const vec3_t padded = prev_pt + dir_vec;
        const vec4_t padded_heading(padded.x(), padded.y(), padded.z(), end_heading);
        ret.push_back(padded_heading);
      }
      else
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

  /* waypointsToPath //{ */
  std::shared_ptr<fog_msgs::srv::Path::Request> Navigation::waypointsToPath(const std::vector<vec4_t>& waypoints)
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
    auto path_srv = std::make_shared<fog_msgs::srv::Path::Request>();
    path_srv->path = msg;
    // read and update the control_command_id_ atomically
    {
      std::scoped_lock lck(control_diags_mutex_);
      path_srv->mission_id = ++control_command_id_;
    }
    return path_srv;
  }
  //}

  // | ----------------- Command-sending methods ---------------- |

  /* startSendingWaypoints() method //{ */
  void Navigation::startSendingWaypoints(const std::vector<vec4_t>& waypoints)
  {
    for (const auto& w : waypoints)
      RCLCPP_INFO_STREAM(get_logger(), "       " << w.transpose());
    visualizePath(waypoints);
    publishFutureTrajectory(waypoints);
    const auto path_req = waypointsToPath(waypoints);
    RCLCPP_INFO_STREAM(get_logger(), "Sending mission #" << path_req->mission_id << " with " << waypoints.size() << " waypoints to the control interface:");
    local_path_future_ = local_path_client_->async_send_request(path_req);
  }
  //}

  /* hover //{ */
  // the caller must lock the following muteces:
  // state_mutex_
  // waypoints_mutex_
  void Navigation::hover()
  {
    waypoints_in_.clear();
    waypoint_current_it_ = 0;
    waypoint_state_ = waypoint_state_t::empty;
    const vec4_t cmd_pose = get_mutexed(cmd_pose_mutex_, cmd_pose_);
    startSendingWaypoints({cmd_pose});
    state_ = nav_state_t::idle;
  }
  //}

  // | -------------- Publish/visualization methods ------------- |

  /* publishDiagnostics //{ */
  std::string toupper(std::string s)
  {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::toupper(c); });
    return s;
  }

  void Navigation::publishDiagnostics()
  {
    fog_msgs::msg::NavigationDiagnostics msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = get_mutexed(octree_mutex_, octree_frame_);
    msg.state = toupper(to_string(state_));
    msg.current_waypoint_status = toupper(to_string(waypoint_state_));
    msg.waypoints_in_buffer = waypoints_in_.size();
    msg.bumper_active = state_ == nav_state_t::avoiding;
    msg.current_waypoint_id = waypoint_current_it_;
    msg.current_nav_goal.at(0) = waypoint_current_.x();
    msg.current_nav_goal.at(1) = waypoint_current_.y();
    msg.current_nav_goal.at(2) = waypoint_current_.z();
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
  void Navigation::visualizeTree(const octomap::OcTree& tree)
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

    for (auto it = tree.begin(); it != tree.end(); it++)
    {
      if (it->getValue() == TreeValue::OCCUPIED)
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
                                       const std::unordered_set<navigation::Node, HashFunction>& closed, const octomap::OcTree& tree)
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
      auto coords = tree.keyToCoord(it->key);
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
      auto coords = tree.keyToCoord(it->key);
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

    const vec4_t uav_pose = get_mutexed(uav_pose_mutex_, uav_pose_);
    std::vector<vec4_t> tmp_waypoints;
    tmp_waypoints.push_back(uav_pose);
    for (auto& w : waypoints)
      tmp_waypoints.push_back(w);

    for (size_t i = 1; i < tmp_waypoints.size(); i++)
    {
      geometry_msgs::msg::Point p1, p2;
      std_msgs::msg::ColorRGBA c;
      p1.x = tmp_waypoints[i - 1].x();
      p1.y = tmp_waypoints[i - 1].y();
      p1.z = tmp_waypoints[i - 1].z();
      p2.x = tmp_waypoints[i].x();
      p2.y = tmp_waypoints[i].y();
      p2.z = tmp_waypoints[i].z();
      c = generateColor(0.1, double(i) / double(tmp_waypoints.size()), 0.1, 1);
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

  /* to_string() method //{ */
  std::string Navigation::to_string(const nav_state_t state) const
  {
    switch (state)
    {
      case nav_state_t::not_ready: return "not_ready";
      case nav_state_t::idle: return "idle";
      case nav_state_t::planning: return "planning";
      case nav_state_t::commanding: return "commanding";
      case nav_state_t::moving: return "moving";
      case nav_state_t::avoiding: return "avoiding";
    }
    return "unknown";
  }
  
  std::string Navigation::to_string(const waypoint_state_t state) const
  {
    switch (state)
    {
      case waypoint_state_t::empty: return "empty";
      case waypoint_state_t::ongoing: return "ongoing";
      case waypoint_state_t::reached: return "reached";
      case waypoint_state_t::unreachable: return "unreachable";
    }
    return "unknown";
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

