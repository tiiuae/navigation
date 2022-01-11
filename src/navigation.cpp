// clang: MatousFormat

// change the default Eigen formatting for prettier printing (has to be done before including Eigen)
#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]")

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

//}

using namespace std::placeholders;

namespace navigation
{
  using vec3_t = Eigen::Vector3d;
  using vec4_t = Eigen::Vector4d;
  using quat_t = Eigen::Quaterniond;
  using anax_t = Eigen::AngleAxisd;

  /* helper functions //{ */
  
  double getYaw(const geometry_msgs::msg::Quaternion& q)
  {
    return atan2(2.0 * (q.z * q.w + q.x * q.y), -1.0 + 2.0 * (q.w * q.w + q.x * q.x));
  }
  
  geometry_msgs::msg::Quaternion yawToQuaternionMsg(const double& yaw)
  {
    geometry_msgs::msg::Quaternion msg;
    quat_t q =
        anax_t(0, vec3_t::UnitX()) * anax_t(0, vec3_t::UnitY()) * anax_t(yaw, vec3_t::UnitZ());
    msg.w = q.w();
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    return msg;
  }
  
  octomap::point3d toPoint3d(const vec4_t& vec)
  {
    octomap::point3d p;
    p.x() = (float)vec.x();
    p.y() = (float)vec.y();
    p.z() = (float)vec.z();
    return p;
  }
  
  double nanosecondsToSecs(const int64_t nanoseconds)
  {
    return nanoseconds / 1e9;
  }
  
  //}

  /* class Navigation //{ */
  class Navigation : public rclcpp::Node
  {
  public:
    Navigation(rclcpp::NodeOptions options);

  private:
    // internal variables
    bool is_initialized_ = false;
    bool getting_octomap_ = false;
    bool getting_odometry_ = false;
    bool getting_desired_pose_ = false;
    bool getting_control_diagnostics_ = false;
    bool visualize_planner_ = true;
    bool show_unoccupied_ = false;
    bool override_previous_commands_ = false;
    bool bumper_active_ = false;
    bool manual_control_ = false;
    bool airborne_ = false;

    std::string parent_frame_;
    int replanning_counter_ = 0;

    bool control_moving_ = false;
    bool goal_reached_ = false;
    bool hover_requested_ = false;

    vec4_t uav_pos_;
    vec4_t desired_pose_;
    vec4_t current_goal_;
    vec4_t last_goal_;
    std::mutex octree_mutex_;
    std::shared_ptr<octomap::OcTree> octree_;

    std::mutex state_mutex_;
    enum nav_state_t
    {
      idle = 0,
      planning,
      commanding,
      moving,
      avoiding
    }  state_ = nav_state_t::idle;
    enum waypoint_state_t
    {
      empty = 0,
      ongoing,
      reached,
      unreachable
    } waypoint_state_ = waypoint_state_t::empty;

    std::deque<vec4_t> waypoints_in_;
    size_t current_waypoint_id_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::TimerBase::SharedPtr execution_timer_;
    void navigationRoutine();

    void state_navigation_idle();
    void state_navigation_planning();
    void state_navigation_commanding();
    void state_navigation_moving();
    void state_navigation_avoiding();

    std::pair<std::vector<vec4_t>, bool> planPath(const vec4_t& goal, std::shared_ptr<octomap::OcTree> mapping_tree);

    std::mutex bumper_mutex_;
    std::unique_ptr<fog_msgs::msg::ObstacleSectors> bumper_msg_;

    // params
    double euclidean_distance_cutoff_;
    double safe_obstacle_distance_;
    double bumper_distance_factor_;
    double navigation_tolerance_;
    bool unknown_is_occupied_;
    double min_altitude_;
    double max_altitude_;
    double max_goal_distance_;
    double distance_penalty_;
    double greedy_penalty_;
    double planning_tree_resolution_;
    double max_waypoint_distance_;
    double max_yaw_step_;
    double planning_timeout_;
    int replanning_limit_;
    double replanning_distance_;
    double main_update_rate_;
    bool bumper_enabled_;

    // visualization params
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
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr goto_subscriber_;
    rclcpp::Subscription<fog_msgs::msg::ControlInterfaceDiagnostics>::SharedPtr control_diagnostics_subscriber_;
    rclcpp::Subscription<fog_msgs::msg::ObstacleSectors>::SharedPtr bumper_subscriber_;

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


    rclcpp::Client<fog_msgs::srv::Path>::SharedPtr local_path_client_;
    rclcpp::Client<fog_msgs::srv::Path>::SharedFuture local_path_future_;

    rclcpp::Client<fog_msgs::srv::WaypointToLocal>::SharedPtr waypoint_to_local_client_;
    rclcpp::Client<fog_msgs::srv::PathToLocal>::SharedPtr path_to_local_client_;

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
    void visualizeTree(const octomap::OcTree& tree);
    void visualizeExpansions(const std::unordered_set<navigation::Node, HashFunction> open, const std::unordered_set<navigation::Node, HashFunction>& closed,
                             const octomap::OcTree& tree);
    void visualizePath(const std::vector<vec4_t>& waypoints);
    void visualizeGoals(const std::deque<vec4_t>& waypoints);


    std::vector<vec4_t> resamplePath(const std::vector<octomap::point3d>& waypoints, const double start_yaw, const double end_yaw);

    std::shared_ptr<fog_msgs::srv::Path::Request> waypointsToPath(const std::vector<vec4_t>& waypoints);
    void startSendingWaypoints(const std::vector<vec4_t>& waypoints);
    void hover();

    void publishDiagnostics();
    void publishFutureTrajectory(const std::vector<vec4_t>& waypoints);

    // bumper
    bool bumperCheckObstacles(const fog_msgs::msg::ObstacleSectors& bumper_msg);
    vec3_t bumperGetAvoidanceVector(const fog_msgs::msg::ObstacleSectors& bumper_msg);

    std_msgs::msg::ColorRGBA generateColor(const double r, const double g, const double b, const double a);

    template <class T>
    bool parse_param(const std::string& param_name, T& param_dest);

    std::string to_string(const nav_state_t state) const;
    std::string to_string(const waypoint_state_t state) const;
  };
  //}

  template <typename T>
  bool future_ready(const std::shared_future<T>& f);
  vec3_t to_eigen(const octomath::Vector3& vec);
  vec4_t to_eigen(const octomath::Vector3& vec, const double yaw);

  /* constructor //{ */
  Navigation::Navigation(rclcpp::NodeOptions options) : Node("navigation", options)
  {

    RCLCPP_INFO(get_logger(), "Initializing...");

    /* parse params from config file //{ */
    RCLCPP_INFO(get_logger(), "-------------- Loading parameters --------------");
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
    loaded_successfully &= parse_param("visualization.expansions_points_scale", expansions_points_scale_);
    loaded_successfully &= parse_param("visualization.path_points_scale", path_points_scale_);
    loaded_successfully &= parse_param("visualization.goal_points_scale", goal_points_scale_);

    loaded_successfully &= parse_param("bumper.enabled", bumper_enabled_);

    if (!loaded_successfully)
    {
      const std::string str = "Could not load all non-optional parameters. Shutting down.";
      RCLCPP_ERROR_STREAM(get_logger(), str);
      rclcpp::shutdown();
      return;
    }
    //}

    current_waypoint_id_ = 0;
    bumper_msg_.reset();

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;

    // publishers
    binary_tree_publisher_ = create_publisher<visualization_msgs::msg::Marker>("~/binary_tree_markers_out", 1);
    expansion_publisher_ = create_publisher<visualization_msgs::msg::Marker>("~/expansion_markers_out", 1);
    path_publisher_ = create_publisher<visualization_msgs::msg::Marker>("~/path_markers_out", 1);
    goal_publisher_ = create_publisher<visualization_msgs::msg::Marker>("~/goal_markers_out", 1);
    status_publisher_ = create_publisher<std_msgs::msg::String>("~/status_out", 1);
    future_trajectory_publisher_ = create_publisher<fog_msgs::msg::FutureTrajectory>("~/future_trajectory_out", 1);
    diagnostics_publisher_ = create_publisher<fog_msgs::msg::NavigationDiagnostics>("~/diagnostics_out", 5);

    // subscribers
    odometry_subscriber_ = create_subscription<nav_msgs::msg::Odometry>("~/odometry_in", 1, std::bind(&Navigation::odometryCallback, this, _1));
    desired_pose_subscriber_ =
        create_subscription<geometry_msgs::msg::PoseStamped>("~/desired_pose_in", 1, std::bind(&Navigation::desiredPoseCallback, this, _1));
    goto_subscriber_ = create_subscription<nav_msgs::msg::Path>("~/goto_in", 1, std::bind(&Navigation::gotoCallback, this, _1));
    control_diagnostics_subscriber_ = create_subscription<fog_msgs::msg::ControlInterfaceDiagnostics>(
        "~/control_diagnostics_in", 1, std::bind(&Navigation::controlDiagnosticsCallback, this, _1));
    octomap_subscriber_ = create_subscription<octomap_msgs::msg::Octomap>("~/octomap_in", 1, std::bind(&Navigation::octomapCallback, this, _1), sub_opt);
    bumper_subscriber_ = create_subscription<fog_msgs::msg::ObstacleSectors>("~/bumper_in", 1, std::bind(&Navigation::bumperCallback, this, _1), sub_opt);

    // clients
    local_path_client_ = create_client<fog_msgs::srv::Path>("~/local_path_out");
    waypoint_to_local_client_ = create_client<fog_msgs::srv::WaypointToLocal>("~/waypoint_to_local_out");
    path_to_local_client_ = create_client<fog_msgs::srv::PathToLocal>("~/path_to_local_out");

    // service handlers
    goto_trigger_service_ = create_service<std_srvs::srv::Trigger>("~/goto_trigger_in", std::bind(&Navigation::gotoTriggerCallback, this, _1, _2));
    hover_service_ = create_service<std_srvs::srv::Trigger>("~/hover_in", std::bind(&Navigation::hoverCallback, this, _1, _2));
    local_waypoint_service_ = create_service<fog_msgs::srv::Vec4>("~/local_waypoint_in", std::bind(&Navigation::localWaypointCallback, this, _1, _2));
    local_path_service_ = create_service<fog_msgs::srv::Path>("~/local_path_in", std::bind(&Navigation::localPathCallback, this, _1, _2));
    gps_waypoint_service_ = create_service<fog_msgs::srv::Vec4>("~/gps_waypoint_in", std::bind(&Navigation::gpsWaypointCallback, this, _1, _2));
    gps_path_service_ = create_service<fog_msgs::srv::Path>("~/gps_path_in", std::bind(&Navigation::gpsPathCallback, this, _1, _2));

    // timers
    execution_timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / main_update_rate_), std::bind(&Navigation::navigationRoutine, this), callback_group_);

    if (max_waypoint_distance_ <= 0)
    {
      max_waypoint_distance_ = replanning_distance_;
    }

    is_initialized_ = true;
    RCLCPP_INFO(get_logger(), "Initialized");
  }
  //}

  /* octomapCallback //{ */
  void Navigation::octomapCallback(const octomap_msgs::msg::Octomap::UniquePtr msg)
  {
    getting_octomap_ = true;
    parent_frame_ = msg->header.frame_id;
    RCLCPP_INFO_ONCE(get_logger(), "Getting octomap");
    octomap_msgs::msg::Octomap map_msg = *msg;

    auto treePtr = octomap_msgs::fullMsgToMap(*msg);

    if (!treePtr)
    {
      RCLCPP_WARN(get_logger(), "Octomap message is waypoint_state_t::empty!");
    } else
    {
      std::scoped_lock lock(octree_mutex_);
      octree_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(treePtr));
    }
  }
  //}

  /* odometryCallback //{ */
  void Navigation::odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg)
  {
    if (!is_initialized_)
    {
      return;
    }

    getting_odometry_ = true;
    RCLCPP_INFO_ONCE(get_logger(), "Getting odometry");
    uav_pos_[0] = msg->pose.pose.position.x;
    uav_pos_[1] = msg->pose.pose.position.y;
    uav_pos_[2] = msg->pose.pose.position.z;
    uav_pos_[3] = getYaw(msg->pose.pose.orientation);
  }
  //}

  /* desiredPoseCallback //{ */
  void Navigation::desiredPoseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr msg)
  {
    if (!is_initialized_)
    {
      return;
    }

    getting_desired_pose_ = true;
    RCLCPP_INFO_ONCE(get_logger(), "Getting desired pose");
    desired_pose_[0] = msg->pose.position.x;
    desired_pose_[1] = msg->pose.position.y;
    desired_pose_[2] = msg->pose.position.z;
    desired_pose_[3] = getYaw(msg->pose.orientation);
  }
  //}

  /* controlDiagnosticsCallback //{ */
  void Navigation::controlDiagnosticsCallback(const fog_msgs::msg::ControlInterfaceDiagnostics::UniquePtr msg)
  {
    if (!is_initialized_)
    {
      return;
    }
    getting_control_diagnostics_ = true;
    RCLCPP_INFO_ONCE(get_logger(), "Getting control_interface diagnostics");
    control_moving_ = msg->moving;
    goal_reached_ = msg->mission_finished;
    manual_control_ = msg->manual_control;
    airborne_ = msg->airborne;
  }
  //}

  /* bumperCallback //{ */
  void Navigation::bumperCallback(const fog_msgs::msg::ObstacleSectors::UniquePtr msg)
  {
    if (!is_initialized_)
    {
      return;
    }

    RCLCPP_INFO_ONCE(get_logger(), "Getting bumper msgs");
    std::scoped_lock lock(bumper_mutex_);
    bumper_msg_ = std::unique_ptr<fog_msgs::msg::ObstacleSectors>{new fog_msgs::msg::ObstacleSectors{*msg}};
  }
  //}

  /* gotoCallback //{ */
  void Navigation::gotoCallback(const nav_msgs::msg::Path::UniquePtr msg)
  {

    if (!is_initialized_)
    {
      RCLCPP_ERROR(get_logger(), "Goto rejected, node not initialized");
      return;
    }

    if (!getting_octomap_)
    {
      RCLCPP_ERROR(get_logger(), "Goto rejected, octomap not received");
      return;
    }

    if (!getting_control_diagnostics_)
    {
      RCLCPP_ERROR(get_logger(), "Goto rejected, control_interface diagnostics not received");
      return;
    }

    if (!getting_desired_pose_)
    {
      RCLCPP_ERROR(get_logger(), "Goto rejected, missing desired pose");
      return;
    }

    if (!airborne_)
    {
      RCLCPP_ERROR(get_logger(), "Goto rejected, vehicle not flying normally");
      return;
    }

    if (msg->poses.empty())
    {
      RCLCPP_ERROR(get_logger(), "Goto rejected, path input does not contain any waypoints");
      return;
    }

    if (state_ != nav_state_t::idle)
    {
      if (!override_previous_commands_)
      {
        RCLCPP_ERROR(get_logger(), "Goto rejected, vehicle not nav_state_t::idle");
        return;
      } else
      {
        RCLCPP_INFO(get_logger(), "Override previous navigation commands");
        hover();
      }
    }

    RCLCPP_INFO(get_logger(), "Recieved %ld waypoints", msg->poses.size());

    waypoints_in_.clear();
    current_waypoint_id_ = 0;
    for (const auto& p : msg->poses)
    {
      vec4_t point;
      point[0] = p.pose.position.x;
      point[1] = p.pose.position.y;
      point[2] = p.pose.position.z;
      point[3] = getYaw(p.pose.orientation);
      waypoints_in_.push_back(point);
    }
    RCLCPP_INFO(get_logger(), "Waiting for nav_state_t::planning trigger");
  }
  //}

  /* gotoTriggerCallback //{ */
  bool Navigation::gotoTriggerCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!is_initialized_)
    {
      response->message = "Goto rejected, node not initialized";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!getting_octomap_)
    {
      response->message = "Goto rejected, octomap not received";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!getting_control_diagnostics_)
    {
      response->message = "Goto rejected, control_interface diagnostics not received";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!getting_desired_pose_)
    {
      response->message = "Goto rejected, missing desired pose";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!airborne_)
    {
      response->message = "Goto rejected, vehicle not flying normally";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (current_waypoint_id_ >= waypoints_in_.size() || waypoints_in_.empty())
    {
      response->message = "Goto rejected, no waypoint provided";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (state_ != nav_state_t::idle)
    {
      if (!override_previous_commands_)
      {
        RCLCPP_ERROR(get_logger(), "Goto rejected, vehicle not nav_state_t::idle");
        return true;
      } else
      {
        RCLCPP_INFO(get_logger(), "Override previous navigation commands");
        hover();
      }
    }

    response->message = "nav_state_t::planning started";
    response->success = true;
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    state_ = nav_state_t::planning;
    return true;
  }
  //}

  /* localPathCallback //{ */
  bool Navigation::localPathCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::Path::Request> request,
                                     std::shared_ptr<fog_msgs::srv::Path::Response> response)
  {
    if (!is_initialized_)
    {
      response->message = "Path rejected, node not initialized";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!getting_octomap_)
    {
      response->message = "Path rejected, octomap not received";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!getting_control_diagnostics_)
    {
      response->message = "Path rejected, control_interface diagnostics not received";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!getting_desired_pose_)
    {
      response->message = "Path rejected, missing desired pose";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!airborne_)
    {
      response->message = "Path rejected, vehicle not flying normally";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (request->path.poses.empty())
    {
      response->message = "Path rejected, path input does not contain any waypoints";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (state_ != nav_state_t::idle)
    {
      if (!override_previous_commands_)
      {
        response->message = "Path rejected, vehicle not nav_state_t::idle";
        response->success = false;
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return true;
      } else
      {
        RCLCPP_INFO(get_logger(), "Override previous navigation commands");
        hover();
      }
    }

    waypoints_in_.clear();
    current_waypoint_id_ = 0;
    for (const auto& p : request->path.poses)
    {
      vec4_t point;
      point[0] = p.pose.position.x;
      point[1] = p.pose.position.y;
      point[2] = p.pose.position.z;
      point[3] = getYaw(p.pose.orientation);
      waypoints_in_.push_back(point);
    }

    RCLCPP_INFO(get_logger(), "Recieved %ld waypoints. nav_state_t::planning started", request->path.poses.size());
    response->message = "nav_state_t::planning started";
    response->success = true;
    state_ = nav_state_t::planning;
    return true;
  }
  //}

  /* gpsPathCallback //{ */
  bool Navigation::gpsPathCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::Path::Request> request,
                                   std::shared_ptr<fog_msgs::srv::Path::Response> response)
  {
    if (!is_initialized_)
    {
      response->message = "Path rejected, node not initialized";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!getting_octomap_)
    {
      response->message = "Path rejected, octomap not received";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!getting_control_diagnostics_)
    {
      response->message = "Path rejected, control_interface diagnostics not received";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!getting_desired_pose_)
    {
      response->message = "Path rejected, missing desired pose";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!airborne_)
    {
      response->message = "Path rejected, vehicle not flying normally";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (request->path.poses.empty())
    {
      response->message = "Path rejected, path input does not contain any waypoints";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (state_ != nav_state_t::idle)
    {
      if (!override_previous_commands_)
      {
        response->message = "Path rejected, vehicle not nav_state_t::idle";
        response->success = false;
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return true;
      } else
      {
        RCLCPP_INFO(get_logger(), "Override previous navigation commands");
        hover();
      }
    }

    auto convert_path_srv = std::make_shared<fog_msgs::srv::PathToLocal::Request>();
    convert_path_srv->path = request->path;
    RCLCPP_INFO(get_logger(), "Calling Path transform");
    auto call_result = path_to_local_client_->async_send_request(convert_path_srv, std::bind(&Navigation::pathFutureCallback, this, std::placeholders::_1));

    response->message = "Processing path";
    response->success = true;
    return true;
  }
  //}

  /* localWaypointCallback //{ */
  bool Navigation::localWaypointCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
                                         std::shared_ptr<fog_msgs::srv::Vec4::Response> response)
  {
    if (!is_initialized_)
    {
      response->message = "Goto rejected, node not initialized";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!getting_octomap_)
    {
      response->message = "Goto rejected, octomap not received";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!getting_control_diagnostics_)
    {
      response->message = "Goto rejected, control_interface diagnostics not received";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!getting_desired_pose_)
    {
      response->message = "Goto rejected, missing desired pose";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!airborne_)
    {
      response->message = "Goto rejected, vehicle not flying normally";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (state_ != nav_state_t::idle)
    {
      if (!override_previous_commands_)
      {
        response->message = "Goto rejected, vehicle not nav_state_t::idle";
        response->success = false;
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return true;
      } else
      {
        RCLCPP_INFO(get_logger(), "Override previous navigation commands");
        hover();
      }
    }

    waypoints_in_.clear();
    current_waypoint_id_ = 0;
    vec4_t point;
    point[0] = request->goal[0];
    point[1] = request->goal[1];
    point[2] = request->goal[2];
    point[3] = request->goal[3];
    waypoints_in_.push_back(point);

    RCLCPP_INFO(get_logger(), "Waypoint set: %.2f, %.2f, %.2f, %.2f", point.x(), point.y(), point.z(), point.w());

    RCLCPP_INFO(get_logger(), "nav_state_t::planning started");
    state_ = nav_state_t::planning;

    response->message = "nav_state_t::planning started";
    response->success = true;
    return true;
  }
  //}

  /* gpsWaypointCallback //{ */
  bool Navigation::gpsWaypointCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
                                       std::shared_ptr<fog_msgs::srv::Vec4::Response> response)
  {
    if (!is_initialized_)
    {
      response->message = "Goto rejected, node not initialized";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!getting_octomap_)
    {
      response->message = "Goto rejected, octomap not received";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!getting_control_diagnostics_)
    {
      response->message = "Goto rejected, control_interface diagnostics not received";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!getting_desired_pose_)
    {
      response->message = "Goto rejected, missing desired pose";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!airborne_)
    {
      response->message = "Goto rejected, vehicle not flying normally";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (state_ != nav_state_t::idle)
    {
      if (!override_previous_commands_)
      {
        response->message = "Goto rejected, vehicle not nav_state_t::idle";
        response->success = false;
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return true;
      } else
      {
        RCLCPP_INFO(get_logger(), "Override previous navigation commands");
        hover();
      }
    }

    RCLCPP_INFO(get_logger(), "Creating coord transform request");
    auto waypoint_convert_srv = std::make_shared<fog_msgs::srv::WaypointToLocal::Request>();
    waypoint_convert_srv->latitude_deg = request->goal[0];
    waypoint_convert_srv->longitude_deg = request->goal[1];
    waypoint_convert_srv->relative_altitude_m = request->goal[2];
    waypoint_convert_srv->yaw = request->goal[3];
    RCLCPP_INFO(get_logger(), "Calling coord transform");
    auto call_result =
        waypoint_to_local_client_->async_send_request(waypoint_convert_srv, std::bind(&Navigation::waypointFutureCallback, this, std::placeholders::_1));


    response->message = "Processing goto";
    response->success = true;
    return true;
  }
  //}

  /* hoverCallback //{ */
  bool Navigation::hoverCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!is_initialized_)
    {
      response->message = "Hover rejected, node not initialized";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!getting_control_diagnostics_)
    {
      response->message = "Hover rejected, control_interface diagnostics not received";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (!airborne_)
    {
      response->message = "Hover rejected, vehicle not flying normally";
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }

    if (state_ == nav_state_t::idle)
    {
      response->message = "Hover not necessary, vehicle is nav_state_t::idle";
      response->success = false;
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return true;
    }

    hover_requested_ = true;
    response->message = "Navigation stopped. Hovering";
    response->success = true;
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    return true;
  }
  //}

  /* navigationRoutine //{ */
  void Navigation::navigationRoutine()
  {
    std::scoped_lock lck(state_mutex_);

    if (is_initialized_ && getting_octomap_ && getting_control_diagnostics_ && getting_odometry_ && getting_desired_pose_)
    {
      RCLCPP_INFO_ONCE(get_logger(), "NAVIGATION IS READY");

      if (bumper_enabled_)
      {
        std::scoped_lock lock(bumper_mutex_);
        if (bumper_msg_ == nullptr || nanosecondsToSecs((get_clock()->now() - bumper_msg_->header.stamp).nanoseconds()) > 1.0)
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Missing bumper data calling hover.");
          hover();
          state_ = nav_state_t::idle;
          waypoint_state_ = waypoint_state_t::empty;
        }
      }

      switch (state_)
      {
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
    }
    else
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Octomap: %s, ControlDiagnostics: %s, Odometry: %s, DesiredPose: %s",
                           getting_octomap_ ? "OK" : "MISSING", getting_control_diagnostics_ ? "OK" : "MISSING",
                           getting_odometry_ ? "OK" : "MISSING", getting_desired_pose_ ? "OK" : "MISSING");
    }

    std_msgs::msg::String msg;
    msg.data = to_string(state_);
    status_publisher_->publish(msg);
    publishDiagnostics();
  }
  //}

  /* state_navigation_idle() method //{ */
  void Navigation::state_navigation_idle()
  {
    if (bumper_enabled_)
    {
      std::scoped_lock lock(bumper_mutex_);

      const bool obstacle_detected = bumperCheckObstacles(*bumper_msg_);
      if (!bumper_active_)
      {
        if (obstacle_detected)
        {
          RCLCPP_WARN(get_logger(), "[Bumper] - obstacle in proximity! Starting avoidance");
          state_ = nav_state_t::avoiding;
          return;
        }
      }
      else if (!obstacle_detected)
      {
        RCLCPP_WARN(get_logger(), "Deactivating bumper");
        bumper_active_ = false;
        return;
      }
    }

    if (waypoints_in_.empty())
    {
      const std::vector<vec4_t> waypoints = {uav_pos_};
      publishFutureTrajectory(waypoints);
    }
    else
    {
      state_ = nav_state_t::planning;
    }
  }
  //}

  /* state_navigation_planning() method //{ */
  void Navigation::state_navigation_planning()
  {
    std::scoped_lock lock(octree_mutex_);

    /* initial checks //{ */
    
    if (manual_control_)
    {
      RCLCPP_INFO(get_logger(), "Manual control enabled. Clearing waypoints and switching to idle");
      waypoints_in_.clear();
      state_ = nav_state_t::idle;
      waypoint_state_ = waypoint_state_t::empty;
      return;
    }
    
    if (hover_requested_)
    {
      RCLCPP_WARN(get_logger(), "Hover requested! Aborting nav_state_t::planning and swiching to commanding");
      waypoints_in_.clear();
      hover();
      state_ = nav_state_t::commanding;
      waypoint_state_ = waypoint_state_t::empty;
      hover_requested_ = false;
      return;
    }
    
    if (octree_ == nullptr || octree_->size() < 1)
    {
      RCLCPP_WARN(get_logger(), "Octomap is nullptr or empty! Aborting nav_state_t::planning and swiching to idle");
      state_ = nav_state_t::idle;
      waypoint_state_ = waypoint_state_t::empty;
      return;
    }
    
    if (waypoints_in_.empty() || current_waypoint_id_ >= waypoints_in_.size())
    {
      RCLCPP_WARN(get_logger(), "No navigation goals available. Switching to idle");
      state_ = nav_state_t::idle;
      return;
    }
    
    //}
  
    // if we got here, let's plan!
    const vec4_t goal = waypoints_in_.at(current_waypoint_id_);
    current_goal_ = last_goal_ = goal;
  
    RCLCPP_INFO_STREAM(get_logger(), "Waypoint [" << goal.transpose() << "] set as the next goal #" << current_waypoint_id_ << "/" << waypoints_in_.size() << ".");
  
    visualizeGoals(waypoints_in_);

    // start the actual nav_state_t::planning
    const auto [waypoints_out, goal_reached] = planPath(goal, octree_);
    // evaluate the result of path nav_state_t::planning
    // if no path was found, check the cause
    if (waypoints_out.empty())
    {
      // the current goal is already waypoint_state_t::reached!
      if (goal_reached)
      {
        replanning_counter_ = 0; // nav_state_t::planning for this goal is done, reset the retry counter
        waypoint_state_ = waypoint_state_t::reached;
        // and it was the final goal that we've got
        if (current_waypoint_id_ >= waypoints_in_.size())
        {
          RCLCPP_INFO(get_logger(), "The last provided navigation goal has been visited. Switching to idle");
          state_ = nav_state_t::idle;
        }
        // if it was not the final goal, let's go to the next goal
        else
        {
          RCLCPP_INFO(get_logger(), "Navigation goal #%lu/%lu visited, continuing.", current_waypoint_id_, waypoints_in_.size());
          current_waypoint_id_++;
        }
      }
      // the current goal is not waypoint_state_t::reached, yet the path is waypoint_state_t::empty. this means that the nav_state_t::planning failed
      else
      {
        RCLCPP_INFO(get_logger(), "A path to the current goal was not found.");
        replanning_counter_++;
        // check if the number of retries is too high
        if (replanning_counter_ >= replanning_limit_)
        {
          RCLCPP_ERROR(get_logger(), "No path produced after %d repeated attempts. Please provide a new waypoint. Switching to idle.", replanning_counter_);
          replanning_counter_ = 0;
          state_ = nav_state_t::idle;
          waypoint_state_ = waypoint_state_t::unreachable;
        }
      }
    }
    // a path was found, let's follow it
    else
    {
      replanning_counter_ = 0; // nav_state_t::planning was successful, so reset the retry counter
      startSendingWaypoints(waypoints_out); // start sending the waypoints to the command interface
      // change the state accordingly
      state_ = nav_state_t::commanding;
      waypoint_state_ = waypoint_state_t::ongoing;
    }
  }
  //}

  /* state_navigation_commanding() method //{ */
  void Navigation::state_navigation_commanding()
  {
    if (manual_control_)
    {
      RCLCPP_INFO(get_logger(), "Manual control enabled, switching to idle");
      waypoints_in_.clear();
      state_ = nav_state_t::idle;
      return;
    }

    if (hover_requested_)
    {
      RCLCPP_WARN(get_logger(), "Hover requested! Aborting nav_state_t::planning and swiching to commanding");
      waypoints_in_.clear();
      hover();
      state_ = nav_state_t::commanding;
      waypoint_state_ = waypoint_state_t::empty;
      hover_requested_ = false;
      return;
    }

    if (future_ready(local_path_future_))
    {
      const auto resp_ptr = local_path_future_.get();
      // not sure if this can even happen - ROS2 documentation on this is waypoint_state_t::empty...
      if (resp_ptr == nullptr)
      {
        RCLCPP_WARN(get_logger(), "Failed to call local path service of control interface!");
        state_ = nav_state_t::idle;
      }
      // this may happen, but we can't do much - control interface is probably not ready or something
      else if (!resp_ptr->success)
      {
        RCLCPP_WARN_STREAM(get_logger(), "Failed to set local path to control interface: " << resp_ptr->message);
        state_ = nav_state_t::idle;
      }
      // otherwise, all went well and the vehicle should be nav_state_t::moving by this point
      else
      {
        RCLCPP_INFO(get_logger(), "Set local path to control interface. Switching state to nav_state_t::moving.");
        state_ = nav_state_t::moving;
      }
    }
  }
  //}

  /* state_navigation_moving() method //{ */
  void Navigation::state_navigation_moving()
  {
    if (manual_control_)
    {
      RCLCPP_INFO(get_logger(), "Manual control enabled, switching to idle");
      waypoints_in_.clear();
      state_ = nav_state_t::idle;
      return;
    }

    if (hover_requested_)
    {
      RCLCPP_WARN(get_logger(), "Hover requested! Aborting nav_state_t::planning and swiching to commanding");
      waypoints_in_.clear();
      hover();
      state_ = nav_state_t::commanding;
      waypoint_state_ = waypoint_state_t::empty;
      hover_requested_ = false;
      return;
    }

    if (bumper_enabled_)
    {
      std::scoped_lock lock(bumper_mutex_);
      const bool obstacle_detected = bumperCheckObstacles(*bumper_msg_);
      if (!bumper_active_)
      {
        if (obstacle_detected)
        {
          RCLCPP_WARN(get_logger(), "[Bumper] - obstacle in proximity while moving! Calling hover");
          hover();
          state_ = nav_state_t::commanding;
          return;
        }
      }
    }

    replanning_counter_ = 0;
    if ((!control_moving_ && goal_reached_))
    {
      RCLCPP_INFO(get_logger(), "End of current segment reached");
      if (bumper_active_)
        state_ = nav_state_t::avoiding;
      else
        state_ = nav_state_t::planning;
    }
  }
  //}

  /* state_navigation_avoiding() method //{ */
  void Navigation::state_navigation_avoiding()
  {
    std::scoped_lock lock(bumper_mutex_);

    if (manual_control_)
    {
      RCLCPP_INFO(get_logger(), "Manual control enabled, switching to idle");
      waypoints_in_.clear();
      state_ = nav_state_t::idle;
      return;
    }
  
    const vec3_t avoidance_vector = bumperGetAvoidanceVector(*bumper_msg_);
    if (avoidance_vector.norm() == 0)
    {
      RCLCPP_INFO(get_logger(), "Nothing to avoid");
      state_ = nav_state_t::idle;
      return;
    }

    vec4_t new_goal = uav_pos_;
    new_goal.x() += avoidance_vector.x();
    new_goal.y() += avoidance_vector.y();
    new_goal.z() = desired_pose_.z() + avoidance_vector.z();  // using desired position instead of the actual Z, because the estimate may drift
    new_goal.w() = desired_pose_.w();

    RCLCPP_INFO(get_logger(), "[Bumper]: Avoiding obstacle by moving from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]", uav_pos_.x(), uav_pos_.y(),
                uav_pos_.z(), new_goal.x(), new_goal.y(), new_goal.z());

    startSendingWaypoints({new_goal});

    if (!bumper_active_)
    {
      RCLCPP_WARN(get_logger(), "Activating bumper");
      bumper_active_ = true;
    }
  
    state_ = nav_state_t::commanding;
  }
  //}

  /* planPath() method //{ */
  // returns a path to the goal (if found) and a bool, indicating if the UAV is already in the goal (if the path is waypoint_state_t::empty)
  // if the path is waypoint_state_t::empty and the bool is false, then the path could not be found or the found path is invalid
  std::pair<std::vector<vec4_t>, bool> Navigation::planPath(const vec4_t& goal, std::shared_ptr<octomap::OcTree> mapping_tree)
  {
    navigation::AstarPlanner planner = navigation::AstarPlanner(
        safe_obstacle_distance_, euclidean_distance_cutoff_, planning_tree_resolution_, distance_penalty_, greedy_penalty_,
        min_altitude_, max_altitude_, planning_timeout_, max_waypoint_distance_, unknown_is_occupied_
      );
  
    const octomap::point3d planning_goal = toPoint3d(goal);
    octomap::point3d planning_start;
    if ((desired_pose_.head<3>() - uav_pos_.head<3>()).norm() <= navigation_tolerance_)
      planning_start = toPoint3d(desired_pose_);
    else
      planning_start = toPoint3d(uav_pos_);
  
    const auto [path, result] = planner.findPath(
        planning_start, planning_goal, mapping_tree, planning_timeout_, std::bind(&Navigation::visualizeTree, this, _1),
        std::bind(&Navigation::visualizeExpansions, this, _1, _2, _3)
      );
  
    RCLCPP_INFO(get_logger(), "Planner returned %ld waypoints", path.size());
  
    bool path_valid;
    bool goal_reached;
    bool append_goal;
    /* evaluate based on the result and the current UAV position whether the desired goal is waypoint_state_t::reached and whether to append the goal to the output //{ */
    switch (result)
    {
      case GOAL_REACHED:
        /*  //{ */
        // if the current UAV heading is different from the desired, just turn
        if (std::abs(uav_pos_.w() - goal.w()) > max_yaw_step_)
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
          if (path_start_end_dist < planning_tree_resolution_)
          {
            if (std::abs(uav_pos_.w() - goal.w()) > max_yaw_step_)
            {
              append_goal = true;
              RCLCPP_INFO(get_logger(), "turning in one spot");
            }
            else
            {
              path_valid = false;
              RCLCPP_WARN(get_logger(), "Found a path that is path too short.");
            }
          }
        }
        //}
        break;
  
      case GOAL_IN_OBSTACLE:
        goal_reached = false;
        append_goal = false;
        path_valid = false;
        RCLCPP_WARN_STREAM(get_logger(), "Goal [" << goal.transpose() << "] is inside an inflated obstacle");
        break;
  
      case FAILURE:
      default:
        goal_reached = false;
        append_goal = false;
        path_valid = false;
        break;
    }
  
    //}

    // if path is not valid, just return an waypoint_state_t::empty path
    if (!path_valid)
      return {{}, goal_reached};
  
    // resample path and add yaw
    const std::vector<vec4_t> resampled = resamplePath(path, desired_pose_.w(), goal.w());
  
    // finally, start filling the buffer up to the maximal replanning distance
    std::vector<vec4_t> waypoints_out;
    waypoints_out.reserve(resampled.size()+1);
    for (const auto& w : resampled)
    {
      if ((w.head<3>() - desired_pose_.head<3>()).norm() <= replanning_distance_)
      {
        waypoints_out.push_back(w);
      } else
      {
        RCLCPP_INFO(get_logger(), "Path exceeding replanning distance");
        append_goal = false;
        break;
      }
    }
  
    if (append_goal)
      waypoints_out.push_back(goal);
  
    return {waypoints_out, goal_reached};
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

      // if the sector is under safe distance
      if (bumper_msg.sectors.at(i) <= safe_obstacle_distance_ * bumper_distance_factor_)
        return true;
    }
    return false;
  }
  //}

  /* bumperGetAvoidanceVector//{ */
  vec3_t Navigation::bumperGetAvoidanceVector(const fog_msgs::msg::ObstacleSectors& bumper_msg)
  {
    const double sector_size = (2.0 * M_PI) / double(bumper_msg.n_horizontal_sectors);
    const vec3_t forward = vec3_t::UnitX();
    for (int i = 0; i < int(bumper_msg.n_horizontal_sectors); i++)
    {
      if (bumper_msg.sectors.at(i) < 0)
        continue;

      if (bumper_msg.sectors.at(i) <= safe_obstacle_distance_ * bumper_distance_factor_)
      {
        const anax_t rot(sector_size*i + M_PI + uav_pos_.w(), vec3_t::UnitZ());
        const vec3_t avoidance_vector = rot * ((safe_obstacle_distance_ - bumper_msg.sectors[i] + planning_tree_resolution_) * forward);
        return avoidance_vector;
      }
    }
    return vec3_t::Zero();
  }
  //}

  /* resamplePath //{ */
  std::vector<vec4_t> Navigation::resamplePath(const std::vector<octomap::point3d>& waypoints, const double start_yaw, const double end_yaw)
  {
    std::vector<vec4_t> ret;

    if (waypoints.size() < 2)
    {
      for (auto& w : waypoints)
        ret.push_back(to_eigen(w, end_yaw));
      return ret;
    }

    ret.push_back(to_eigen(waypoints.front(), 0.0));

    size_t i = 1;
    while (i < waypoints.size())
    {
      double dist = std::sqrt(std::pow(ret.back().x() - waypoints[i].x(), 2) + std::pow(ret.back().y() - waypoints[i].y(), 2)
                              + std::pow(ret.back().z() - waypoints[i].z(), 2));
      if (dist > max_waypoint_distance_)
      {
        vec3_t direction;
        direction.x() = waypoints[i].x() - ret.back().x();
        direction.y() = waypoints[i].y() - ret.back().y();
        direction.z() = waypoints[i].z() - ret.back().z();
        direction = direction.normalized() * max_waypoint_distance_;

        vec4_t padded;
        padded.x() = ret.back().x() + direction.x();
        padded.y() = ret.back().y() + direction.y();
        padded.z() = ret.back().z() + direction.z();
        ret.push_back(padded);
      } else
      {
        ret.push_back(vec4_t(waypoints[i].x(), waypoints[i].y(), waypoints[i].z(), 0.0));
        i++;
      }
    }

    RCLCPP_INFO(get_logger(), "Padded %lu original waypoints to %lu points", waypoints.size(), ret.size());

    /* add yaw //{ */

    double delta_yaw = std::atan2(std::sin(end_yaw - start_yaw), std::cos(end_yaw - start_yaw));
    double yaw_step = delta_yaw / ret.size();
    RCLCPP_INFO(get_logger(), "Start yaw: %.2f, end yaw: %.2f, yaw step: %.2f", start_yaw, end_yaw, yaw_step);

    if (std::abs(yaw_step) <= max_yaw_step_)
    {
      ret.front().w() = start_yaw;
      for (size_t j = 1; j < ret.size(); j++)
      {
        ret[j].w() = ret[j - 1].w() + yaw_step;
      }
    } else
    {
      // resample again to limit yaw rate and avoid fast turning
      int resampling_factor = int(std::abs(yaw_step) / max_yaw_step_) + 1;
      RCLCPP_INFO(get_logger(), "Yaw step: %.2f is greater than max yaw step: %.2f", yaw_step, max_yaw_step_);
      RCLCPP_INFO(get_logger(), "Resampling factor: %d", resampling_factor);
      std::vector<vec4_t> resampled;
      for (const auto& p : ret)
      {
        for (int j = 0; j < resampling_factor; j++)
        {
          vec4_t v(p[0], p[1], p[2], p[3]);
          resampled.push_back(v);
        }
      }

      ret.clear();
      ret.insert(ret.end(), resampled.begin(), resampled.end());
      yaw_step = delta_yaw / ret.size();
      RCLCPP_INFO(get_logger(), "New yaw step: %.2f", yaw_step);
      ret.front().w() = start_yaw;
      for (size_t j = 1; j < ret.size(); j++)
      {
        ret[j].w() = ret[j - 1].w() + yaw_step;
      }
    }

    //}

    return ret;
  }
  //}

  /* waypointsToPath //{ */
  std::shared_ptr<fog_msgs::srv::Path::Request> Navigation::waypointsToPath(const std::vector<vec4_t>& waypoints)
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = parent_frame_;
    for (size_t i = 0; i < waypoints.size(); i++)
    {
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = waypoints[i].x();
      p.pose.position.y = waypoints[i].y();
      p.pose.position.z = waypoints[i].z();
      p.pose.orientation = yawToQuaternionMsg(waypoints[i].w());
      msg.poses.push_back(p);
    }
    auto path_srv = std::make_shared<fog_msgs::srv::Path::Request>();
    path_srv->path = msg;
    return path_srv;
  }
  //}

  /* startSendingWaypoints() method //{ */
  void Navigation::startSendingWaypoints(const std::vector<vec4_t>& waypoints)
  {
    RCLCPP_INFO(get_logger(), "Sending %ld waypoints to the control interface:", waypoints.size());
    for (const auto& w : waypoints)
    {
      RCLCPP_INFO(get_logger(), "       %.2f, %.2f, %.2f, %.2f", w.x(), w.y(), w.z(), w.w());
    }
    visualizePath(waypoints);
    publishFutureTrajectory(waypoints);
    const auto path_req = waypointsToPath(waypoints);
    local_path_future_ = local_path_client_->async_send_request(path_req);
  }
  //}

  /* hover //{ */
  void Navigation::hover()
  {
    startSendingWaypoints({desired_pose_});
  }
  //}

  /* publishDiagnostics //{ */
  void Navigation::publishDiagnostics()
  {
    fog_msgs::msg::NavigationDiagnostics msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = parent_frame_;
    msg.state = to_string(state_);
    msg.current_waypoint_status = to_string(waypoint_state_);
    msg.waypoints_in_buffer = waypoints_in_.size();
    msg.bumper_active = bumper_active_;
    msg.current_waypoint_id = current_waypoint_id_;
    msg.current_nav_goal.at(0) = current_goal_.x();
    msg.current_nav_goal.at(1) = current_goal_.y();
    msg.current_nav_goal.at(2) = current_goal_.z();
    msg.last_nav_goal.at(0) = last_goal_.x();
    msg.last_nav_goal.at(1) = last_goal_.y();
    msg.last_nav_goal.at(2) = last_goal_.z();
    diagnostics_publisher_->publish(msg);
  }
  //}

  /* publishFutureTrajectory //{ */
  void Navigation::publishFutureTrajectory(const std::vector<vec4_t>& waypoints)
  {
    fog_msgs::msg::FutureTrajectory msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = parent_frame_;
    for (const auto& w : waypoints)
    {
      fog_msgs::msg::Vector4Stamped v;
      v.x = w.x();
      v.y = w.y();
      v.z = w.z();
      v.w = w.w();
      v.header.frame_id = parent_frame_;
      msg.poses.push_back(v);
    }
    future_trajectory_publisher_->publish(msg);
  }
  //}

  /* waypointFutureCallback //{ */
  bool Navigation::waypointFutureCallback(rclcpp::Client<fog_msgs::srv::WaypointToLocal>::SharedFuture future)
  {
    std::shared_ptr<fog_msgs::srv::WaypointToLocal_Response> result = future.get();

    if (result->success)
    {
      RCLCPP_INFO(get_logger(), "Coordinate transform returned: %.2f, %.2f", result->local_x, result->local_y);

      const vec4_t point(result->local_x, result->local_y, result->local_z, result->yaw);

      if (point.z() < min_altitude_)
      {
        RCLCPP_WARN(get_logger(), "Goal Z coordinate (%.2f) is below the minimum allowed altitude (%.2f)", point.z(),
                    min_altitude_);
        return false;
      }

      if (point.z() > max_altitude_)
      {
        RCLCPP_WARN(get_logger(), "Goal Z coordinate (%.2f) is above the maximum allowed altitude (%.2f)", point.z(),
                    max_altitude_);
        return false;
      }

      if ((point.head<3>() - desired_pose_.head<3>()).norm() > max_goal_distance_)
      {
        RCLCPP_WARN(get_logger(), "Distance to goal (%.2f) exceeds the maximum allowed distance (%.2f m)",
                    (point.head<3>() - desired_pose_.head<3>()).norm(), max_goal_distance_);
        return false;
      }

      waypoints_in_.push_back(point);

      RCLCPP_INFO(get_logger(), "Waypoint added (LOCAL): %.2f, %.2f, %.2f, %.2f", point.x(), point.y(), point.z(), point.w());

      RCLCPP_INFO(get_logger(), "nav_state_t::planning started");
      state_ = nav_state_t::planning;

      return true;
    }
    return false;
  }
  //}

  /* pathFutureCallback //{ */
  bool Navigation::pathFutureCallback(rclcpp::Client<fog_msgs::srv::PathToLocal>::SharedFuture future)
  {
    std::shared_ptr<fog_msgs::srv::PathToLocal_Response> result = future.get();

    if (result->success)
    {
      RCLCPP_INFO(get_logger(), "Coordinate transform returned %ld points", result->path.poses.size());

      for (const auto& pose : result->path.poses)
      {
        const vec4_t point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, getYaw(pose.pose.orientation));

        if (point.z() < min_altitude_)
        {
          RCLCPP_WARN(get_logger(), "Goal Z coordinate (%.2f) is below the minimum allowed altitude (%.2f)", point.z(),
                      min_altitude_);
          return false;
        }

        if (point.z() > max_altitude_)
        {
          RCLCPP_WARN(get_logger(), "Goal Z coordinate (%.2f) is above the maximum allowed altitude (%.2f)", point.z(),
                      max_altitude_);
          return false;
        }

        if ((point.head<3>() - desired_pose_.head<3>()).norm() > max_goal_distance_)
        {
          RCLCPP_WARN(get_logger(), "Distance to goal (%.2f) exceeds the maximum allowed distance (%.2f m)",
                      (point.head<3>() - desired_pose_.head<3>()).norm(), max_goal_distance_);
          return false;
        }

        waypoints_in_.push_back(point);
        RCLCPP_INFO(get_logger(), "Waypoint added (LOCAL): %.2f, %.2f, %.2f, %.2f", point.x(), point.y(), point.z(), point.w());
      }

      RCLCPP_INFO(get_logger(), "nav_state_t::planning started");
      state_ = nav_state_t::planning;

      return true;
    }
    return false;
  }
  //}

  /* visualization //{ */

  /* visualizeTree //{ */
  void Navigation::visualizeTree(const octomap::OcTree& tree)
  {
    RCLCPP_INFO_ONCE(get_logger(), "Visualizing tree");
    visualization_msgs::msg::Marker msg;
    msg.header.frame_id = parent_frame_;
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
    RCLCPP_INFO_ONCE(get_logger(), "Visualizing open nav_state_t::planning expansions");
    visualization_msgs::msg::Marker msg;
    msg.header.frame_id = parent_frame_;
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
    msg.header.frame_id = parent_frame_;
    msg.header.stamp = get_clock()->now();
    msg.ns = "path";
    msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    msg.id = 8;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = path_points_scale_;

    std::vector<vec4_t> tmp_waypoints;
    tmp_waypoints.push_back(uav_pos_);
    for (auto& w : waypoints)
    {
      tmp_waypoints.push_back(w);
    }

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
    msg.header.frame_id = parent_frame_;
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

      if (id == current_waypoint_id_)
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

  /* parse_param //{ */
  template <class T>
  bool Navigation::parse_param(const std::string& param_name, T& param_dest)
  {
    declare_parameter<T>(param_name);
    if (!get_parameter(param_name, param_dest))
    {
      RCLCPP_ERROR(get_logger(), "Could not load param '%s'", param_name.c_str());
      return false;
    } else
    {
      RCLCPP_INFO_STREAM(get_logger(), "Loaded '" << param_name << "' = '" << param_dest << "'");
    }
    return true;
  }
  //}
  
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
  vec3_t to_eigen(const octomath::Vector3& vec)
  {
    return {vec.x(), vec.y(), vec.z()};
  }
  
  vec4_t to_eigen(const octomath::Vector3& vec, const double yaw)
  {
    return {vec.x(), vec.y(), vec.z(), yaw};
  }
  //}

  std::string Navigation::to_string(const nav_state_t state) const
  {
    switch (state)
    {
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

}  // namespace navigation
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(navigation::Navigation)

