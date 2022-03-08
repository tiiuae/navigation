^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2022-03-03)
-----------
* better printing of waypoints
* added gps_waypoint service into action_client_node
* added cancel request into action_client_node
* removed checking for too short path
* waypoints are now cleared correctly after mission is finished
* goal feedback will now be published periodically
* [action_server]: working version, tested in simulations
* [action_server]: barebones version working
* [action_server]: wip
* rewrote to use action server of control interface instead of services
* added skeleton for control_interface action_client
* Fixed install of include folder
* minor update to reflect new mission states in control interface
* update for the stopped mission state from control interface
* removed unnecesary declarations of function
* refactored the diagnostics message to use enums
* in case a waypoint/path is rejected, the reason will now be correctly returned and printed
* perform a vertical bounce if path begins outside octomap
* updated new fog_msgs/srv/Vec4 type handling
* removed unnecessary installing of folder in Cmakelist
* update to new mission_progress msg
* when ending avoiding manoeuvre, the state will switch directly to planning if there are any waypoints left (to idle else)
* state will now switch to not_ready when bumper data is missing and bumper is enabled
* fixed crash when no bumper message is received before switching to idle, added some more checks
* fixed conflicts with mission-engine -> make status to be in UPPERCASE letters
* Contributors: Matouš Vrba, Vojtech Spurny, stibipet

0.1.0 (2022-02-02)
-----------
* prevent temporary goal generation from skipping octree frontiers
* minor refactor, astar planner will now log to console
* diagnostics timer now has a separate rate
* dedicated timer for diagnostics publishing
* moved some common functions to fog_lib
* renamed yaw to heading, fixed heading issues
* Merge branch 'matous' of github.com:fly4future/navigation into matous
* wip fixing yaw->heading
* removed continuous yaw change - yaw will now be set to the desired value ASAP
* states are now publised uppercase in the diagnostics
* carefully rewrote mutex locking order to avoid deadlocks
* removed potential deadlock
* added check for correct size of Vec4 request
* added bumper min replan period, minor refactoring
* add ID to control requests and responses
* fixed bug with missing waypoint increment when mission is finished
* fixed control_interface dependency
* when avoiding, navigation will only send one service request at a time and wait for it to finish before sending a new one
* control interface state enums will now be decoded from ROS messages using the functions defined in control_interface
* added fog_lib dependency and moved some code there
* removed redundant NED to ENU conversion of cmd_pose and fixed debug mode
* minor fixes and refactoring, cmd pose is now taken from control_interface instead of pixhawk messages
* minor refactor and cleanup
* fixes with mutexes
* changed callback groups to mut.ex., fix for foxy parameter loading
* working version, refactoring, fixes
* wip rewriting control interface diags message
* refactoring and bug fixing wip
* started some refactoring and fixes
* Remove status topic remapping, conflicts mission-engine
* Add galactic fix, update invoke tasks
  Ability to pass commit hash to packaging and build scripts
  Move git rev-parse commands to build.sh from invoke tasks
  Add tagging to docker images
* Galactic support for main.yaml workflow
* Add containerized build workflow
* New build scripts with container build
* fix debug print
* Resolve conflict
* New build scripts, remove fogsw-ci-scripts
* fix yaw step printed as double

0.0.8 (2021-11-30)
-----------
* update package versions
* add current waypoint status
* remove commented code templates
* renamed odometry pkg name
* Merge branch 'odometry2' of github.com:tiiuae/navigation into odometry2
* more code cleanup, move visualization topics under namespace
* Merge branch 'master' into odometry2
* remove unused 'field' publsiher
* launchfile cleanup
* code cleanup
* make path sampling less dense
* Merge pull request `#14 <https://github.com/tiiuae/navigation/issues/14>`_ from tiiuae/fix_nav_regression
  Fix regression happened in bumper PR
* Fix regression happened in last PR
* fix merge typo
* update mutex in avoidance
* plan from the last desired position, but only if its nearby
* do not allow planning before takeoff finishes
* allow yaw change without planning
* erase waypoint buffers if control is manually taken over
* check if under manual control
* update path visualization
* update bumper avoidance behavior
* connect to odometry, check desired pose presence
* Merge pull request `#12 <https://github.com/tiiuae/navigation/issues/12>`_ from tiiuae/bumper
  Virtual bumper

0.0.6 (2021-09-29)
-----------
* Require fog_msgs 0.0.6
* add distance factor for bumper activation
* integrate bumper behavior into navigation state machine
* subscribe to desired pose, in addition to estimated pose
* control odom -> odom odom
* again improved current_waypoint_id, added bumper and octomap subscriberds to CallbackGroupType::Reentrant
* fixed current_waypoint_id
* added bumper
* cleanup of dependencies
* fix typo, add references into some methods, and added current_waypoint_id
* add main routine rate as parameter in config file, all config parameters are now non-optinal
* parse_param update and cleanup in config file
* do not filter path when escaping from no-go zone
* version -> 0.0.5, add diagnostics
* better resampling, path generation, added yaw
* update planner
* fix getting stuck when goal is inside obstacle
* update repeated planning behavior
* planning speedup
* include edf into cost
* initialize end waypoint variable
* Merge pull request `#11 <https://github.com/tiiuae/navigation/issues/11>`_ from tiiuae/fix_debug_interface_on_systemd
  Fix for launch done from systemd service
* Fix for launch done from systemd service
* do not filter path when escaping from no-go zone

0.0.5 (2021-09-29)
-----------
* version -> 0.0.5, add diagnostics
* better resampling, path generation, added yaw
* update planner
* fix getting stuck when goal is inside obstacle
* update repeated planning behavior
* planning speedup
* include edf into cost
* Merge pull request `#9 <https://github.com/tiiuae/navigation/issues/9>`_ from tiiuae/increase_safety_distance
  Increase safe obstacle distance to 1.5m
* Increase safe obstacle distance to 1.5m
  Safe obstacle distance is calculated from the center of the drone. This
  is important to keep in mind when setting safe obstacle distance.
* Merge pull request `#8 <https://github.com/tiiuae/navigation/issues/8>`_ from tiiuae/trigger_fog-drone_build
  trigger fog-drone build
* trigger fog-drone build
* Merge pull request `#7 <https://github.com/tiiuae/navigation/issues/7>`_ from tiiuae/remove_dispatch_event
  remove repository dispatch events
* remove repository dispatch events
  Trigger builds only when repository is updated. Use git sha as build id
  for Artifactory builds.
* Merge pull request `#6 <https://github.com/tiiuae/navigation/issues/6>`_ from tiiuae/smaller_obstacle_safe_distance
  Reduce safe obsatcle distance
* Reduce safe obsatcle distance
  Reason is to have more possible paths available in indoor testing.
* Contributors: Esa Kulmala, Jari Nippula, Manuel Segarra-Abad, Vojtech Spurny, stibipet


0.0.3 (2021-06-21)
-----------
* Global coordinates navigation update (`#5 <https://github.com/tiiuae/navigation/issues/5>`_)
  * add global coordinate input interface
  * add include dirs to cmakelists
* fix github action build_id
* Merge pull request `#4 <https://github.com/tiiuae/navigation/issues/4>`_ from tiiuae/repository_dispatch
  * CI: add rebuild repository dispatch event
* add rebuild repository dispatch event
* Merge pull request `#3 <https://github.com/tiiuae/navigation/issues/3>`_ from tiiuae/fix-pkg-name
  * pkg_name to navigation
* Merge pull request `#2 <https://github.com/tiiuae/navigation/issues/2>`_ from tiiuae/integration_fixes
  * tty fix + moved launch file from fog_core
* moved launch file from fog_core
* Merge pull request `#1 <https://github.com/tiiuae/navigation/issues/1>`_ from tiiuae/DP-853_f4f_navigation_build
  * add CI workflow
* add the option to override previous commands or interrupt navigation and hover
* Contributors: Esa Kulmala, Jari Nippula, Sergey Smirnov, sergey-unikie, stibipet

0.0.2 (2021-06-02)
-----------
* subscribe to control diagnostics for mission status
* update path generation and postprocessing
* Contributors: stibipet

0.0.1 (2021-05-19)
------------------
* Add service to set path with fog_msgs::srv::Path
* Add changelog
* Update input to use nav_msgs::Path + std_srvs::Trigger
* Contributors: Vojtech Spurny, stibipet
