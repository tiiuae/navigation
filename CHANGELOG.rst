^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
