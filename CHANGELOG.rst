^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
