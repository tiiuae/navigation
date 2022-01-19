cmake_minimum_required(VERSION 3.5)
project(navigation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall")
add_definitions("-Wextra")
add_definitions("-Wpedantic")
add_definitions("-g")
add_definitions("-O3")
if ("$ENV{ROS_DISTRO}" STREQUAL "foxy") # to select the correct version of declare_parameter() and avoid errors/warnings
  add_definitions("-DROS_FOXY")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(fog_msgs 0.0.8 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(dynamicEDT3D REQUIRED)
find_package(fog_lib REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${DYNAMICEDT3D_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                       compile                              |
## --------------------------------------------------------------

#{ navigation
add_library(navigation SHARED
  src/navigation.cpp
  src/astar_planner.cpp
)

target_compile_definitions(navigation
  PRIVATE "${PROJECT_NAME}_BUILDING_DLL")

ament_target_dependencies(navigation
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  px4_msgs
  fog_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  octomap
  octomap_msgs
  dynamicEDT3D
  fog_lib
)

target_link_libraries(navigation ${OCTOMAP_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES})
rclcpp_components_register_nodes(navigation PLUGIN "${PROJECT_NAME}::Navigation" EXECUTABLE navigation)

#}

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(TARGETS
  navigation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  # DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

# install(DIRECTORY include
#   DESTINATION include
# )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
