cmake_minimum_required(VERSION 3.8)
project(ee4308_turtle)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options(-Wno-error=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(ee4308_core REQUIRED)
# find_package(ee4308_interfaces)

nav2_package()

set (DEPENDENCIES
  rclcpp
  rclcpp_lifecycle
  rcutils
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  nav2_util
  nav2_core
  nav2_costmap_2d
  pluginlib
  Eigen3
  std_msgs
  visualization_msgs
  # ee4308_core
)

# Controller Nav2 Plugin
add_library(ee4308_turtle_controller SHARED 
  src/controller.cpp
  )
target_include_directories(ee4308_turtle_controller PUBLIC include)
target_compile_definitions(ee4308_turtle_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS") # prevent pluginlib from using boost
target_compile_features(ee4308_turtle_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(ee4308_turtle_controller ${DEPENDENCIES})
install(TARGETS ee4308_turtle_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
pluginlib_export_plugin_description_file(nav2_core ee4308_turtle_controller.xml)

# Planner Nav2 Plugin
add_library(ee4308_turtle_planner SHARED 
  src/planner.cpp
  )
target_include_directories(ee4308_turtle_planner PUBLIC include)
target_compile_definitions(ee4308_turtle_planner PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS") # prevent pluginlib from using boost
target_compile_features(ee4308_turtle_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(ee4308_turtle_planner ${DEPENDENCIES})
install(TARGETS ee4308_turtle_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
pluginlib_export_plugin_description_file(nav2_core ee4308_turtle_planner.xml)


install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_package()
