cmake_minimum_required(VERSION 3.8)
project(simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_library(simulation_get_home_pose_bt_node SHARED src/get_home_pose.cpp)
ament_target_dependencies(simulation_get_home_pose_bt_node
  rclcpp geometry_msgs behaviortree_cpp)
target_compile_features(simulation_get_home_pose_bt_node PUBLIC cxx_std_17)
target_compile_definitions(simulation_get_home_pose_bt_node PRIVATE BT_PLUGIN_EXPORT)
install(TARGETS simulation_get_home_pose_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

endif()

# Install directories
install(DIRECTORY
  launch
  rviz
  urdf
  worlds
  config
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/nodding_io.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME nodding_io
)

install(PROGRAMS scripts/sensor_covariance_adapter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sensor_covariance_adapter
)

install(PROGRAMS scripts/pointcloud_height_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pointcloud_height_filter
)

install(PROGRAMS scripts/home_pose_saver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME home_pose_saver
)

ament_package()
