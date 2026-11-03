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

find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common visualization)
# Eigenの設定がPCL経由で裸のコンパイル引数になる環境でも、マクロとして渡す。
remove_definitions(EIGEN_MPL2_ONLY)
add_compile_definitions(EIGEN_MPL2_ONLY)
find_package(Threads REQUIRED)
# 外部ソースは変更せず、ライブラリ部分をこのターゲットでビルドする。
set(LINEFIT_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../deps/linefit/linefit_ground_segmentation"
  CACHE PATH "Linefit library source directory")
if(NOT EXISTS "${LINEFIT_SOURCE_DIR}/src/ground_segmentation.cc")
  message(FATAL_ERROR "Fetch Linefit first: cd ws && vcs import deps < linefit.repos")
endif()
add_library(simulation_linefit STATIC
  ${LINEFIT_SOURCE_DIR}/src/bin.cc
  ${LINEFIT_SOURCE_DIR}/src/segment.cc
  ${LINEFIT_SOURCE_DIR}/src/ground_segmentation.cc
  ${LINEFIT_SOURCE_DIR}/src/viewer.cc)
target_include_directories(simulation_linefit SYSTEM PUBLIC
  ${LINEFIT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS})
target_link_libraries(simulation_linefit PUBLIC ${PCL_LIBRARIES} Threads::Threads)
target_compile_features(simulation_linefit PUBLIC cxx_std_17)
target_compile_options(simulation_linefit PRIVATE -O3)
add_executable(linefit_ground_filter src/linefit_ground_filter.cpp)
ament_target_dependencies(linefit_ground_filter
  rclcpp sensor_msgs tf2_ros pcl_conversions)
target_include_directories(linefit_ground_filter PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(linefit_ground_filter simulation_linefit)
target_compile_features(linefit_ground_filter PUBLIC cxx_std_17)
install(TARGETS linefit_ground_filter DESTINATION lib/${PROJECT_NAME})

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
