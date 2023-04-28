cmake_minimum_required(VERSION 3.8)
project(pedsim_dataset_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pedsim_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)

set(dependencies
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "pedsim_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "rosbag2_cpp"
  "rosbag2_transport"
)

# include_directories(include)

# add_executable(pedsim_data_recorder src/pedsim_data_recorder.cpp)
# ament_target_dependencies(pedsim_data_recorder ${dependencies})

# add_executable(agent_pose_publisher src/agent_pose_publisher.cpp)
# ament_target_dependencies(agent_pose_publisher ${dependencies})

# target_include_directories(agent_pose_publisher PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(agent_pose_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install(TARGETS agent_pose_publisher agent_pose_publisher
#   DESTINATION lib/${PROJECT_NAME})

# install(TARGETS pedsim_data_recorder pedsim_data_recorder
# DESTINATION lib/${PROJECT_NAME})

  # Install launch files and directories.
install(DIRECTORY
src launch params ${PROJECT_NAME}
DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
