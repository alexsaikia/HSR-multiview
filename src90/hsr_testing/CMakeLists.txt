cmake_minimum_required(VERSION 3.8)
project(hsr_testing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)

add_executable(iiwa_first_move src/iiwa_first_move.cpp)
# target_include_directories(iiwa_first_move PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(iiwa_first_move PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(
  iiwa_first_move
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(iiwa_ompl_demo src/iiwa_ompl_demo.cpp)
# target_include_directories(iiwa_ompl_demo PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(iiwa_ompl_demo PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(
  iiwa_ompl_demo
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(iiwa_trajectory_test src/iiwa_trajectory_test.cpp)
# target_include_directories(iiwa_trajectory_test PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(iiwa_trajectory_test PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(
  iiwa_trajectory_test
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

add_executable(iiwa_avoid_box src/iiwa_avoid_box.cpp)
# target_include_directories(iiwa_avoid_box PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(iiwa_avoid_box PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(
  iiwa_avoid_box
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

add_executable(add_walls src/add_walls.cpp)
# target_include_directories(iiwa_avoid_box PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(iiwa_avoid_box PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(
  add_walls
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)


install(TARGETS
  iiwa_first_move
  iiwa_ompl_demo
  iiwa_trajectory_test
  iiwa_avoid_box
  add_walls
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
