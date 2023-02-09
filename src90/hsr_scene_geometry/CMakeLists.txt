cmake_minimum_required(VERSION 3.8)
project(hsr_scene_geometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)



add_executable(hsr_scene_geometry src/hsr_scene_geometry.cpp)
target_include_directories(hsr_scene_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hsr_scene_geometry PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(
  hsr_scene_geometry
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)
install(TARGETS hsr_scene_geometry
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)


ament_package()
