cmake_minimum_required(VERSION 3.8)
project(hsr_capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(d405_capture src/d405_capture.cpp)
target_include_directories(d405_capture PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(d405_capture PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17

install(TARGETS d405_capture
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_target_dependencies(
  d405_capture
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)
ament_package()
