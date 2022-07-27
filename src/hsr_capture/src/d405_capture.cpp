#include <stdio.h>
#include <iostream>
#include <fstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <math.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "image_transport/image_transport.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <Eigen/Dense>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <realsense2_camera_msgs/msg/extrinsics.hpp>
using std::placeholders::_1;
using namespace cv;
#define PI 3.141592653589793238462643383279502884L /* pi */

class d405_capture : public rclcpp::Node
{
public:
  d405_capture() : Node("d405_capture")
  {
    this->declare_parameter<std::string>("data_dir", "/home/kukasrv/data/dataset1/");
    this->declare_parameter<int>("save_imgs", 0);
    this->declare_parameter<int>("acq_num", 0);
    this->declare_parameter<int>("count", 0);

    sub_infra1 = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/infra1/image_rect_raw", 1, std::bind(&d405_capture::infra1_rect_raw_callback, this, _1));

    sub_infra2 = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/infra2/image_rect_raw", 1, std::bind(&d405_capture::infra2_rect_raw_callback, this, _1));

    sub_rgb = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_rect_raw", 1, std::bind(&d405_capture::rgb_callback, this, _1));

    sub_depth = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth/image_rect_raw", 1, std::bind(&d405_capture::depth_callback, this, _1));

    sub_aligned_depth = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/aligned_depth_to_color/image_raw", 1, std::bind(&d405_capture::aligned_depth_callback, this, _1));

    sub_depth_cam_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/depth/camera_info", 1, std::bind(&d405_capture::depth_cam_info_callback, this, _1));

    sub_aligned_depth_cam_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/aligned_depth_to_color/camera_info", 1, std::bind(&d405_capture::aligned_depth_cam_info_callback, this, _1));

    sub_rgb_cam_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/color/camera_info", 1, std::bind(&d405_capture::rgb_cam_info_callback, this, _1));

    sub_left_cam_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/infra1/camera_info", 1, std::bind(&d405_capture::left_cam_info_callback, this, _1));

    sub_right_cam_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/infra2/camera_info", 1, std::bind(&d405_capture::right_cam_info_callback, this, _1));

    sub_extr_depth_to_colour = this->create_subscription<realsense2_camera_msgs::msg::Extrinsics>(
        "/camera/extrinsics/depth_to_color", 1, std::bind(&d405_capture::extr_depth_to_colour , this, _1));

    sub_extr_depth_to_depth = this->create_subscription<realsense2_camera_msgs::msg::Extrinsics>(
        "/camera/extrinsics/depth_to_depth", 1, std::bind(&d405_capture::extr_depth_to_depth , this, _1));

    sub_extr_depth_to_infra1 = this->create_subscription<realsense2_camera_msgs::msg::Extrinsics>(
        "/camera/extrinsics/depth_to_infra1", 1, std::bind(&d405_capture::extr_depth_to_infra1 , this, _1));
        
    sub_extr_depth_to_infra2 = this->create_subscription<realsense2_camera_msgs::msg::Extrinsics>(
        "/camera/extrinsics/depth_to_infra2", 1, std::bind(&d405_capture::extr_depth_to_infra2 , this, _1));
  }

  void img_cap(std::string file_path, int acq_num, sensor_msgs::msg::Image::SharedPtr msg)
  {

    if (this->get_parameter("save_imgs").as_int() == 1)
    {
      try
      {
        cv_bridge::CvImagePtr cvptr;
        std::string rgb = "rgb/";
        if (file_path == rgb)
        {
          cvptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        else
        {
          cvptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        // std::string name = (this->get_parameter("data_dir")).as_string() + file_path + std::to_string(acq_num) + ".png";
        char buffer[200];
        sprintf(buffer, "%s%s%04d.png", (this->get_parameter("data_dir")).as_string().c_str(), file_path.c_str(), acq_num);
        imwrite(buffer, cvptr->image);
        this->set_parameter(rclcpp::Parameter("count", this->get_parameter("count").as_int() + 1));
        while (this->get_parameter("save_imgs").as_int() == 1 && this->get_parameter("count").as_int() == 5)
        {
        }
      }
      catch (cv_bridge::Exception &e)
      {
        auto logger = rclcpp::get_logger("d405_capture");
        RCLCPP_ERROR(logger, "Could not convert from '%s'.", msg->encoding.c_str());
      }
    }
  }

  void save_cam_info(sensor_msgs::msg::CameraInfo::SharedPtr msg, char *file_path)
  {
    std::ofstream cam_info;
    cam_info.open(file_path);
    cam_info << "Timestamp:," << msg->header.stamp.sec <<" ." << msg->header.stamp.nanosec << "\n"
             << "Frame ID:," << msg->header.frame_id << "\n"
             << "Height:," << msg->height << "\n"
             << "Width:," << msg->width << "\n"
             << "Distortion model:," << msg->distortion_model << "\n"
             << "D:,";
    for (auto i : msg->d)
      cam_info << i << ",";
    cam_info << "\n"
             << "K:,";
    for (auto i : msg->k)
      cam_info << i << ",";
    cam_info << "\n"
             << "R:,";
    for (auto i : msg->r)
      cam_info << i << ",";
    cam_info << "\n"
             << "P:,";
    for (auto i : msg->p)
      cam_info << i << ",";
    cam_info << "\n" <<"Bin X:," << msg->binning_x << "\n"
             << "Bin Y:," << msg->binning_y << "\n";
    // // Close txt file
    cam_info.close();
  }

  void save_cam_extr(realsense2_camera_msgs::msg::Extrinsics::SharedPtr msg, char *file_path)
  {
    std::ofstream cam_info;
    cam_info.open(file_path,std::ios::app);
    cam_info << "Rot Mat:,";
    for (auto i : msg->rotation)
    {
      cam_info << i << ",";
    }
      cam_info << "\n" << "Translation:,";
    for (auto i : msg->translation)
      {
      cam_info << i << ",";
      }
    cam_info << "\n";
    // // Close txt file
    cam_info.close();
  }

  void savePose(std::ofstream &pose, moveit::planning_interface::MoveGroupInterface &move_group_interface, const std::string &end_effector_link)
  {
    const std::string &eef = end_effector_link.empty() ? move_group_interface.getEndEffectorLink() : end_effector_link;
    geometry_msgs::msg::PoseStamped msg;
    std::vector<double> rpy_msg;
    msg = move_group_interface.getCurrentPose(eef);
    rpy_msg = move_group_interface.getCurrentRPY(eef);
    char buffer[25];
    sprintf(buffer, "%04ld", (this->get_parameter("acq_num")).as_int());
    pose
        << buffer << ","
        << msg.header.stamp.sec << "." << msg.header.stamp.nanosec << ","
        << msg.header.frame_id << ","
        << msg.pose.position.x << ","
        << msg.pose.position.y << ","
        << msg.pose.position.z << ","
        << msg.pose.orientation.x << ","
        << msg.pose.orientation.y << ","
        << msg.pose.orientation.z << ","
        << msg.pose.orientation.w << ","
        << rpy_msg[0] << ","
        << rpy_msg[1] << ","
        << rpy_msg[2]
        << "\n";
  }

private:
  void infra1_rect_raw_callback(sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Left image received from Realsense\tSize: %dx%d - Timestamp: %u.%u sec ",
                msg->width, msg->height,
                msg->header.stamp.sec, msg->header.stamp.nanosec);
    std::string file_path = "left_rect_raw/";
    int acq_num = (this->get_parameter("acq_num")).as_int();
    img_cap(file_path, acq_num, msg);
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_infra1;

  void infra2_rect_raw_callback(sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Right image received from Realsense\tSize: %dx%d - Timestamp: %u.%u sec ",
                msg->width, msg->height,
                msg->header.stamp.sec, msg->header.stamp.nanosec);
    std::string file_path = "right_rect_raw/";
    int acq_num = (this->get_parameter("acq_num")).as_int();
    img_cap(file_path, acq_num, msg);
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_infra2;

  void rgb_callback(sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Colour image received from Realsense\tSize: %dx%d - Timestamp: %u.%u sec ",
                msg->width, msg->height,
                msg->header.stamp.sec, msg->header.stamp.nanosec);
    std::string file_path = "rgb/";
    int acq_num = (this->get_parameter("acq_num")).as_int();
    img_cap(file_path, acq_num, msg);
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rgb;

  void depth_callback(sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Depth map received from Realsense\tSize: %dx%d - Timestamp: %u.%u sec ",
                msg->width, msg->height,
                msg->header.stamp.sec, msg->header.stamp.nanosec);
    std::string file_path = "depth/";
    int acq_num = (this->get_parameter("acq_num")).as_int();
    img_cap(file_path, acq_num, msg);
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth;

  void aligned_depth_callback(sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Aligned Depth map received from Realsense\tSize: %dx%d - Timestamp: %u.%u sec ",
                msg->width, msg->height,
                msg->header.stamp.sec, msg->header.stamp.nanosec);
    std::string file_path = "aligned_depth/";
    int acq_num = (this->get_parameter("acq_num")).as_int();
    img_cap(file_path, acq_num, msg);
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_aligned_depth;

  void aligned_depth_cam_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // Create txt file for camera info
    char depth_buffer[250];
    sprintf(depth_buffer, "%saligned_depth/CameraInfo.csv", (this->get_parameter("data_dir")).as_string().c_str());
    save_cam_info(msg, depth_buffer);
  }
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_aligned_depth_cam_info;

  void depth_cam_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // Create txt file for camera info
    char depth_buffer[250];
    sprintf(depth_buffer, "%sdepth/CameraInfo.csv", (this->get_parameter("data_dir")).as_string().c_str());
    save_cam_info(msg, depth_buffer);
  }
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_depth_cam_info;

  void rgb_cam_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // Create txt file for camera info
    char rgb_buffer[250];
    sprintf(rgb_buffer, "%srgb/CameraInfo.csv", (this->get_parameter("data_dir")).as_string().c_str());
    save_cam_info(msg, rgb_buffer);
  }
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_rgb_cam_info;

  void left_cam_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // Create txt file for camera info
    char left_buffer[250];
    sprintf(left_buffer, "%sleft_rect_raw/CameraInfo.csv", (this->get_parameter("data_dir")).as_string().c_str());
    save_cam_info(msg, left_buffer);
  }
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_left_cam_info;

  void right_cam_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // Create txt file for camera info
    char right_buffer[250];
    sprintf(right_buffer, "%sright_rect_raw/CameraInfo.csv", (this->get_parameter("data_dir")).as_string().c_str());
    save_cam_info(msg, right_buffer);
  }
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_right_cam_info;

  void extr_depth_to_colour(realsense2_camera_msgs::msg::Extrinsics::SharedPtr msg)
  {
// Create txt file for camera extrinsics
    char buffer[250];
    sprintf(buffer, "%sextrinsics/extr_depth_to_color.csv", (this->get_parameter("data_dir")).as_string().c_str());
    save_cam_extr(msg, buffer);
  }
  rclcpp::Subscription<realsense2_camera_msgs::msg::Extrinsics>::SharedPtr sub_extr_depth_to_colour;

  void extr_depth_to_depth(realsense2_camera_msgs::msg::Extrinsics::SharedPtr msg)
  {
// Create txt file for camera extrinsics
    char buffer[250];
    sprintf(buffer, "%sextrinsics/extr_depth_to_depth.csv", (this->get_parameter("data_dir")).as_string().c_str());
    save_cam_extr(msg, buffer);
  }
  rclcpp::Subscription<realsense2_camera_msgs::msg::Extrinsics>::SharedPtr sub_extr_depth_to_depth;

  void extr_depth_to_infra1(realsense2_camera_msgs::msg::Extrinsics::SharedPtr msg)
  {
// Create txt file for camera extrinsics
    char buffer[250];
    sprintf(buffer, "%sextrinsics/extr_depth_to_infra1.csv", (this->get_parameter("data_dir")).as_string().c_str());
    save_cam_extr(msg, buffer);
  }
  rclcpp::Subscription<realsense2_camera_msgs::msg::Extrinsics>::SharedPtr sub_extr_depth_to_infra1;

  void extr_depth_to_infra2(realsense2_camera_msgs::msg::Extrinsics::SharedPtr msg)
  {
// Create txt file for camera extrinsics
    char buffer[250];
    sprintf(buffer, "%sextrinsics/extr_depth_to_infra2.csv", (this->get_parameter("data_dir")).as_string().c_str());
    save_cam_extr(msg, buffer);
  }
  rclcpp::Subscription<realsense2_camera_msgs::msg::Extrinsics>::SharedPtr sub_extr_depth_to_infra2;
};

int main(int argc, char *argv[])
{

  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<d405_capture>();
  // int capture = 0;
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("d405_capture");

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]()
                             { executor.spin(); });

  // Open csv file to store Poses
  char pose_buffer[250];
  sprintf(pose_buffer, "%sposes/ee_poses.csv", (node->get_parameter("data_dir")).as_string().c_str());
  std::ofstream ee_poses;
  ee_poses.open(pose_buffer);
  ee_poses << "Image #,"
        << "Time,"
        << "Frame ID,"
        << "X,"
        << "Y,"
        << "Z,"
        << "Q_x,"
        << "Q_y,"
        << "Q_z,"
        << "Q_w,"
        << "Roll,"
        << "Pitch,"
        << "Yaw,"
        << "\n";
  std::string ee = "iiwa_link_ee";

  
  sprintf(pose_buffer, "%sposes/color_optical_frame_poses.csv", (node->get_parameter("data_dir")).as_string().c_str());
  std::ofstream color_poses;
  color_poses.open(pose_buffer);
  color_poses << "Image #,"
        << "Time,"
        << "Frame ID,"
        << "X,"
        << "Y,"
        << "Z,"
        << "Q_x,"
        << "Q_y,"
        << "Q_z,"
        << "Q_w,"
        << "Roll,"
        << "Pitch,"
        << "Yaw,"
        << "\n";
  std::string color_frame = "camera_color_optical_frame";

  
  sprintf(pose_buffer, "%sposes/depth_optical_frame_poses.csv", (node->get_parameter("data_dir")).as_string().c_str());
  std::ofstream depth_poses;
  depth_poses.open(pose_buffer);
  depth_poses << "Image #,"
        << "Time,"
        << "Frame ID,"
        << "X,"
        << "Y,"
        << "Z,"
        << "Q_x,"
        << "Q_y,"
        << "Q_z,"
        << "Q_w,"
        << "Roll,"
        << "Pitch,"
        << "Yaw,"
        << "\n";
  std::string depth_frame = "camera_depth_optical_frame";

  
  sprintf(pose_buffer, "%sposes/infra1_optical_frame_poses.csv", (node->get_parameter("data_dir")).as_string().c_str());
  std::ofstream infra1_poses;
  infra1_poses.open(pose_buffer);
  infra1_poses << "Image #,"
        << "Time,"
        << "Frame ID,"
        << "X,"
        << "Y,"
        << "Z,"
        << "Q_x,"
        << "Q_y,"
        << "Q_z,"
        << "Q_w,"
        << "Roll,"
        << "Pitch,"
        << "Yaw,"
        << "\n";
  std::string infra1_frame = "camera_infra1_optical_frame";

  
 
  
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "iiwa");

  // Set speeds
  move_group_interface.setMaxVelocityScalingFactor(0.75);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  move_group_interface.setPlanningTime(5.0);

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closure for updating the text in rviz
  auto const draw_title = [&moveit_visual_tools](auto text)
  {
    auto const text_pose = []
    {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };

  auto const prompt = [&moveit_visual_tools](auto text)
  { moveit_visual_tools.prompt(text); };

  const int N = 8;
  const double polar_range = 0.7 * PI / 2;
  const double rad = 0.15;
  double azimuth[N];
  double polar[N];
  int s = 0;
  int f = 0;

  for (int i = 0; i < N; i++)
  {
    azimuth[i] = (N - 1 - i) * 2 * PI / N;
    polar[i] = (N - 1 - i) * polar_range / N;
  }

  // Define sample point
  Eigen::Vector3d s_pos(0.4125, 0.6125, 0.06);
  auto const sample_collision_object = [frame_id = move_group_interface.getPlanningFrame(), s_pos, rad]
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "sample";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.SPHERE;
    primitive.dimensions.resize(1);
    primitive.dimensions[primitive.SPHERE_RADIUS] = 0.9 * rad;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = s_pos[0];
    box_pose.position.y = s_pos[1];
    box_pose.position.z = s_pos[2];

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(sample_collision_object);

  for (int i = 0; i < N; i++)
  {
    for (int j = 0; j < N; j++)
    {
      node->set_parameter(rclcpp::Parameter("acq_num", N * i + j));
      node->set_parameter(rclcpp::Parameter("count", 0));
      node->set_parameter(rclcpp::Parameter("save_imgs", 0));
      // Set a target Pose
      auto target_pose = [i, j, s_pos, azimuth, polar, rad]
      {
        geometry_msgs::msg::Pose msg;
        tf2::Quaternion q;

        // Define Camera Position
        Eigen::Vector3d c_pos(s_pos[0] + rad * cos(azimuth[j]) * sin(polar[i]),
                              s_pos[1] + rad * sin(azimuth[j]) * sin(polar[i]),
                              s_pos[2] + rad * cos(polar[i]));

        msg.position.x = c_pos[0];
        msg.position.y = c_pos[1];
        msg.position.z = c_pos[2];
        Eigen::Vector3d v = c_pos - s_pos;

        if (v[0] == 0 && v[1] == 0)
        {
          q.setRPY(PI, 0.0, azimuth[j]);
        }
        else if (v[0] <= 0)
        {
          q.setRPY(PI - atan(sqrt(v[0] * v[0] + v[1] * v[1]) / v[2]), 0.0, acos(v[1] / (sqrt(v[0] * v[0] + v[1] * v[1]))));
        }
        else
        {
          q.setRPY(PI - atan(sqrt(v[0] * v[0] + v[1] * v[1]) / v[2]), 0.0, -acos(v[1] / (sqrt(v[0] * v[0] + v[1] * v[1]))));
        }
        q.normalize();

        msg.orientation.x = q[0];
        msg.orientation.y = q[1];
        msg.orientation.z = q[2];
        msg.orientation.w = q[3];
        return msg;
      }();
      move_group_interface.setPoseTarget(target_pose);

      // Create a plan to that target pose
      prompt("Press 'next' in the RvizVisualToolsGui window to plan");
      draw_title("Planning");
      moveit_visual_tools.trigger();
      auto [success, plan] = [&move_group_interface]
      {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
      }();

      // Execute the plan
      if (success)
      {
        moveit_visual_tools.trigger();
        prompt("Press 'next' in the RvizVisualToolsGui window to execute");
        draw_title("Executing");
        moveit_visual_tools.trigger();
        move_group_interface.execute(plan);
        auto markers = Eigen::Isometry3d::Identity();
        markers.translation().x() = (s_pos[0] + rad * cos(azimuth[j]) * sin(polar[i]));
        markers.translation().y() = (s_pos[1] + rad * sin(azimuth[j]) * sin(polar[i]));
        markers.translation().z() = (s_pos[2] + rad * cos(polar[i]));
        moveit_visual_tools.publishSphere(markers, rviz_visual_tools::GREEN);
        prompt("Press 'next' in the RvizVisualToolsGui window to capture data");
        moveit_visual_tools.trigger();
        node->set_parameter(rclcpp::Parameter("save_imgs", 1));
        node->savePose(ee_poses, move_group_interface, ee);
        node->savePose(color_poses, move_group_interface, color_frame);
        node->savePose(depth_poses, move_group_interface, depth_frame);
        node->savePose(infra1_poses, move_group_interface, infra1_frame);
        while (node->get_parameter("count").as_int() != 5)
        {
        }
        s++;
      }
      else
      {
        draw_title("Planning Failed!");
        moveit_visual_tools.trigger();
        RCLCPP_ERROR(logger, "Planning failed!");
        auto markers = Eigen::Isometry3d::Identity();
        markers.translation().x() = (s_pos[0] + rad * cos(azimuth[j]) * sin(polar[i]));
        markers.translation().y() = (s_pos[1] + rad * sin(azimuth[j]) * sin(polar[i]));
        markers.translation().z() = (s_pos[2] + rad * cos(polar[i]));
        moveit_visual_tools.publishSphere(markers, rviz_visual_tools::RED);
        f++;
      }
      RCLCPP_INFO(logger, "There have been %d successes so far and %d failures.", s, f);
    }
  }

  // Close pose csv file
  ee_poses.close();
  color_poses.close();
depth_poses.close();
infra1_poses.close();
  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}