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

    sub_depth_cam_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/depth/CameraInfo", 1, std::bind(&d405_capture::depth_cam_info_callback, this, _1));
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
        while (this->get_parameter("save_imgs").as_int() == 1 && this->get_parameter("count").as_int() == 4)
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

  void depth_cam_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // Create txt file for camera info
    char depth_buffer[200];
    std::ofstream depth_cam_info;
    sprintf(depth_buffer, "%sdepth/CameraInfo.txt", (this->get_parameter("data_dir")).as_string().c_str());
    depth_cam_info.open(depth_buffer);
    depth_cam_info.write((char*)&msg,sizeof(msg));
    // depth_cam_info << msg->header.as_string().c_str() << "\n" 
    // << msg->height << "\n"
    // << msg->width << "\n"
    // << msg->distortion_model.as_string().c_str() << "\n"
    // << msg->D << "\n"
    // << msg->K << "\n"
    // << msg->R << "\n"
    // << msg->P << "\n"
    // << msg->binning_x << "\n"
    // << msg->binning_y << "\n";
    // Close txt file
    depth_cam_info.close();
  }
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_depth_cam_info;
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

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "iiwa");

  // Set speeds
  move_group_interface.setMaxVelocityScalingFactor(0.25);
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
        while (node->get_parameter("count").as_int() != 4)
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

  
  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}