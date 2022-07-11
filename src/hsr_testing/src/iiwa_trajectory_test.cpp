#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <math.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <Eigen/Dense>

#define PI 3.141592653589793238462643383279502884L /* pi */

int main(int argc, char *argv[])
{

  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "iiwa_trajectory_test", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("iiwa_trajectory_test");

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
  move_group_interface.setMaxVelocityScalingFactor(0.5);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);
  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                             move_group_interface.getRobotModel()};
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
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("iiwa")](
          auto const trajectory)
  {
    moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
  };

  // auto const ref_link_ = move_group_interface.getPoseReferenceFrame();
  // auto const ee_link_ = move_group_interface.getEndEffectorLink();
  // move_group_interface.clearPoseTargets();
  // move_group_interface.clearPathConstraints();
  
  // shape_msgs::msg::SolidPrimitive cbox;
  // cbox.type = shape_msgs::msg::SolidPrimitive::BOX;
  // cbox.dimensions = {1.6, 1.3, 3.0};
  
  // geometry_msgs::msg::Pose cbox_pose;
  // cbox_pose.position.x = 0.0;
  // cbox_pose.position.y = 0.4;
  // cbox_pose.position.z = 0.5;

  // moveit_msgs::msg::Constraints path_constraints;
  // path_constraints.name = "box constraints";
  
  // moveit_msgs::msg::PositionConstraint pcm5;
  // pcm5.header.frame_id = ref_link_;
  // pcm5.link_name = "iiwa_link_5";
  // pcm5.weight = 1.0;
  // pcm5.constraint_region.primitives.emplace_back(cbox);
  // pcm5.constraint_region.primitive_poses.emplace_back(cbox_pose);
  // path_constraints.position_constraints.emplace_back(pcm5);

  // moveit_msgs::msg::PositionConstraint pcm6;
  // pcm6.header.frame_id = ref_link_;
  // pcm6.link_name = "iiwa_link_6";
  // pcm6.weight = 1.0;
  // pcm6.constraint_region.primitives.emplace_back(cbox);
  // pcm6.constraint_region.primitive_poses.emplace_back(cbox_pose);
  // path_constraints.position_constraints.emplace_back(pcm6);
  
  // moveit_msgs::msg::PositionConstraint pcm7;
  // pcm7.header.frame_id = ref_link_;
  // pcm7.link_name = "iiwa_link_7";
  // pcm7.weight = 1.0;
  // pcm7.constraint_region.primitives.emplace_back(cbox);
  // pcm7.constraint_region.primitive_poses.emplace_back(cbox_pose);
  // path_constraints.position_constraints.emplace_back(pcm7);
  
  // moveit_msgs::msg::PositionConstraint pcmC;
  // pcmC.header.frame_id = ref_link_;
  // pcmC.link_name = "iiwa_link_camera";
  // pcmC.weight = 1.0;
  // pcmC.constraint_region.primitives.emplace_back(cbox);
  // pcmC.constraint_region.primitive_poses.emplace_back(cbox_pose);
  // path_constraints.position_constraints.emplace_back(pcmC);

  // moveit_msgs::msg::PositionConstraint pcmee;
  // pcmee.header.frame_id = ref_link_;
  // pcmee.link_name = "iiwa_link_ee";
  // pcmee.weight = 1.0;
  // pcmee.constraint_region.primitives.emplace_back(cbox);
  // pcmee.constraint_region.primitive_poses.emplace_back(cbox_pose);
  // path_constraints.position_constraints.emplace_back(pcmee);
  
  // move_group_interface.setPathConstraints(path_constraints);

  const int N = 8;
  const double polar_range = 0.7 * PI / 2;
  const double rad = 0.10;
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
  Eigen::Vector3d s_pos(0.4, 0.6, 0.06);
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
        draw_trajectory_tool_path(plan.trajectory_);
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
      std::cout << "There have been " << s << " successes so far and " << f << " failures." << std::endl;
    }
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}