// ROS
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>
#include <shape_msgs/SolidPrimitive.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

/**
 * @brief Create scene objects
 *
 * @details Create the box to be picked and placed by the robot
 *
 * @return Vector of scene objects
 */
std::vector<moveit_msgs::CollisionObject> CreateCollisionObjects() {
  // Box to be moved
  moveit_msgs::CollisionObject box;
  box.header.frame_id = "panda_link0";
  box.id = "object";

  // Define box dimensions
  box.primitives.resize(1);
  box.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  box.primitives[0].dimensions = {0.2, 0.2, 0.2};

  // Define the pose of the object
  box.primitive_poses.resize(1);
  box.primitive_poses[0].position.x = 0.0;
  box.primitive_poses[0].position.y = 1.0;
  box.primitive_poses[0].position.z = 0;
  box.primitive_poses[0].orientation.w = 1.0;
  box.operation = moveit_msgs::CollisionObject::ADD;

  return {box};
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "scene");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Add collision objects to scene
  const auto collision_objects = CreateCollisionObjects();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObjects(std::move(collision_objects));

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  ros::waitForShutdown();
}
