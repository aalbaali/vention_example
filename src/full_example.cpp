/* Author: Ioan Sucan, Ridhwan Luthra*/

// ROS
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

void openGripper(trajectory_msgs::JointTrajectory & posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory & posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface & move_group)
{
  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single
  // grasp. This is essentially useful when using a grasp generator to generate
  // and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // Make sure that when you set the grasp_pose, you are setting it to be the
  // pose of the last link in your manipulator which in this case would be
  // `"panda_link8"` You will have to compensate for the transform from
  // `"panda_link8"` to the palm of the end effector.
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI_2, -M_PI_4, -M_PI_2);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.415;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.5;

  //// Setting pre-grasp approach
  //// ++++++++++++++++++++++++++
  //[> Defined with respect to frame_id <]
  // grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  //[> Direction is set as positive x axis <]
  // grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  // grasps[0].pre_grasp_approach.min_distance = 0.095;
  // grasps[0].pre_grasp_approach.desired_distance = 0.115;

  //// Setting post-grasp retreat
  //// ++++++++++++++++++++++++++
  //[> Defined with respect to frame_id <]
  // grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  //[> Direction is set as positive z axis <]
  // grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  // grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick2
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  move_group.setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  move_group.pick("object", grasps);
  // END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface & group)
{
  // BEGIN_SUB_TUTORIAL place
  // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied
  // place locations failed. Retrying last location in verbose mode." This is a
  // known issue. |br| |br| Ideally, you would create a vector of place
  // locations to be attempted although in this example, we only create a single
  // place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, tau / 4);  // A quarter turn about the z-axis
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* For place location, we set the value to the exact location of the center of
   * the object. */
  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.4;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("table2");
  // Call place to place the object using the place locations given.
  group.place("object", place_location);
  // END_SUB_TUTORIAL
}

void addCollisionObjects(
  moveit::planning_interface::PlanningSceneInterface & planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  //// Add the first table where the cube will originally be kept.
  // collision_objects[0].id = "table1";
  // collision_objects[0].header.frame_id = "panda_link0";

  //[> Define the primitive and its dimensions. <]
  // collision_objects[0].primitives.resize(1);
  // collision_objects[0].primitives[0].type =
  // collision_objects[0].primitives[0].BOX;
  // collision_objects[0].primitives[0].dimensions.resize(3);
  // collision_objects[0].primitives[0].dimensions[0] = 0.2;
  // collision_objects[0].primitives[0].dimensions[1] = 0.4;
  // collision_objects[0].primitives[0].dimensions[2] = 0.4;

  //[> Define the pose of the table. <]
  // collision_objects[0].primitive_poses.resize(1);
  // collision_objects[0].primitive_poses[0].position.x = 0.5;
  // collision_objects[0].primitive_poses[0].position.y = 0;
  // collision_objects[0].primitive_poses[0].position.z = 0.2;
  // collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  //// END_SUB_TUTORIAL

  // collision_objects[0].operation = collision_objects[0].ADD;

  //// BEGIN_SUB_TUTORIAL table2
  //// Add the second table where we will be placing the cube.
  // collision_objects[1].id = "table2";
  // collision_objects[1].header.frame_id = "panda_link0";

  //[> Define the primitive and its dimensions. <]
  // collision_objects[1].primitives.resize(1);
  // collision_objects[1].primitives[0].type =
  // collision_objects[1].primitives[0].BOX;
  // collision_objects[1].primitives[0].dimensions.resize(3);
  // collision_objects[1].primitives[0].dimensions[0] = 0.4;
  // collision_objects[1].primitives[0].dimensions[1] = 0.2;
  // collision_objects[1].primitives[0].dimensions[2] = 0.4;

  //[> Define the pose of the table. <]
  // collision_objects[1].primitive_poses.resize(1);
  // collision_objects[1].primitive_poses[0].position.x = 0;
  // collision_objects[1].primitive_poses[0].position.y = 0.5;
  // collision_objects[1].primitive_poses[0].position.z = 0.2;
  // collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  //// END_SUB_TUTORIAL

  // collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[0].header.frame_id = "panda_link0";
  collision_objects[0].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.02;
  collision_objects[0].primitives[0].dimensions[1] = 0.02;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the object. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.5;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "panda_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  group.setPlanningTime(45.0);
  ros::WallDuration(1.0).sleep();

  addCollisionObjects(planning_scene_interface);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

  bool success;

  // 1. Move to home position
  group.setJointValueTarget(group.getNamedTargetValues("home"));
  success = (group.plan(my_plan_arm) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  group.move();

  //// New target
  // ROS_INFO_STREAM("Setting new target");
  // geometry_msgs::PoseStamped current_pose;
  // current_pose = group.getCurrentPose("panda_link0");

  // geometry_msgs::Pose target_pose1;

  // target_pose1.orientation = current_pose.pose.orientation;
  // target_pose1.position = current_pose.pose.position;
  // target_pose1.position.x += 0.2;
  // group.setPoseTarget(target_pose1);
  // success = (group.plan(my_plan_arm) ==
  // moveit::core::MoveItErrorCode::SUCCESS);

  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s",
  //                success ? "" : "FAILED");
  // group.move();
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

  //// 3. Open the gripper
  // group.setJointValueTarget(group.getNamedTargetValues("open"));
  // success =
  //     (group.plan(my_plan_gripper) ==
  //     moveit::core::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Opening gripper: %s", success ? "" : "FAILED");
  // group.move();

  //// 4. Move the TCP close to the object
  // target_pose1.position.z = target_pose1.position.z + 0.2;
  // group.setPoseTarget(target_pose1);
  // success = (group.plan(my_plan_arm) ==
  // moveit::core::MoveItErrorCode::SUCCESS); ROS_INFO_NAMED("tutorial", "Moving
  // TCP close to the object %s",
  //                success ? "" : "FAILED");
  // group.move();

  //// 5. Close the  gripper
  // group.setJointValueTarget(group.getNamedTargetValues("closed"));

  // success =
  //     (group.plan(my_plan_gripper) ==
  //     moveit::core::MoveItErrorCode::SUCCESS);

  // ROS_INFO_NAMED("tutorial", "Closing gripper %s",
  //                success ? "" : "FAILED");

  // group.move();

  //// ALTERNATIVE: close the gripper
  // moveit_msgs::Grasp grasp;
  // grasp.grasp_pose.header.frame_id = "panda_link0";
  //// grasp.grasp_pose.pose = target_pose1;
  // closedGripper(grasp.grasp_posture);

  ////// Set the target joint values for closing the gripper
  //// std::vector<double> joint_values = {0.0, 0.0};

  ////// Set the joint state target for the Panda gripper
  //// group.setJointValueTarget(joint_values);

  //// Move the Panda gripper to the target joint state
  // moveit::planning_interface::MoveGroupInterface::Plan plan;
  // success = group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  // ROS_INFO_NAMED("tutorial", "Closing gripper %s", success ? "" : "FAILED");

  // ros::WallDuration(1.0).sleep();
  // group.move();

  // ros::WallDuration(1.0).sleep();
  // ROS_INFO("Done closing gripper");

  ros::waitForShutdown();
  return 0;
}
