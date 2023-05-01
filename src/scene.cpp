#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <log4cxx/helpers/object.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

class ObjectMarkerPublisher {

public:
  ObjectMarkerPublisher() {
    // Instantiate subscribers and publishers
    object_id_ = "object";
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(
        object_id_ + "_marker", 1, true);
    planning_scene_subscriber_ = nh_.subscribe<moveit_msgs::PlanningScene>(
        "/move_group/monitored_planning_scene", 1,
        std::bind(&ObjectMarkerPublisher::planningSceneCallback, this,
                  std::placeholders::_1));
  }

  /**
   * @brief Planning-scene callback that publishes the arrow marker on the
   * desired object
   *
   * @param[in] msg Planning scene message
   */
  void planningSceneCallback(const moveit_msgs::PlanningScene::ConstPtr &msg) {
    // Find the object from the vector of world objects
    const auto &objects = msg->world.collision_objects;
    const auto object_it = std::find_if(
        objects.cbegin(), objects.cend(),
        [this](const auto &object) { return object.id == object_id_; });

    // Flag indicating if the object is found
    bool object_found = false;

    // Stamped pose of the found object
    geometry_msgs::PoseStamped object_pose;

    // Check collision objects if world objects are not found
    if (object_it != objects.end()) {
      ROS_DEBUG_STREAM("'" << object_id_ << "' \033[92;1mfound\033[0m");
      object_pose.pose = object_it->pose;
      object_pose.header = object_it->header;
      object_found = true;
    }

    // Search in attached object, if object is not found in world objects
    if (!object_found) {
      const auto &attached_objects =
          msg->robot_state.attached_collision_objects;
      const auto attached_object_it = std::find_if(
          attached_objects.cbegin(), attached_objects.cend(),
          [this](const auto &object) { return object.object.id == object_id_; });
      if (attached_object_it != attached_objects.cend()) {
        object_pose.pose = attached_object_it->object.pose;
        object_pose.header = attached_object_it->object.header;
        object_found = true;
      }
    }

    // Abort if object is not found
    if (!object_found) {
      ROS_DEBUG_STREAM("'" << object_id_ << "' not found");
      return;
    }

    // Create an arrow-marker object
    visualization_msgs::Marker marker;
    marker.header = object_pose.header;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    // The default orientation of the arrow is in the x-axis. However, we want
    // the axis to be on the z-axis. As such, apply the orientation change so
    // that the arrow points in the positive z-axis direction.
    tf2::Quaternion object_orientation;
    tf2::fromMsg(object_pose.pose.orientation, object_orientation);
    tf2::Quaternion orientation_change;
    orientation_change.setRPY(0, -M_PI_2, 0);
    const auto final_orientation = object_orientation * orientation_change;

    marker.pose = object_pose.pose;
    marker.pose.orientation = tf2::toMsg(final_orientation);
    marker.scale.x = 0.3;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 1.0;

    marker_publisher_.publish(marker);
  }

private:
  /** Node handle */
  ros::NodeHandle nh_;
  ros::Publisher marker_publisher_;

  /** Subscribe to topic that contains the object */
  ros::Subscriber planning_scene_subscriber_;

  /** Subscribe to the monitored planning scene */
  ros::Subscriber monitored_planning_scene_subscriber_;

  /** The scene object ID to add the marker to */
  std::string object_id_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "marker_publisher");

  // Object to subscribe to the object scene and publish the marker
  ObjectMarkerPublisher object_marker;
  ros::spin();

  return 0;
}
