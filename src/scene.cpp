#include <algorithm>
#include <log4cxx/helpers/object.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class ObjectMarkerPublisher {

public:
  ObjectMarkerPublisher() {
    object_id_ = "object";
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(
        object_id_ + "_marker", 1, true);
    planning_scene_subscriber_ = nh_.subscribe<moveit_msgs::PlanningScene>(
        "/move_group/monitored_planning_scene", 1,
        std::bind(&ObjectMarkerPublisher::planningSceneCallback, this,
                  std::placeholders::_1));
  }

  void planningSceneCallback(const moveit_msgs::PlanningScene::ConstPtr &msg) {
    const auto objects = msg->world.collision_objects;
    const auto object_it = std::find_if(
        objects.cbegin(), objects.cend(),
        [this](const auto &object) { return object.id == object_id_; });
    if (object_it == objects.end()) {
      ROS_DEBUG_STREAM("'" << object_id_ << "' not found");
      return;
    }

    ROS_INFO_STREAM("'" << object_id_ << "' \033[92;1mfound\033[0m");

    const auto object = *object_it;
    // Create a Marker object
    visualization_msgs::Marker marker;
    marker.header = object.header;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = object.pose;
    marker.scale.x = 0.1;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 1.0;

    marker_publisher_.publish(marker);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher marker_publisher_;
  ros::Subscriber planning_scene_subscriber_;
  std::string object_id_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "marker_publisher");

  // Subscribe to the planning scene topic
  ObjectMarkerPublisher object_marker;
  // Spin until shutdown
  ros::spin();

  return 0;
}
