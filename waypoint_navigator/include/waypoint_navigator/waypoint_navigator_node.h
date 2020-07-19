#ifndef WAYPOINT_NAVIGATOR_NODE_H
#define WAYPOINT_NAVIGATOR_NODE_H

#include <glog/logging.h>
#include <gtest/gtest_prod.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <stdio.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen>
#include "tf/tf.h"
#include <friends_msgs/friends_msgs.h>
#include <geodetic_utils/geodetic_conv.hpp>
#include <std_srvs/Empty.h>


namespace waypoint_navigator {
class WaypointNavigatorNode {
 public:
  WaypointNavigatorNode(const ros::NodeHandle& nh,
                        const ros::NodeHandle& nh_private);
  WaypointNavigatorNode(const WaypointNavigatorNode&) = delete;
  WaypointNavigatorNode& operator=(const WaypointNavigatorNode&) = delete;
  ~WaypointNavigatorNode() = default;

 private:
  void loadParameters();

  bool loadPathFromFile();

  void addCurrentTelemetryWaypoint();

  void publishCommands();

  bool executePathCallback(std_srvs::Empty::Request& request,
                           std_srvs::Empty::Response& response);

  bool executePathFromFileCallback();

  // Send arm command.
  bool armCallback();
  // Send a landing command.
  bool landCallback();
  // Send a take-off command.
  bool takeoffmess();
  // Cancel mission and keep helicopter in current position
  bool abortPathCallback();

  // Publishes a single waypoint to go to if the path mode is 'poses' [5Hz].
  void poseTimerCallback(const ros::TimerEvent&);

  bool takeoffCallback(std_srvs::Empty::Request& request, 
                        std_srvs::Empty::Response& response);


  void telemetryCallback(const std_msgs::String::ConstPtr& telemetry_msg);

  static const double kCommandTimerFrequency;
  // Distance before a waypoint is considered reached [m].
  static const double kWaypointAchievementDistance;
  // Minimum distance between intermediate waypoints [m].
  static const double kIntermediatePoseTolerance;

  // ROS comms.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher cmd_publisher_;

  ros::Subscriber telemetry_subscriber_;

  ros::ServiceServer start_service_, takeoff_service_;

  ros::Timer command_timer_;

  // Parameters.
  // GPS/ENU coordinates.
  std::string coordinate_type_;
  // Heading alignment method.
  std::string heading_mode_;

  // Geodetic coordinate conversion (from lat/lon to Cartesian ENU).
  //geodetic_converter::GeodeticConverter geodetic_converter_;

  bool got_telemetry_;
  friends_msgs::Telemetry telemetry_;

  // A list of waypoints to visit.
  // [x,y,z,heading]
  std::vector<friends_msgs::Position> coarse_waypoints_;

    // Geodetic coordinate conversion (from lat/lon to Cartesian ENU).
  geodetic_converter::GeodeticConverter geodetic_converter_;

  // Path execution state (for pose publishing).
  size_t current_leg_;

  // Callback number for command_timer_.
  unsigned int timer_counter_;
};
}  // namespace: waypoint_navigator

#endif  // WAYPOINT_NAVIGATOR_NODE_H
