
#include <ros/package.h>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>

#include <waypoint_navigator/waypoint_navigator_node.h>

namespace waypoint_navigator {
const double WaypointNavigatorNode::kCommandTimerFrequency = 5.0;
const double WaypointNavigatorNode::kWaypointAchievementDistance = 0.2;

WaypointNavigatorNode::WaypointNavigatorNode(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      got_telemetry_(false)

{
  loadParameters();

  telemetry_subscriber_ = nh_.subscribe(
      "telem", 1, &WaypointNavigatorNode::telemetryCallback, this);

  cmd_publisher_ = nh_.advertise<std_msgs::String>("/cmd", 1);

  start_service_ = nh_.advertiseService(
      "execute_path", &WaypointNavigatorNode::executePathCallback, this);

  takeoff_service_ = nh_.advertiseService(
      "take_off", &WaypointNavigatorNode::takeoffCallback, this);

  // Wait until GPS reference parameters are initialized.
  while (!geodetic_converter_.isInitialised()) {
    LOG_FIRST_N(INFO, 1) << "Waiting for GPS reference parameters...";

    double latitude;
    double longitude;
    double altitude;

    if (nh_private_.getParam("/gps_ref_latitude", latitude) &&
        nh_private_.getParam("/gps_ref_longitude", longitude) &&
        nh_private_.getParam("/gps_ref_altitude", altitude)) {
      geodetic_converter_.initialiseReference(latitude, longitude, altitude);
      ROS_INFO_STREAM("Initial Pose: " << latitude << ", " << longitude << ", " << altitude);
    } else {
      LOG(INFO) << "GPS reference not ready yet, use set_gps_reference_node_telem to "
                   "set it.";
      ros::Duration(0.5).sleep();
    }
  }


  LOG(INFO)
      << "Waypoint navigator ready. Call 'take_off' service to get going.";

  //WaypointNavigatorNode::executePathCallback();
}

void WaypointNavigatorNode::loadParameters() {
  CHECK(
      nh_private_.getParam("coordinate_type", coordinate_type_) &&
      nh_private_.getParam("heading_mode", heading_mode_))
      << "Error loading parameters!";

  if (coordinate_type_ == "gps" || coordinate_type_ == "enu") {
  } else {
    LOG(FATAL) << ("Unknown coordinate type - please enter 'gps' or 'enu'.");
  }

  if (heading_mode_ == "auto" || heading_mode_ == "manual" ||
      heading_mode_ == "zero") {
  } else {
    LOG(FATAL) << "Unknown heading alignment mode - please enter 'auto', "
                  "'manual', or 'zero'.";
  }

}

bool WaypointNavigatorNode::loadPathFromFile() {
  // Fetch the trajectory from the parameter server.
  std::vector<double> easting;
  std::vector<double> northing;
  std::vector<double> height;
  std::vector<double> heading;

  CHECK(nh_private_.getParam("easting", easting) &&
        nh_private_.getParam("northing", northing) &&
        nh_private_.getParam("height", height))
      << "Error loading path parameters!";

  if (heading_mode_ == "manual" && !nh_private_.getParam("heading", heading)) {
    LOG(FATAL) << "Heading in manual mode is unspecified!";
  }

  // Check for valid trajectory inputs.
  if (!(easting.size() == northing.size() &&
        northing.size() == height.size())) {
    LOG(FATAL) << "Error: path parameter arrays are not the same size";
  }
  if (heading_mode_ == "manual" && !(height.size() == heading.size())) {
    LOG(FATAL) << "Error: path parameter arrays are not the same size";
  }

  coarse_waypoints_.clear();
  addCurrentTelemetryWaypoint();

  // Add (x,y,z) co-ordinates from file to path.
  for (size_t i = 0; i < easting.size(); i++) {
    friends_msgs::Position cwp;
    // GPS path co-ordinates.
    if (coordinate_type_ == "gps") {
      cwp.longitude_deg = easting[i];
      cwp.latitude_deg = northing[i];
      cwp.relative_altitude_m = height[i];
    }
    // ENU path co-ordinates.
    else if (coordinate_type_ == "enu") {
      // Convert ENU point to GPS co-ordinates.
      geodetic_converter_.enu2Geodetic(
          easting[i], northing[i], height[i],
          &cwp.latitude_deg, &cwp.longitude_deg, &cwp.relative_altitude_m);
      
    }
    coarse_waypoints_.push_back(cwp);
  }

  // Add heading from file to path.
  for (size_t i = 0; i < coarse_waypoints_.size()-1; i++) {
    if (heading_mode_ == "manual") {
      coarse_waypoints_[i].heading = heading[i];
    } else if (heading_mode_ == "auto") {
      // Compute heading in direction towards next point.
      coarse_waypoints_[i].heading = 
          (atan2( sin(coarse_waypoints_[i+1].longitude_deg - coarse_waypoints_[i].longitude_deg)*cos(coarse_waypoints_[i+1].latitude_deg),
                  cos(coarse_waypoints_[i].latitude_deg)*sin(coarse_waypoints_[i+1].latitude_deg) - 
                  sin(coarse_waypoints_[i].latitude_deg)*cos(coarse_waypoints_[i+1].latitude_deg)*cos(coarse_waypoints_[i+1].longitude_deg - coarse_waypoints_[i].longitude_deg)) / M_PI) * 180.0;
    } else if (heading_mode_ == "zero") {
      coarse_waypoints_[i].heading = 0.0;
    }
  }

  // As first target point, add current (x,y) position, but with height at
  // that of the first requested waypoint, so that the MAV first adjusts height
  // moving only vertically.
  if (coarse_waypoints_.size() >= 2) {
    friends_msgs::Position vwp;
    vwp.latitude_deg = telemetry_.position.latitude_deg;
    vwp.longitude_deg = telemetry_.position.longitude_deg;
    vwp.relative_altitude_m = coarse_waypoints_[1].relative_altitude_m;
    if (heading_mode_ == "zero") {
      vwp.heading = 0.0;
    } else if (heading_mode_ == "manual") {
      // Do not change heading.
      vwp.heading = coarse_waypoints_[0].heading;
    }
    coarse_waypoints_.insert(coarse_waypoints_.begin() + 1, vwp);
  }

  LOG(INFO) << "Path loaded from file. Number of points in path: "
            << coarse_waypoints_.size();

  current_leg_ = 0;
  return true;
}

void WaypointNavigatorNode::addCurrentTelemetryWaypoint() {
  friends_msgs::Position vwp;
  vwp = telemetry_.position;
  coarse_waypoints_.push_back(vwp);
}

void WaypointNavigatorNode::publishCommands() {
  command_timer_ = nh_.createTimer(ros::Duration(1.0 / kCommandTimerFrequency),
                        &WaypointNavigatorNode::poseTimerCallback, this);
}

bool WaypointNavigatorNode::executePathCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  CHECK(got_telemetry_)
      << "No telemetry received yet, can't start path following.";
  CHECK(telemetry_.on_air())
      << "Not ready. Please take off first.";
  command_timer_.stop();
  current_leg_ = 0;
  timer_counter_ = 0;

  CHECK(loadPathFromFile()) << "Path could not be loaded!";

  publishCommands();
  LOG(INFO) << "Starting path execution...";
  return true;
}

bool WaypointNavigatorNode::executePathFromFileCallback() {
  // Stop executing the current path.
  std_srvs::Empty::Request empty_request;
  std_srvs::Empty::Response empty_response;
  abortPathCallback();

  std::string path_filename =
      ros::package::getPath("waypoint_navigator") + "/paths/" + "trajectory_simple_enu";
  std::string load_command =
      "rosparam load " + path_filename + ".yaml " + ros::this_node::getName();

  if (system(load_command.c_str()) != 0) {
    LOG(ERROR) << "New mission parameters not loaded properly!";
    return false;
  }

  executePathCallback(empty_request, empty_response);
  return true;
}

bool WaypointNavigatorNode::armCallback() {
  std_msgs::String msg;
  msg.data = friends_msgs::create_cmd("arm", telemetry_.droneID);
  cmd_publisher_.publish(msg);
  return true;
}

bool WaypointNavigatorNode::landCallback() {
  std_msgs::String msg;
  msg.data = friends_msgs::create_cmd("land", telemetry_.droneID);
  cmd_publisher_.publish(msg);
  return true;
}

bool WaypointNavigatorNode::takeoffmess() {
  std_msgs::String msg;
  msg.data = friends_msgs::create_cmd("takeoff", telemetry_.droneID);
  cmd_publisher_.publish(msg);
  return true;
}

bool WaypointNavigatorNode::abortPathCallback() {
  coarse_waypoints_.clear();

  // Stop sending commands to the controller.
  command_timer_.stop();
  LOG(INFO) << "Aborting path execution...";
  return true;
}

void WaypointNavigatorNode::poseTimerCallback(const ros::TimerEvent&) {
  // Check for leg completion based on distance.
  // If current leg has been completed, go to the next one.
  double pos_x, pos_y, pos_z;
  double target_x, target_y, target_z;
  geodetic_converter_.geodetic2Enu(telemetry_.position.latitude_deg, telemetry_.position.longitude_deg, telemetry_.position.relative_altitude_m, &pos_x, &pos_y, &pos_z); 
  geodetic_converter_.geodetic2Enu(coarse_waypoints_[current_leg_].latitude_deg, coarse_waypoints_[current_leg_].longitude_deg, coarse_waypoints_[current_leg_].relative_altitude_m, &target_x, &target_y, &target_z);

  const double dist_to_end = sqrt((pos_x-target_x)*(pos_x-target_x) + (pos_y-target_y)*(pos_y-target_y) + (pos_z-target_z)*(pos_z-target_z));

  ROS_INFO_STREAM(dist_to_end);

  if (current_leg_ != coarse_waypoints_.size() - 1 &&
      dist_to_end < kWaypointAchievementDistance) {
    if (current_leg_ == 0) {
      LOG(INFO) << "Going to first waypoint... ";
    } else {
      LOG(INFO) << "Leg " << current_leg_ << " of "
                << coarse_waypoints_.size() - 1 << " completed!";
    }
    current_leg_++;
  }

  std_msgs::String msg;
  msg.data = friends_msgs::create_cmd_goto(telemetry_.droneID, coarse_waypoints_[current_leg_]);
  cmd_publisher_.publish(msg);
  timer_counter_++;
}

void WaypointNavigatorNode::telemetryCallback(const std_msgs::String::ConstPtr& telemetry_message) {
  if (!got_telemetry_) {
    got_telemetry_ = true;
  }

  telemetry_.fromString(telemetry_message);
}

bool WaypointNavigatorNode::takeoffCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
  armCallback();
  takeoffmess();
  LOG(INFO)
    << "Takeoff done. Call 'execute_path' service to get going.";
}
}



int main(int argc, char** argv) {
  // Start the logging.
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  // Initialize ROS, start node.
  ros::init(argc, argv, "WaypointNavigatorNode");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  waypoint_navigator::WaypointNavigatorNode waypoint_navigator_node(nh,
                                                                    nh_private);
  ros::spin();
  return 0;
}
