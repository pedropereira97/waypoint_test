# waypoint_navigator

GPS/ENU co-ordinates are accepted as input destinations.
This README provides a brief overview of the package and its utilities.

## Installation Instructions (Ubuntu)

To install this package with [ROS Melodic]:

1. Install additional system dependencies:

```
sudo apt-get install python-wstool python-catkin-tools ros-melodic-cmake-modules
```

2. Set up a catkin workspace (if not already done):

```
mkdir -p ~/ros1_ws/src
cd ~/ros1_ws
catkin init
catkin config --extend /opt/ros/melodic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
```

3. Install the repository and its dependencies (with rosinstall):

```
cd src
wstool init
wstool merge waypoint_test/dependencies.rosinstall
wstool update -j4
echo "source ~/ros1_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
 
4. Use [catkin_build](http://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) to build the repository:

```
catkin build
```

## Node: waypoint_navigator

### Parameters:

* Parameters are stored in a .yaml file defining the trajectory to execute, which should be loaded to the parameter server. See ``trajectory_simple_enu.yaml`` and ``trajectory_simple_gps.yaml`` in the ``paths`` sub-folder for example trajectory files with parameter descriptions.

### Subscribed Topics:

* `/telem` - odometry received from MAV.

### Published Topics:

* `/cmd` - Friends command messages for ROS2 [published at 5Hz].

### Services:

* `/execute_path` - start the mission
* `/takeoff` - takeoff 
<--* `/land` - land
* `/abort_path` - stop sending waypoints--!>

## Input Format

The coordinates ``.yaml`` file should read as an array of floating-point numbers, separated by commas and spaces. There are two options:

1. **ENU points** (`coordinate_type = 'enu'`):
  Array format: [x (East), y (North), z (Up), height (above starting point), yaw angle (wrt. East)]

2. **GPS points** (`coordinate_type = 'gps'`):
  Array format: [Latitude, longitude, height (above initial reference), yaw angle (wrt. East)]
 
## Instructions

1. In a command window, open ROS2 node.

2. In a new command window,open ROS2-ROS1 bridge (https://github.com/ros2/ros1_bridge).

3. In a new command window, type:

 ```
 $ roslaunch waypoint_navigator waypoint_navigator.launch
 ```
 
  > **Note**: For GPS, we use a edited version of the [geodetic_utils](https://github.com/ethz-asl/geodetic_utils) package to establish a GPS reference point for the local frame. These parameters are set on the ROS parameter server: `/gps_ref_latitude`, `/gps_ref_longitude`, `/gps_ref_altitude`.
  
4. If using GPS, wait for it to be initialized.

5. In a new command window, type:

 ```
 $ rosservice call /friends_hexa/take_off
 ```
   
  Only execute this command if the drone had not taken off already.
   
5. In a new command window, type:

 ```
 $ rosservice call /firefly/execute_path
 ```
 
 This begins execution of the path that was read from the file. You should see the MAV moving along the path.
