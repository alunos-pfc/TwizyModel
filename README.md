# SD Twizy Vehicle Simulation

Gazebo simulation packages for the SD Twizy vehicle

### Changes from Original Repository

The original repository was designed to work with ROS Kinetic. This version has been modified to work with ROS Noetic (Ubuntu 20.04). 

For the original version, please visit the [original StreetDrone repository](https://github.com/streetdrone-home/SD-TwizyModel).

## Requirements:

##### - Ubuntu 20.04 LTS
##### - ROS noetic [ros-noetic-desktop-full](http://wiki.ros.org/noetic/Installation/Ubuntu)
##### - Catkin Command Line Tools [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)
##### - Gazebo [ros-noetic-gazebo-ros-pkgs](http://gazebosim.org/tutorials?tut=ros_installing)  
This model has been tested with Gazebo 11. Run `gazebo --version` to make sure you have the correct version installed.  

## Docker

For users who prefer not to install all the dependencies, a Docker container is available. This container has all the necessary dependencies pre-installed.

To use the Docker container, please follow the instructions provided at the following link:

[Docker Instructions](https://github.com/alunos-pfc/TwizyModel-Noetic/tree/master/Docker)

## Setup your workspace:

### Install required dependencies:
```bash
apt install \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-joy \
    ros-noetic-velodyne-simulator
```
- `ros-noetic-teleop-twist-keyboard` is required for the keyboard control.
- `ros-noetic-joy` is required for the joystick control.
- `ros-noetic-velodyne-simulator` is required for the Velodyne VLP-16 LiDAR model.


### A. Create a catkin workspace:
To setup your workspace after installing ROS noetic and catkin tools, do:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```
For more information, visit [create_a_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

### B. Clone this repository to your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/alunos-pfc/TwizyModel-Noetic.git
```

### C. Navigate to your workspace, install the dependencies and build the simulation
```
cd ~/catkin_ws
catkin config --extend /opt/ros/noetic
rosdep install --from-paths src/ --ignore-src -r -y
catkin build
```

After the built has successfully finished, source ros and your workspace:
```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

### D. Models Download (Optional)

Due to a bug in Gazebo, the required models are not downloaded automatically, so we provide a script to download them.

Before running the simulation, you can download the required models by using the provided script:
```bash
cd ~/catkin_ws/src/TwizyModel-Noetic/streetdrone_model
./download_models.sh
```
That step is optional, but if you don't do it, you will only be able to use empty world.

### E. Launch the simulation:
This launches the vehicle model in Gazebo and RViz for visualizing the sensors' output.
```
roslaunch sd_robot sd_twizy_empty.launch
# OR roslaunch sd_robot sd_twizy_worlds.launch enable_rviz:=true world:=empty
```

For more detailed information on launch, refer to the [robot page](https://github.com/alunos-pfc/TwizyModel-Noetic/tree/master/streetdrone_model/sd_robot)

<p align="center"> 
<img src="streetdrone_model/sd_docs/imgs/sd.png">
</p>

## Sensors
**LiDAR:** VLP - 16 Velodyne  
**Cameras:** 8 x Blackfly S 2.3MP  
The scripts for the sensors are written based on the common scripts that exist for sensors in Gazebo.

## Controlling the Robot
### Joystick
The robot supports the generic Linux
[joystick](http://wiki.ros.org/joy) controllers. The `sd_control`
package contains a node to turn joystick commands into control
messages that drive the throttle and steering of the model. To use
this, launch a simulation as described above, then run the following:
```
roslaunch sd_control sd_twizy_control_teleop.launch
```

You can map specific buttons using the parameters defined in that
launch file. For instance, the following uses the left stick for
throttle, the right stick for steering, and right button (RB) to
enable control on a Logitech F710 Gamepad:
```
roslaunch sd_control sd_twizy_control_teleop.launch enable_button:=5 throttle_axis:=1 steer_axis:=2
```

### Keyboard
The simulation can also be controlled by the keyboard.

To launch the sd_teleop_keyboard node, run the following:
```
bash ~/catkin_ws/src/TwizyModel-Noetic/streetdrone_model/sd_control/keyboardlaunch.sh 
```
And follow the instructions on the terminal.


## Vehicle Interface

The StreetDrone Vehicle Interface is a ROS package that provides a bridge between the SD-TwizyModel simulation and the StreetDrone Vehicle API.

### Installation

### A. Clone the SD-VehicleInterface repository to your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone --single-branch -b melodic-devel https://github.com/streetdrone-home/SD-VehicleInterface.git
```

### B. Remove the msgs folder (duplicate of the can_msgs package) from the SD-VehicleInterface package:
```bash
rm -r ~/catkin_ws/src/SD-VehicleInterface/msgs/
```

### C. Change code in the socketcan_bridge_node.cpp file:

The SD-VehicleInterface package is not compatible with ROS Noetic. To fix this, you need to change the code in the socketcan_bridge_node.cpp file.

Open the file `SD-VehicleInterface/vehicle_interface/src/socketcan_bridge/socketcan_bridge_node.cpp` in a text editor of your choice.

Replace the existing code in line 45:
```cpp
can::ThreadedSocketCANInterfaceSharedPtr driver = std::make_shared<can::ThreadedSocketCANInterface> ();
```
with the following code:
```cpp
std::shared_ptr<can::ThreadedInterface<can::SocketCANInterface>> driver = std::make_shared<can::ThreadedSocketCANInterface>();
```

Or simply run the following command:

```bash
sed -i 's/can::ThreadedSocketCANInterfaceSharedPtr/std::shared_ptr<can::ThreadedInterface<can::SocketCANInterface>>/' ~/catkin_ws/src/SD-VehicleInterface/vehicle_interface/src/socketcan_bridge/socketcan_bridge_node.cpp
```

### D. Build the SD-VehicleInterface package:

```bash
cd ~/catkin_ws
catkin build vehicle_interface
```

### E. Launch the SD-VehicleInterface package:

After building the SD-VehicleInterface package and starting the simulation, you can launch the SD-VehicleInterface package:

1. Open a new terminal window.
2. Source ros and your workspace:
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```
3. Run the following command:

```bash
roslaunch sd_vehicle_interface sd_vehicle_interface.launch sd_vehicle:=twizy sd_gps_imu:=none sd_simulation_mode:=true
```

For more detailed information on the StreetDrone Vehicle Interface, refer to the official documentation: [SD-VehicleInterface Documentation](https://github.com/streetdrone-home/SD-VehicleInterface)