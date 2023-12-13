# Use the official ROS Noetic image as the base image
FROM osrf/ros:noetic-desktop-full

# Install required packages
RUN apt-get update && apt-get install -y \
    wget \
    git \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-joy \
    python3-catkin-tools ros-noetic-socketcan-interface \
    ros-noetic-can-msgs && \
    rm -rf /var/lib/apt/lists/*

# Create a Catkin workspace and set it up
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
RUN catkin init

# Clone the SD-TwizyModel repository and the SD-VehicleInterface repository
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/alunos-pfc/TwizyModel-Noetic.git
RUN git clone --single-branch -b melodic-devel https://github.com/streetdrone-home/SD-VehicleInterface.git

# Remove the msgs folder (duplicate of the can_msgs package) and replace the ThreadedSocketCANInterfaceSharedPtr with the correct type for noetic
WORKDIR /root/catkin_ws/src/SD-VehicleInterface
RUN rm -r msgs/
RUN sed -i 's/can::ThreadedSocketCANInterfaceSharedPtr/std::shared_ptr<can::ThreadedInterface<can::SocketCANInterface>>/' /root/catkin_ws/src/SD-VehicleInterface/vehicle_interface/src/socketcan_bridge/socketcan_bridge_node.cpp


# Build the Catkin workspace
WORKDIR /root/catkin_ws
RUN catkin config --extend /opt/ros/noetic
RUN catkin build

# Source the ROS environment and the Catkin workspace in the entry point
# and create an alias for the keyboardlaunch script and the vehicle_interface launch file
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "alias control='bash /root/catkin_ws/src/TwizyModel-Noetic/streetdrone_model/sd_control/keyboardlaunch.sh'" >> /root/.bashrc && \
    echo "alias vehicle_interface='roslaunch sd_vehicle_interface sd_vehicle_interface.launch sd_vehicle:=twizy sd_gps_imu:=none sd_simulation_mode:=true'" >> /root/.bashrc

# Set the working directory to the home folder
WORKDIR /root