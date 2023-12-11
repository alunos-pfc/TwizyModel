# Use the official ROS Noetic image as the base image
FROM osrf/ros:noetic-desktop-full

# Install required packages
RUN apt-get update && apt-get install -y \
    wget \
    git \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-joy \
    python3-catkin-tools && \
    rm -rf /var/lib/apt/lists/*

# Create a Catkin workspace and set it up
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
RUN catkin init

# Clone the SD-TwizyModel repository
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/alunos-pfc/TwizyModel-Noetic.git

# Build the Catkin workspace
RUN catkin config --extend /opt/ros/noetic
WORKDIR /root/catkin_ws
RUN catkin build

# Source the ROS environment and the Catkin workspace in the entry point
# and create an alias for the keyboardlaunch script
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "alias control='bash /root/catkin_ws/src/TwizyModel-Noetic/streetdrone_model/sd_control/keyboardlaunch.sh'" >> /root/.bashrc

# Set the working directory to the home folder
WORKDIR /root