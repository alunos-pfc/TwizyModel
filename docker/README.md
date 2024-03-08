# SD-TwizyModel Simulation

This repository includes a Dockerfile to create a Docker image for running the SD-TwizyModel on ROS Noetic with and without the [StreetDrone Vehicle Interface](https://github.com/streetdrone-home/SD-VehicleInterface).

All credit for the simulation goes to the original StreetDrone repository: [SD-TwizyModel](https://github.com/streetdrone-home/SD-TwizyModel).

Always refer to the original repository for more detailed information.

## Cloning the Repository

Clone this repository.

```bash	
git clone https://github.com/alunos-pfc/TwizyModel-Noetic
```

Navigate to the docker folder inside the repository.

```bash
cd TwizyModel-Noetic/Docker
```

## Building the Image

### With StreetDrone Vehicle Interface

Build the Docker image with SD-VI using the following command:

```bash
docker build -t twizymodel-noetic-vi -f simulation-with-vi/Dockerfile .
```

### Without StreetDrone Vehicle Interface

Build the Docker image without SD-VI using the following command:

```bash
docker build -t twizymodel-noetic -f simulation/Dockerfile .
```

## Models Download (Optional)

Due to a bug in Gazebo, the required models are not downloaded automatically, so we provide a script to download them.

### Download models outside container
Before running the simulation, you can download the required models by using the provided script:

```bash
./download_models.sh
```

This step ensures that the essential models are available for a successful simulation run.

It will create a `models` folder in the root directory, which will be linked to Gazebo models within the container when running with the provided run script.

### Download models inside the container

If you prefer to download the models inside the container, there is a script available for that. Run the following command after launching the container:

```bash
cd ~/catkin_ws/src/TwizyModel-Noetic/streetdrone_model
./download_models.sh
```

## Running the Container

To run the Docker container, utilize the provided run script with the following parameters:

```bash
./run.sh <image-name> [--rm] [--nvidia]
```

- `<image-name>`: The name you assigned to the Docker image during the build process.
- `--rm`: Automatically remove the container when it exits.
- `--nvidia`: Run the container with NVIDIA GPU support.

If the `models` folder is present in the root directory, it will be linked to Gazebo models within the container. This linking is necessary due to a bug preventing Gazebo from downloading the required models.

If there is a `bags` folder in the `Docker` directory, it will be linked to the `bags` folder inside the container. This linking is necessary if you want to save the LiDAR data to your computer.

# Inside Container - Simulation Setup

## Launching the Simulation

The package includes three different world configurations built using the default Gazebo models. To launch the worlds, use the following command:

```bash
roslaunch sd_robot sd_twizy_worlds.launch enable_rviz:=true world:=default gpu:=true
```

You can customize the launch with the following arguments:

| arg         | values                         | default | description                      |
|-------------|--------------------------------|---------|----------------------------------|
| enable_rviz | {true, false}                  | true    | Launch RVIZ alongside Gazebo     |
| world       | {default, empty, park, shapes} | default | Gazebo world                     |
| gpu         | {true, false}                  | false   | Enable GPU support in simulation |

For more detailed information, refer to the [robot page](https://github.com/alunos-pfc/TwizyModel-Noetic/tree/master/streetdrone_model/sd_robot).

## Controlling the Robot

You will need to open a new terminal window to control the robot.

On a new terminal window, run the following command to enter the container:
```bash
docker exec -it <container-name> bash
```

### Keyboard Control

You can control the simulation using the keyboard by running:

```bash
cd ~/catkin_ws/src/TwizyModel-Noetic/streetdrone_model/sd_control
bash keyboardlaunch.sh 
```

For additional details on control configurations, refer to [README](https://github.com/alunos-pfc/TwizyModel-Noetic/blob/master/README.md).

## Recording LiDAR Data

If you want to save it to your computer, create a `bags` folder in the `TwizyModel-Noetic/Docker` directory before running the container and it will be linked to the `bags` folder inside the container.

If you have created the `bags` folder, navigate to it inside the container:

```bash
cd /root/bags
```

Once the simulation is running, you can record the LiDAR data by running the following command in a new terminal window:

```bash
rosbag record -O <Filename>.bag /points_raw
```

To stop recording, press `Ctrl+C` in the terminal window where the rosbag record command was executed.

## Vehicle Interface

The StreetDrone Vehicle Interface is a ROS package that provides a bridge between the SD-TwizyModel simulation and the StreetDrone Vehicle API.
If you are using the Docker image with SD-VehicleInterface, you can launch the Vehicle Interface by:

1. Opening a new terminal window.
2. Running the following command to enter the container:

```bash
docker exec -it <container-name> bash
```

3. Running the following command to launch the Vehicle Interface:

```bash
roslaunch sd_vehicle_interface sd_vehicle_interface.launch sd_vehicle:=twizy sd_gps_imu:=none sd_simulation_mode:=true
```

For more detailed information on the StreetDrone Vehicle Interface, refer to the official documentation: [SD-VehicleInterface Documentation](https://github.com/streetdrone-home/SD-VehicleInterface)

## Customizations

- Aliases are added to facilitate common commands:
    - `control`: launches the keyboard control.
    - `vehicle_interface`: launches the StreetDrone Vehicle Interface.