# SD-TwizyModel Simulation

This repository includes a Dockerfile to create a Docker image for running the 
SD-TwizyModel on ROS Noetic.

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

Build the Docker image using the following command:

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

## Running the Image

To run the Docker image, utilize the provided run script with the following parameters:

```bash
./run.sh <image-name> [--rm] [--no-nvidia]
```

- `<image-name>`: The name you assigned to the Docker image during the build process.
- `--rm`: Automatically remove the container when it exits.
- `--no-nvidia`: Run the container without NVIDIA GPU support.

If the `models` folder is present in the root directory, it will be linked to Gazebo models within the container. This linking is necessary due to a bug preventing Gazebo from downloading the required models.

# Inside Container - Simulation Setup

## Launching the Simulation

The package includes three different world configurations built using the default Gazebo models. To launch the worlds, use the following command:

```bash
roslaunch sd_robot sd_twizy_worlds.launch enable_rviz:=true world:=default
```

You can customize the launch with the following arguments:

| arg         | values                         | default | description                  |
|-------------|--------------------------------|---------|------------------------------|
| enable_rviz | {true, false}                  | true    | Launch RVIZ alongside Gazebo |
| world       | {default, empty, park, shapes} | default | Gazebo world                 |

For more detailed information, refer to the [robot page](https://github.com/alunos-pfc/TwizyModel-Noetic/tree/master/streetdrone_model/sd_robot).

## Controlling the Robot

You will need to open a new terminal window to control the robot.

On a new terminal window, run the following command:
```bash
docker exec -it <container-name> bash
```

### Keyboard Control

You can control the simulation using the keyboard by running:

```bash
control
```

Which is an alias for the original command:

```bash
bash ~/catkin_ws/src/TwizyModel-Noetic/streetdrone_model/sd_control/keyboardlaunch.sh 
```

For additional details on control configurations, refer to [README](https://github.com/alunos-pfc/TwizyModel-Noetic/blob/master/README.md).