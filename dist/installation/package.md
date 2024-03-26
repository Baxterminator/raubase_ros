# Raubase_ROS package installation

!!! warning
    Prior to any installation, make sure ROS2 Humble is installed on your computer and sourced in the terminal. 

    To install ROS2 Humble, follow the official installation guides from the [ROS2 Documentation](https://docs.ros.org/en/humble/Installation.html).

    To source ROS2, use the following command:
    ```shell
    . /opt/ros/humble/setup.sh
    ```

## Preparing the environnement

As any ROS package, you must first make a workspace to build the several packages that you will need.

For that, create a directory wherever you want, and create a `src` folder inside.

```shell
mkdir dtu_ws/src
cd dtu_ws/src
```

## Fetching the packages and dependencies

The next step is to install the packages and their dependencies from Github.

```shell
sudo apt update
sudo apt install ros-humble-camera-calibration-parsers ros-humble-camera-info-manager ros-humble-launch-testing-ament-cmake
git clone https://github.com:ros-perception/image_pipeline.git -b humble
git clone https://github.com:Baxterminator/raubase_msgs.git
git clone https://github.com:Baxterminator/raubase_ros.git
```

## (Optional) Compile only necessary packages

The two first repository in the list are meta-packages (i.e. repositories that gather several packages in one place so that you can compile everything together), and you don't really need everything. In fact the only packages that we really need are:

- `image_pipeline/image_calibration`: a tool to calibrate cameras in ROS2[^2]


[^1]: Cf. the description made on the [Vision_OpenCV Package Github](https://github.com/ros-perception/vision_opencv/tree/humble)
[^2]: Installation instruction and usage of the calibration tool can be found on the [Nav2 Wiki](https://navigation.ros.org/tutorials/docs/camera_calibration.html)

In the end, a lot of the `image_pipeline` packages are not really needed.

```shell
echo > image_pipeline/COLCON_IGNORE
mv image_pipeline/image_calibration image_calibration
```

## Building everything

Now is the time to build everything:

```shell
cd ..
colcon build
```

## Sourcing the whole stack

When everything is built, you can source the stack for further purposes. The method doesn't change from a regular ROS2 workspace, so you can rely on the [ROS2 Documentation](https://docs.ros.org/en/humble/Installation.html) for this part. Nonetheless, we give here a reminder on how to do it:

```shell
source ./install/local_setup.sh
```

!!! info
    If you want to source automatically the workspace on startup, you can add the following line in your .bashrc / .zshrc

    ```shell
    source "/absolute_path_to_workspace/install/setup.sh"
    ```
