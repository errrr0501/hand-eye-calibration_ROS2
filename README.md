# Universal Automatical Hand-Eye-Calibration
This Hand-eye_calibration method include [FlexBE](http://philserver.bplaced.net/fbe/download.php), and [MoveIt!](https://moveit.ros.org/), which make this whole framework easier for beginner to do hand-eye-calibration, also implement on different platform more faster.

## Acknowledgment
This charuco marker detection is forked from the [charuco_detector](https://github.com/carlosmccosta/charuco_detector).

## Requirements 
This package requires a system setup with ROS. It is recommended to use **Ubuntu 20.04 with ROS
noetic**, however using Ubuntu 18.04 with ROS melodic should also work.

camera SDK and ROS package is needed. Here we use [Linux Distribution](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages) and [Realsense-ros](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy)


To make sure that program isn't affected by python version, it is highly recommended to use a docker, 
we have build a environment that all Requirement for this package is build, 
See the [ubuntu-docker](https://github.com/errrr0501/ubuntu20.04_docker) on information how to set this up.

## Building

```bash

# Install MoveIt 
sudo apt install ros-$ROS_DISTRO--moveit
# Clone FlexBE app repository to your workspace
git clone https://github.com/FlexBE/flexbe_app.git

# Clone this hand-eye-cliabtion package to your workspace
git clone --recurse-submodules https://github.com/errrr0501/hand-eye-calibration.git

# build the workspace
catkiin_make

# activate the workspace (ie: source it)
source devel/setup.bash
```
## Alternative: Robot arm package
Note: MoveIt has been used on over 126 robots by the community, in this automatical hand-eye-calibration method, 
we chose it to do robot control and motion planning, therefore, user needs to use their own robot and it's MoveIt package.
For example, we use [Universal Robot 5](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) as sample.

## Setting up a robot for hand-eye-calibration

### Prepare the camera also do camera calibration first
Note:In our method we use Realsense camera model and do camera calibration with charuco marker board.
No matter what kind of calibration method just make sure you have Distortion parameter and Intrinsic parameter,
like we put in [directory](https://github.com/errrr0501/hand-eye-calibration/tree/main/charuco_detector/config/camera_calibration).
### Camera calibration
we call Realsense with python, calibrate with charuco marker board, and everything is under FlexBE.
If you want to use, call FlexBE:

```bash
roslaunch flexbe_app flexbe_full.launch
```

Then, press `Load Behavior` on the top, and select `camera_calibration` in left window.

After this, you can press `Runtime Control` on the top, execution window will show:

<center><img src="doc/resource/camera_calibration_execution.png" alt="camera_calibration_execution" style="width: 80%;"/></center>

Before press `Start Execution`, parameter `pic_num` decide how many calibration picture you will take, 
parameter `square_size`, `marker_size` , `col_count` , `row_count` means the spec of charuco marker board we use.
parameter `save_file_name` can let you change your result file name.

### Prepare the robot
Activate robot and open with MoveIt. 

As example using Universal Robot 5:

```bash
roslaunch hand-eye-calibration/charuco_detector/launch/ur5_hand_eye_calibration.launch
```

If you want to change robot, copy and edit this launch file.


#### Quick start
Note:Here we use Realsense D435i+UR5, 
we use whole method with FlexBE and ROS, open camera with ROS first:

```bash
cd ~/<your ros_realsense workspace>
source devel/setup.bash
roslaunch realsense2_camera rs_camera.launch color_width:=1920 color_height:=1080 color_fps:=30 
```
WARN:Here use 1080p as camera resolution, because we use 1080p as camera calibration resolution,
if you want to change resolution, need to do camera calibration again with the resolution you change.

Then launch hand_eye_calibration.lunch, will pop up rviz and FlexBE windows, rviz can check camera view,
FlexBE is for user to change parameter.

```bash
roslaunch charuco_detector ur5_hand_eye_calibration.launch
```
Note: In launch file we can chage parameter `robot_ip` for real robot's ip,
parameter `image_topic` is for recieve camera view with ROS, default is `/camera/color/image_raw`
parameter `camera_info_topic` is for recieve camera info on ROS, default is `/camera/color/camera_info`
parameter `base_link` choose the base coordinate for hand-eye-transform.
parameter `tip_link` choose the end-effector coordinate for hand-eye-transform.
parameter `eye_in_hand_mode` is for hand-eye-transform program know which mode we use.
parameter `customize` is to decide whether want to load result with customize name.
parameter `filename` is the custo,ized result file name.

Then, press `Load Behavior` on the top, and select `Automatic Hand Eye Calibration` in left window.

After this, you can press `Runtime Control` on the top, execution window will show:

<center><img src="doc/resource/auto_hand_eye_calib_execute.png" alt="auto_hand_eye_calib_execute" style="width: 80%;"/></center>

Before press `Start Execution`, parameter `eye_in_hand_mode` decide which calibration mode you want to use, 
parameter `customize_file` , decide whether you want to change save file name or not,
`base_link` , `tip_link` means the base coordinate and end-effector coordinate's name, 
`calibration_file_name` means the name of save file. `move_distance` means the distance of each sample points,
parameter `refernce_frame` means the frame can let you check every link's coordinate.
parameter `group_name` means the move_group in MoveIt that you use for robot (will show in srdf file).
parameter `axis` means the coordinate axis from camera coordinate to base coordinate.(usually eye-in-hand is x-y-z, eye to hand is -yxz)
points_num means how many sample points you want to take.

## Result file

Result will save in `charuco_detector/config/hand_eye_calibration/` .
