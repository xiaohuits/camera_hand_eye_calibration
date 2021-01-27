# camera_hand_eye_calibration
ROS node for pinhole camera calibration and hand eye calibration.

## Features
Compared with other camera calibration tool (e.g., [camera_calibration](http://wiki.ros.org/camera_calibration))and hand-eye calibration tool (e.g., [easy_handeye](https://github.com/IFL-CAMP/easy_handeye)), this package provides the following features:

* Camera and hand-eye position calibrated together. 
The user only need to move the camera (by commanding the robot) around the target and collect a single set of data. 
The data will be used for both camera calibration and hand-eye calibration.
* Use [ArUco boards](https://docs.opencv.org/master/db/da9/tutorial_aruco_board_detection.html) instead of the traditional chessboards. 
Detecting of AruCo board are much more versatile because it allows occlusions and partial views. 

<a href="http://www.youtube.com/watch?feature=player_embedded&v=toB9m3gMc7s
" target="_blank"><img src="doc/demo.gif" 
alt="demo video" height="480"/></a>

## Install Latest OpenCV
The custom built OpenCV that comes with ROS melodic will not work properly for detecting AruCo board. We need to build latest OpenCV from source. 
```bash
sudo apt update && sudo apt install -y cmake g++ wget unzip
# create a folder for OpenCV
mkdir ~/OpenCV && cd ~/OpenCV
# Download and unpack sources
wget -O opencv.zip https://github.com/opencv/opencv/archive/master.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/master.zip
unzip opencv.zip
unzip opencv_contrib.zip
# Create build directory and switch into it
mkdir -p build && cd build
# Configure
cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-master/modules ../opencv-master
# Build
cmake --build .
```
It might take 20 minutes to build. See [here](https://docs.opencv.org/4.5.1/d7/d9f/tutorial_linux_install.html) if needs more instructions. At the time when I created this repository, the lastest OpenCV version is 4.5.1

## Usage

### 1. prepare the ChArUco Board


### 2. create launch file

```xml
<launch>
    <!-- start your robot driver -->

    <!-- start your camera driver -->

    <!-- start calibration node -->
    <!-- you need to replace the argument value based on your application -->
    <include file="$(find camera_hand_eye_calibration)/launch/launch_calibration.launch">
        <arg name="image_topic"        value="/avt_camera_img"   doc="the image topic name"/>
        <arg name="tf_gripper_name"    value="tool0_controller"  doc="name of the gripper joint in the tf tree"/>
        <arg name="tf_base_name"       value="base"              doc="name of the robot base in the tf tree"/>
        <arg name="chessboard_rows"    value="10"                doc="number of chess board rows"/>
        <arg name="chessboard_columns" value="8"                 doc="number of chessboard columns"/>
        <arg name="square_length"      value="0.02244"           doc="chessboard square length (meter)"/>
        <arg name="marker_length"      value="0.01122"           doc="Aruco marker length (meter)"/>
    </include>
</launch>
```

### 3. collect data


### 4. calibrate

The calibration results will be print to the console, also saved to yaml file "`camera_hand_eye_calibration.yaml`".
You can locate the file in `~/.ros` folder.
```
# This is autogenerated file to store camera and hand-eye calibration results
camera_calibration:
  # camera matrix
  K: [1350.881910478003, 0, 768.9209200452633, 0, 1350.994460435373, 556.015338127424, 0, 0, 1]
  # distortion parameters
  D: [-0.07149669652986015, 0.1038887256802538, -0.0007713606820135701, 0.0009113898706119508, -0.05958660621785733]
hand_eye_position:
  # rotaion matrix
  rotation: [-0.000466712, -0.00562231, -0.00715545, 0.999958]
  # translation
  translation: [-0.001117274244385104, -0.08463651745339704, 0.09354036398990093]
```
the hand eye position is the transformation of camera frame with respect to end-effector frame.
