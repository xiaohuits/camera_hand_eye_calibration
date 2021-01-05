# camera_hand_eye_calibration
ROS node for pinhole camera calibration and hand eye calibration.

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