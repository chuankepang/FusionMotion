# FusionMotion
FusionMotion is a multi-sensory fusion algorithm integrating vision, touch, and motion control for enhanced robotic task execution. It has been validated on real-world platforms, including the Rokae robotic arm and Inspire Hand, demonstrating efficient and precise performance in physical experiments.

## realsense-ros installation
### realsense SDK
1. SDK Download

```
git clone https://github.com/IntelRealSense/librealsense.git
```

2. Dependency Installation

```
sudo apt-get install libudev-dev pkg-config libgtk-3-dev
sudo apt-get install libusb-1.0-0-dev pkg-config
sudo apt-get install libglfw3-dev
sudo apt-get install libssl-dev
```

3. Permission Scripts Installation

```
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger 
```

4. Make

```
mkdir build
cd build
cmake ../ -DBUILD_EXAMPLES=true
make -j4
sudo make install

```

5. Usage

```
realsense-viewer
```

### realsense-ros

1. realsense-ros installation

```
sudo apt install ros-<ROS_DISTRO>-realsense2-*
```

2. realsense start

```
roslaunch realsense2_camera rs_camera.launch
```