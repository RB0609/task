# Task: This repo contains the code of two nodes that communicate with each other to capture and save images with timestamp
### NOTE: This repo is explicitly for ros2 jazzy jalisco + ubuntu 24.04.<br>
If you want to use this repo, it is recommended to install ros2 jalisco + ubuntu 24.04, follow this installation guide: [link](https://docs.ros.org/en/jazzy/Tutorials.html)<br>
For other ros2 distro users, you can install ros2 jazzy via docker, follow this installation guide: [link](https://docs.ros.org/en/jazzy/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html) <br>
for windows users, it is recommended to follow this installation guide: [link](https://docs.ros.org/en/jazzy/Installation/Alternatives/Windows-Development-Setup.html) <br>
for macOS users, it is recommended to follow this installation guide: [link](https://docs.ros.org/en/jazzy/Installation/Alternatives/macOS-Development-Setup.html) <br>

### System Requirements
1. ROS2 Jazzy Jalisco <br>
2. Ubuntu 24.04 <br>
3. Laptop with camera or webcam <br>
### Install ROS2 
1) After installing, Make sure you can run this:
```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
```
### Install the dependencies
1) Since ROS 2 uses Python 3, please make sure that python3-numpy is installed, or install like this:
```
sudo apt install python3-numpy
```
2) The cv_bridge python backend still has a dependency on python boost (equal or higher than 1.58.0), and install them as follows in Ubuntu:
```
sudo apt install libboost-python-dev
```
3) Install these for excel and csv, in order to save log data
```
sudo apt install python3-pandas
sudo apt install python3-yaml
```
4) Install vision-opencv and cv-bridge
```
sudo apt-get install ros-jazzy-cv-bridge
```
```
sudo apt-get install ros-jazzy-vision-opencv
```
# Building the project workspace and cloning the repo
a) source the directory<br>
```
source /opt/ros/jazzy/setup.bash
```
b) Clone this repo
```
git clone https://github.com/RB0609/task.git
```
c) Resolve dependencies
```
cd ~/task
rosdep install --from-paths src -i -y
```
d) Build and source the workspace
```
cd ~/task
colcon build --symlink-install
source install/setup.bash
```
# How to run this system
1) open a new terminal and divide the terminal into two, one for Publisher and another for subscriber<br>
2) In the first terminal run this code. Ignore, if you've done it already<br>
```
cd ~/task
colcon build --symlink-install
source install/setup.bash
```
below is the command line for publisher node, which we've to run after building and sourcing workspace<br>
```
ros2 run planblue_pkg publisher
```
3) From your second terminal run this code line<br>
```
cd ~/task
colcon build --symlink-install
source install/setup.bash
```
Below is the command line, which we've to run after building and sourcing workspace<br>
```
ros2 run planblue_pkg subscriber
```
# Where the output files save
Output files are saved in your home directory with these folder names:
1) publisher_captures
2) subscriber_captures
# Working Demo 


https://github.com/user-attachments/assets/73512c38-1c55-40bb-9a4c-031167cdc742








