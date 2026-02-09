# Task
### NOTE: This repo is explicitly for ros2 jazzy jalisco + ubuntu 24.04.<br>
If you want to use this repo, it is recommended to use ros2 jalisco + ubuntu 24.04. <br>
For other ros2 users, you can install ros2 jazzy via docker, follow this installation guide: [link](https://docs.ros.org/en/jazzy/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html) <br>
for windows users, it is recommended to follow this installation guide: [link](https://docs.ros.org/en/jazzy/Installation/Alternatives/Windows-Development-Setup.html) <br>
for macOS users, it is recommended to follow this installation guide: [link](https://docs.ros.org/en/jazzy/Installation/Alternatives/macOS-Development-Setup.html) <br>

### Requirements
Tech: ROS2(Jazzy or any other) <br>
OS: Ubuntu/Linux <br>
Laptop with cam or webcam <br>
### Install ROS2 
1) Install ROS 2 Jazzy Jalisco<br>
Follow the official installation guide: [link](https://docs.ros.org/en/jazzy/Tutorials.html)<br>
Make sure you can run:
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
1. Build a directory and src folder and then Git clone this workspace <br>
a) source the directory<br>
```
source /opt/ros/jazzy/setup.bash
```
b) create a new directory
```
mkdir -p ~/task/src
cd ~/task/src
```
c) Clone this repo
```
git clone https://github.com/RB0609/task.git
```
d) Resolve dependencies
```
cd ~/task
rosdep install --from-paths src -i -y
```
e) Build and source the workspace
```
cd ~/task
colcon build --symlink-install
source install/setup.bash
```
# How to run this system
1) open the new terminal and divide the terminal into two, one for Publisher and another for subscriber<br>
2) In the first terminal run this code<br>
```
cd ~/task
colcon build --symlink-install
source install/setup.bash
```
below is the command line, which we have to run after building and sourcing our workspace<br>
```
ros2 run planblue_pkg publisher
```
3) From your second terminal run this code line<br>
```
cd ~/task
colcon build --symlink-install
source install/setup.bash
```
Below is the commad line, which we have to run after building and sourcing our workspace<br>
```
ros2 run planblue_pkg subscriber
```
# Where the output files save
Output files are saved in your home directory under two folders:
1) publisher_captures
2) subscriber_captures
# Working Demo 


https://github.com/user-attachments/assets/73512c38-1c55-40bb-9a4c-031167cdc742








