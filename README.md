# task
### Install Dependencies
1) Install ROS 2 Jazzy
Follow the official installation guide:
- https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Make sure you can run:
```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
```
2) Since ROS 2 uses Python 3, please make sure that python3-numpy is installed, or install like this:
```
sudo apt install python3-numpy
```
3) The cv_bridge python backend still has a dependency on python boost (equal or higher than 1.58.0), and install them as follows in Ubuntu:
```
sudo apt install libboost-python-dev
```
4) Install these for excel and csv, in order to save log data
```
sudo apt install python3-pandas
sudo apt install python3-yaml
```
5) Install vision-opencv and cv-bridge
```
sudo apt-get install ros-jazzy-cv-bridge
```
```
 sudo apt-get install ros-jazzy-vision-opencv
```
# Building the project workspace and cloning the repo
1. Build a directory and src folder and then Git clone this workspace
a) source the directory
NOTE: replace {ROS} with your own ros distrubution, 
```
source /opt/ros/{ROS}/setup.bash
```
b) create a new directory
NOTE:
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
1) Open your terminal and divide the terminal into two, one for Publisher and another for subscriber
2) In the first terminal run this code<br>
```
ros2 run planblue_pkg publisher
```
3) From your second terminal run this code line<br>
```
ros2 run planblue_pkg subscriber
```
# Where the output files save
Output files are saved in your home directory under two folders:
1) publisher_captures
2) subscriber_captures
# Working demo
