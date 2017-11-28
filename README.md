**pathplanning_simulation**
================================================================================================================================
# Installing prerequisites
	First of all, you can use the install_ros_gazebo.sh. In that case you have to download it separately, skip 3 upper steps and skip installing required python libraries.

### 1. Install ROS Kinetic

```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
```bash
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```
```bash
$ sudo apt-get update
```
```bash
$ sudo apt-get install ros-kinetic-desktop-full
```
```bash
$ sudo rosdep init
$ rosdep update
```
```bash
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

### 2. Install Gazebo

```bash
$ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
```

### 3. Install turtlebot packages

```bash
$ sudo apt install ros-kinetic-turtlebot
```


### 4. Initialize catkin workspace

```bash
$ cd {custom_path}/catkin_ws/src
$ catkin_init_workspace
$ cd ..
$ mkdir -p {custom_path}/catkin_ws/src
$ cd {custom_path}/catkin_ws/
$ catkin_make
```
```bash
$ echo "source {path_to_catkin_ws}/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```


# Setting up pathplanning_simulation prerequisites

### stringtemplate3

```bash
$ sudo apt install python-stringtemplate3
```

### Antlr

```bash
$ sudo apt install python-antlr
```

# Compile pathplanning_simulation

### Optional: Install git

```bash
$ sudo apt install git
```

### Clone and compile repository

```bash
$ cd {path_to_your_catkin_workspace}/src
$ git clone https://github.com/PathPlanning/MultiRobotPathFinding-ROS-Gazebo-Demo.git
$ cd ..
$ catkin_make
```

# Run

1) Generate world and launch files

```bash
$ roslaunch pathplanning_generator generate_world.launch
$ Ctrl+c
```

2) Run simulation 

```bash
$ roslaunch pathplanning_gazebo turtlebot_world.launch
```

3)

```bash
$ roslaunch pathplanning_mover2 run_agents.launch
```
