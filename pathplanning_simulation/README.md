# pathplanning_simulation

# Prerequisites

Antlr

```bash
sudo apt install python-antlr
```

TODO: Complete

# Compile

```bash
cd {path_to_your_catkin_workspace}/src
git clone https://github.com/avbokovoy/pathplanning_simulation.git
cd ..
catkin_make
```

# Run

1) Generate world file (doesn't work atm)

```bash
rosrun pathplanning_generator world_generator.py
```

2) Run simulation 

```bash
roslaunch pathplanning_gazebo turtlebot_world.launch
```

3) Run pathplanning_mover2 (work in progress)

```bash
rosrun pathplanning_mover2 ...
```
