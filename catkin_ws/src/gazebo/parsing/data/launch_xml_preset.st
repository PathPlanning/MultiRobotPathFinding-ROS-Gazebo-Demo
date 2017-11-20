 <include file="$(find turtlebot_gazebo)/launch/includes/$(arg base).launch.xml">
     <arg name="x" value=$robot_x$/>
     <arg name="y" value=$robot_y$/>
     <arg name="name" value=$robot_name$/>
     <arg name="base" value="$(arg base)"/>
     <arg name="stacks" value="$(arg stacks)"/>
     <arg name="3d_sensor" value="$(arg 3d_sensor)"/>
 </include>