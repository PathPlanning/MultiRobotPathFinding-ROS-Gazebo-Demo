<sdf version="1.4">
    <world name='default'>
        <!-- A global light source -->
        <include>
            <uri>model://sun</uri>
        </include>
        <!-- A ground plane -->
        <include>
            <uri>model://ground_plane</uri>
        </include>
        <!-- Own physics settings to speed up simulation -->
        <physics type="ode">
            <max_step_size>0.01</max_step_size>
            <real_time_factor>1</real_time_factor>
            <real_time_update_rate>100</real_time_update_rate>
            <gravity>0 0 -9.8</gravity>
        </physics>

                <!-- visual and collisions here-->
                $if(models)$ $models: {u|$u$};separator="\n         "$
                $endif$
        <model name='ground_plane'>
            <static>1</static>
            <link name='link'>
                <collision name='collision'>
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>100 100</size>
                        </plane>
                    </geometry>
                    <surface>
                    <friction>
                        <ode>
                            <mu>100</mu>
                        <mu2>50</mu2>
                        </ode>
                    </friction>
                    <bounce/>
                    <contact>
                        <ode/>
                    </contact>
                    </surface>
                    <max_contacts>10</max_contacts>
                </collision>
                <visual name='visual'>
                    <cast_shadows>0</cast_shadows>
                        <geometry>
                            <plane>
                                <normal>0 0 1</normal>
                                <size>100 100</size>
                            </plane>
                        </geometry>
                        <material>
                            <script>
                                <uri>file://media/materials/scripts/gazebo.material</uri>
                                     <name>Gazebo/Grey</name>
                            </script>
                        </material>
                </visual>
                <velocity_decay>
                    <linear>0</linear>
                    <angular>0</angular>
                </velocity_decay>
                <self_collide>0</self_collide>
                <kinematic>0</kinematic>
                <gravity>1</gravity>
            </link>
            <pose>0.497681 0 0 0 -0 0</pose>
        </model>
    </world>
</sdf>