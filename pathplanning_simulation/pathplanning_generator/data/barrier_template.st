    <model name='$model_name$'>
      <pose> $x$ $y$ $z$ 0 -0 0</pose>
        <static>1</static>
        <link name='link'>
        <collision name='collision'>
          <geometry>
            <box>
              <size>$cellsize$ $cellsize$ $cellsizez$</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>$cellsize$ $cellsize$ $cellsizez$</size>
            </box>
          </geometry>
            <material>
              <ambient>1 0 0 1</ambient>
              <diffuse>1 0 0 1</diffuse>
              <specular>0.1 0.1 0.1 1</specular>
              <emissive>0 0 0 0</emissive>
            </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
      </link>
    </model>