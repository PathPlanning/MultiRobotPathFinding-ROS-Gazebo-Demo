#!/usr/bin/env python
# coding=utf-8

import os
import xml.etree.ElementTree as et
import stringtemplate3 as st
import csv
import stat
import rospkg
import rospy

head = '<launch>\n  <arg name="world_file"  default= "$(find pathplanning_gazebo)/world/playground.world"/>\n  <arg name="base" value="$(optenv TURTLEBOT_BASE kobuki)"/> <!-- create, roomba --> \n  <arg name="battery"   value="$(optenv TURTLEBOT_BATTERY /proc/acpi/battery/BAT0)"/>  <!-- /proc/acpi/battery/BAT0 -->\n  <arg name="gui" default="true"/>\n  <arg name="stacks"    value="$(optenv TURTLEBOT_STACKS hexagons)"/>  <!-- circles, hexagons -->\n  <arg name="3d_sensor" value="$(optenv TURTLEBOT_3D_SENSOR kinect)"/>  <!-- kinect, asus_xtion_pro -->\n  <include file="$(find gazebo_ros)/launch/empty_world.launch">\n    <arg name="use_sim_time" value="true"/>\n    <arg name="debug" value="false"/>\n    <arg name="gui" value="$(arg gui)" />\n    <arg name="world_name" value="$(arg world_file)"/>\n  </include>\n'
bottom = '  \n  <!-- Fake laser -->\n  <node pkg="nodelet" type="nodelet" name="laserscan_nodelet_manager" args="manager"/>\n  <node pkg="nodelet" type="nodelet" name="depthimage_to_laserscan"\n        args="load depthimage_to_laserscan/DepthImageToLaserScanNodelet laserscan_nodelet_manager">\n    <param name="scan_height" value="10"/>\n    <param name="output_frame_id" value="/camera_depth_frame"/>\n    <param name="range_min" value="0.45"/>\n    <remap from="image" to="/camera/depth/image_raw"/>\n    <remap from="scan" to="/scan"/>\n  </node>\n</launch>'


def addNewBarrier(x, y, z):
    model = subgroup.getInstanceOf('barrier_template')
    model['x'] = str(x * float(cellsize) + float(cellsize) / 2)
    model['y'] = str(y * float(cellsize) + float(cellsize) / 2)
    model['z'] = str((float(z) * float(cellsize)) / 2)
    model['cellsize'] = cellsize
    model['cellsizez'] = str(float(cellsize) * int(z))
    model['model_name'] = 'barrier_' + str(x) + '_' + str(y) + '_' + str(z)
    models.append(model)


# костыль потому, что stringtemplate распознает "$" там, где этого делать не нужно
def addNewRobot(robot_name, x, y):
    namespaceopen = ' <group ns="' + robot_name + '">\n'
    include = '  <include file ="$(find pathplanning_gazebo)/launch/includes/kobuki.launch.xml">\n'
    x = '    <arg name = "x" value ="' + str(int(x) * float(cellsize) + float(cellsize) / 2) + '"/>\n'
    y = '    <arg name = "y" value ="' + str(int(y) * float(cellsize) + float(cellsize) / 2) + '"/>\n'
    name = '    <arg name = "model" value ="' + robot_name + '"/>\n'
    params = '    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">\n    <param name="publish_frequency" type="double" value="30.0" />\n  </node>\n    <arg name = "base" value = "$(arg base)"/> \n    <arg name = "stacks" value = "$(arg stacks)"/>\n    <arg name = "3d_sensor" value = "$(arg 3d_sensor)"/>\n'
    closeinclude = '  </include>'
    namespaceclose = '</group>'
    robot = namespaceopen + include + x + y + name + params + closeinclude + namespaceclose
    robots.append(robot)


# Пробегаем по матрице и создаем препятствия в .world файле, если это требуется
def generate_world():
    y = 0
    for row in root.findall('map/grid/row'):
        columns = row.text.split()
        for x in range(len(columns)):
            if columns[x] == "0":
                continue
            else:
                addNewBarrier(x, y, columns[x])
        y += 1

    preset['models'] = models
    with open(os.path.join(generated_base_path, "world", "playground.world"), "w") as world:
        world.write(preset.toString())
    print('playground.world has been generated successfully')


# получить все команды для всех агентов, записать в .csv
def generate_launch():
    for child in root.findall('log/agent'):
        agentnumber = child.get('number')
        params = []
        for section in child.findall('./path/hplevel/section'):
            section_starty = section.get('start.y')
            section_startx = section.get('start.x')
            section_length = float(section.get('length'))  * cellsize / speed
            section_finishy = section.get('finish.y')
            section_finishx = section.get('finish.x')
            params.append({"startx": section_startx,
                           "starty": section_starty,
                           "finishx": section_finishx,
                           "finishy": section_finishy,
                           "length": section_length})
        startx = params[0]['startx']
        starty = params[0]['starty']
        robot_name = 'robot_' + agentnumber
        addNewRobot(robot_name, startx, starty)
        robot_filename = agentnumber + '.csv'
        with open(os.path.join(commands_path, "src/commands", robot_filename), "w") as f:
            columns = ["startx", "starty", "finishx", "finishy", "length"]
            writer = csv.DictWriter(f, fieldnames=columns)
            writer.writerows(params)

    launch_preset['robots'] = robots

    with open(os.path.join(generated_base_path, "launch", "turtlebot_world.launch"), "w") as launch:
        string = head + '\n' + launch_preset.toString() + '\n' + bottom
        launch.write(string)
    print(
        'turtlebot_world.launch has been generated successfully\n')


# Генерация run_agents.launch для запуска пачки агентов
def generate_robots_launch():
    launch_string_header = '<launch>'
    launch_string_footer = '\n</launch>'
    node_string = ''
    for i in range(len(robots)):
        node_string += '\n  <include file="$(find pathplanning_mover2)/src/launch/includes/agent.launch.xml">\n  <arg name="name" value = "%s"/>\n  <arg name="cellsize" value = "%s"/>\n </include>' % (i, cellsize)
    launch_string_header = launch_string_header + node_string + launch_string_footer
    with open(os.path.join(commands_path, "src/launch/run_agents.launch"), "w") as run:
        run.write(launch_string_header)

    print("run_agents.launch has been generated successfully in \"~/catkin_ws/src/pathplanning_simulation/pathplanning_mover2/src/launch\" directory")

rospy.init_node('pathplanning_generator')
ros_package = rospkg.RosPack()
generator_path = ros_package.get_path("pathplanning_generator")
commands_path = ros_package.get_path("pathplanning_mover2")
generated_base_path = ros_package.get_path("pathplanning_gazebo")
filename = os.path.join(generator_path, "input", (raw_input("type filename :")))

root = et.parse(filename)
path = os.path.join(generator_path, 'data')
group = st.StringTemplateGroup("Supergroup", path)
preset = group.getInstanceOf("preset")

launch_preset_group = st.StringTemplateGroup("LaunchPresetGroup", path)
launch_preset = launch_preset_group.getInstanceOf("launch_preset")
subgroup = st.StringTemplateGroup('subgroup', superGroup=group)
launch_subgroup = st.StringTemplateGroup('launch_subgroup', superGroup=launch_preset_group)

modelsCnt = 0
models = []
robots = []

text = ''
width = int(root.find('map/width').text)
height = int(root.find('map/height').text)
cellsize = float(root.find('map/cellsize').text)
speed = float(root.find('map/speed').text)

agents = root.find('map/agents').text
print ('XML file contains ' + agents + ' agents')

generate_world()

generate_launch()

generate_robots_launch()
