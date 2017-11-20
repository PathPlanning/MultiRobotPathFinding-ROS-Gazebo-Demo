# coding=utf-8

import os
import xml.etree.ElementTree as et
import stringtemplate3 as st
import csv
import stat

head = '<launch>\n  <arg name="world_file"  default= "$(env TURTLEBOT_GAZEBO_WORLD_FILE)"/>\n  <arg name="base" value="$(optenv TURTLEBOT_BASE kobuki)"/> <!-- create, roomba --> \n  <arg name="battery"   value="$(optenv TURTLEBOT_BATTERY /proc/acpi/battery/BAT0)"/>  <!-- /proc/acpi/battery/BAT0 -->\n  <arg name="gui" default="true"/>\n  <arg name="stacks"    value="$(optenv TURTLEBOT_STACKS hexagons)"/>  <!-- circles, hexagons -->\n  <arg name="3d_sensor" value="$(optenv TURTLEBOT_3D_SENSOR kinect)"/>  <!-- kinect, asus_xtion_pro -->\n  <include file="$(find gazebo_ros)/launch/empty_world.launch">\n    <arg name="use_sim_time" value="true"/>\n    <arg name="debug" value="false"/>\n    <arg name="gui" value="$(arg gui)" />\n    <arg name="world_name" value="$(arg world_file)"/>\n  </include>\n'
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


def addNewRobot(robot_name, x, y):
    namespaceopen = ' <group ns="' + robot_name + '">\n'
    include = '  <include file ="$(find gazebo)/launch/includes/kobuki.launch.xml">\n'
    x = '    <arg name = "x" value ="'+ str(int(x)*float(cellsize) + float(cellsize)/2) + '"/>\n'
    y = '    <arg name = "y" value ="' + str(int(y)*float(cellsize) + float(cellsize)/2) +'"/>\n'
    name = '    <arg name = "model" value ="' + robot_name + '"/>\n'
    params = '    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">\n    <param name="publish_frequency" type="double" value="30.0" />\n  </node>\n    <arg name = "base" value = "$(arg base)"/> \n    <arg name = "stacks" value = "$(arg stacks)"/>\n    <arg name = "3d_sensor" value = "$(arg 3d_sensor)"/>\n'
    closeinclude = '  </include>'
    namespaceclose = '</group>'
    robot = namespaceopen + include + x + y + name + params + closeinclude + namespaceclose
    robots.append(robot)


# Пробегаем по матрице и создаем препятствия в .world файле, если это требуется
def get_grid():
    y = 0
    for row in root.findall('map/grid/row'):
        columns = row.text.split()
        for x in range(len(columns)):
            if columns[x] == "0":
                continue
            else:
                addNewBarrier(x, y, columns[x])
        y = y + 1


# получить все команды для всех агентов, записать в .csv
def get_all_commands_for_all_agents():
    for child in root.findall('log/agent'):
        agentnumber = child.get('number')
        params = []
        for section in child.findall('./path/hplevel/section'):
            # section_number = section.tag + ' number ' + section.get('number')

            section_starty = section.get('start.y')
            section_startx = section.get('start.x')
            section_length = section.get('length')
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
        with open(os.path.join(path_to_input, "gazebo/commands",robot_filename), "w") as f:
            columns = ["startx", "starty", "finishx", "finishy", "length"]
            writer = csv.DictWriter(f, fieldnames=columns)
            writer.writerows(params)


def generate_sh():
    sh_string = ''
    for i in range(len(robots)):
        sh_string += "rosrun mover2 mover_clbk.py " + str(i) + "\n"

    with open(os.path.join(path_to_input, "run_robots.sh"), "w") as run:
        run.write(sh_string)
        st=os.stat(path_to_input+"/run_agents.sh")
        os.chmod(path_to_input+"/run_agents.sh", st.st_mode|stat.S_IEXEC)

    print("run_robots.sh has been generated successfully in \"~/catkin_ws/src\" directory")

base_path = os.path.abspath("catkin_ws/src/gazebo")
path_to_input = os.path.dirname(base_path)
generator_path = os.path.dirname(os.path.realpath(__file__))
filename = os.path.join(path_to_input,"gazebo/input", (raw_input("type filename :")))

xml_file = ""
try:
    xml_file = os.path.join(base_path, filename)
except NameError as e:
    print (e.message)

root = et.parse(xml_file)
path = os.path.join(base_path, 'parsing/data')
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

get_all_commands_for_all_agents()

# print ('width: ' + width)
# print ('height: ' + height)

agents = root.find('map/agents').text
print ('XML file contains ' + agents + ' agents')

get_grid()

preset['models'] = models
launch_preset['robots'] = robots
with open(os.path.join(path_to_input, "gazebo/world", "playground.world"), "w") as tf:
    tf.write(preset.toString())
print('playground.world has been generated successfully')

with open(os.path.join(path_to_input, "gazebo/launch", "turtlebot_world.launch"), "w") as world:
    string = head + '\n' + launch_preset.toString() + '\n' + bottom
    world.write(string)
print('turtlebot_world.launch has been generated successfully\nNow you can run it by \"./run_robots.sh\"\n do not forget make it readable by chmod u+x!')

generate_sh()
