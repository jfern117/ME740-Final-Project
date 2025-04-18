from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    node_list = []

    #get the gazebo simulation launch file
    gazebo_sim_package_dir = get_package_share_directory('tb_sim_custom')

    gazebo_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_sim_package_dir,
                'launch',
                'gazebo_tb_sim.launch.py'
            )
        )
    )

    interface_node = Node(package='swarm_gui',
                          executable='gazebo_interface',
                          name = 'gazebo_interface')
    

    #gui handler node
    gui_node = Node(package = 'swarm_gui',
                    executable = 'gui',
                    name = 'gui_handler'
        )

    #keyboard control node
    control_node = Node(package = 'swarm_gui',
                        executable = 'controller',
                        name = 'controller_input_handler'
    )

    #autnomous control node
    auto_ctl_node = Node(package='swarm_gui',
                         executable='autonmous_control',
                         name="follower_controller")
    
    node_list.append(gazebo_launch_description)
    node_list.append(interface_node)
    node_list.append(gui_node)
    node_list.append(control_node)
    node_list.append(auto_ctl_node)


    return LaunchDescription(node_list)

