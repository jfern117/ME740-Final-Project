from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    node_list = []

    #Simulation node
    node_list.append(
        Node(package = 'swarm_gui',
             executable ='talker',
             name = 'sim'
        )
    )

    #gui handler node
    node_list.append(Node(package = 'swarm_gui',
                     executable = 'listener',
                     name = 'gui_handler'
        )
    )

    return LaunchDescription(node_list)