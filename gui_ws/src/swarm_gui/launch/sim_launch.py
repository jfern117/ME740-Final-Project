from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    node_list = []

    #Simulation node
    node_list.append(
        Node(package = 'swarm_gui',
             executable ='sim',
             name = 'sim_handler'
        )
    )

    #gui handler node
    node_list.append(Node(package = 'swarm_gui',
                     executable = 'gui',
                     name = 'gui_handler'
        )
    )

    #keyboard control node
    node_list.append(Node(package = 'swarm_gui',
                          executable = 'controller',
                          name = 'controller_input_handler'
    ))

    return LaunchDescription(node_list)