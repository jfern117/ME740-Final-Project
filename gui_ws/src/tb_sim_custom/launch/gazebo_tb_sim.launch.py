#!/usr/bin/env python3
#
# Based on modified code from Arshad Mehmood, details can be found at https://medium.com/@arshad.mehmood/efficient-deployment-and-operation-of-multiple-turtlebot3-robots-in-gazebos-f72f6a364620
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition


import numpy as np


def generate_launch_description():
    ld = LaunchDescription()

    #set some common arguments
    TURTLEBOT3_MODEL = "waffle_pi"

    enable_drive = LaunchConfiguration("enable_drive", default="true")
    declare_enable_drive = DeclareLaunchArgument(
        name="enable_drive", default_value="true", description="Enable robot drive node"
    )

    #create a consistent reference to our package name
    package_name = "tb_sim_custom"


    sim_pkg_name = get_package_share_directory(package_name)
    launch_file_dir = os.path.join(sim_pkg_name, "launch")

    world = os.path.join(
        sim_pkg_name, "worlds", "empty_world.world"
    )

    urdf_file_name = "turtlebot3_" + TURTLEBOT3_MODEL + ".urdf"
    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        sim_pkg_name, "urdf", urdf_file_name
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    #testing if this is what launches the GUI
    # gzclient_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
    #     ),
    # )

    ld.add_action(declare_enable_drive)
    ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)


    #significant variations from the source from here on
    
    """
    For reference, here's our starting positions. TODO: store this in a more centralized location eventually (life is hard)
     starting_offset = 0.5*np.sqrt(2)/2
        agent_states = np.zeros((self.num_agents, self.state_dim))
        agent_states[0] = np.array([0, 0, 0, 0]) #add initial velocity to demo leader follower setup while working on teleop
        agent_states[1] = np.array([0.5, 0, 0, 0])
        agent_states[2] = np.array([starting_offset, -starting_offset, 0, 0])
        agent_states[3] = np.array([0, -0.5, 0, 0])
        agent_states[4] = np.array([-starting_offset, -starting_offset, 0, 0])
    """
    # ROWS = 5
    # COLS = 5

    # x = -ROWS
    # y = -COLS
    last_action = None


    num_agents = 5
    state_dim = 4
    starting_offset = 0.5*np.sqrt(2)/2
    agent_states = np.zeros((num_agents, state_dim))
    agent_states[0] = np.array([0, 0, 0, 0]) #add initial velocity to demo leader follower setup while working on teleop
    agent_states[1] = np.array([0.5, 0, 0, 0])
    agent_states[2] = np.array([starting_offset, -starting_offset, 0, 0])
    agent_states[3] = np.array([0, -0.5, 0, 0])
    agent_states[4] = np.array([-starting_offset, -starting_offset, 0, 0])

    # Remapping is required for state publisher otherwise /tf and /tf_static will get be published on root '/' namespace
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    for agent_idx in range(num_agents):

        #give each agent a unique name and namespace
        name = f"tb{agent_idx}"
        namespace = f"/agent{agent_idx}"

        #create a state publisher node for this agent instance
        turtlebot_state_publisher = Node(
            package = "robot_state_publisher", #TODO: figure out how to get this dependency for recreatability
            namespace = namespace, 
            executable = "robot_state_publisher",
            output = "screen",
            parameters = [{"use_sim_time":False,
                           "publish_frequency": 30.0}],
            remappings = remappings,
            arguments = [urdf],
        )
        
        #create the call to spawn the agent
        spawn_turtlebot3_burger = Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-file", os.path.join(sim_pkg_name,'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
                    "-entity", name,
                    "-robot_namespace", namespace,
                    "-x", str(agent_states[agent_idx][0]),
                    "-y", str(agent_states[agent_idx][1]),
                    "-z", "0.01",
                    "-Y", "3.14159",
                    "-unpause",
                ],
                output="screen",
            )


        if last_action is None:
                # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
                ld.add_action(turtlebot_state_publisher)
                ld.add_action(spawn_turtlebot3_burger)
                
        else:
            # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
            # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
            spawn_turtlebot3_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[spawn_turtlebot3_burger,
                                turtlebot_state_publisher],
                )
            )
            ld.add_action(spawn_turtlebot3_event)

        # Save last instance for next RegisterEventHandler
        last_action = spawn_turtlebot3_burger


    #This might be an autopilot
    # # Start all driving nodes after the last robot is spawned
    # for agent_idx in range(num_agents):

    #     namespace = f"/agent{agent_idx}"

    #     # Create spawn call
    #     drive_turtlebot3_burger = Node(
    #         package="turtlebot3_gazebo",
    #         executable="turtlebot3_drive",
    #         namespace=namespace,
    #         output="screen",
    #         condition=IfCondition(enable_drive),
    #     )

    #     # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
    #     # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
    #     drive_turtlebot3_event = RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=last_action,
    #             on_exit=[drive_turtlebot3_burger],
    #         )
    #     )
        
    #     ld.add_action(drive_turtlebot3_event)

    return ld



    # # Spawn turtlebot3 instances in gazebo
    # for i in range(COLS):
    #     x = -ROWS
    #     for j in range(ROWS):
    #         # Construct a unique name and namespace
    #         name = "turtlebot" + str(i) + "_" + str(j)
    #         namespace = "/tb" + str(i) + "_" + str(j)

    #         # Create state publisher node for that instance
    #         turtlebot_state_publisher = Node(
    #             package="robot_state_publisher",
    #             namespace=namespace,
    #             executable="robot_state_publisher",
    #             output="screen",
    #             parameters=[{"use_sim_time": False,
    #                          "publish_frequency": 10.0}],
    #             remappings=remappings,
    #             arguments=[urdf],
    #         )

    #         # Create spawn call
    #         spawn_turtlebot3_burger = Node(
    #             package="gazebo_ros",
    #             executable="spawn_entity.py",
    #             arguments=[
    #                 "-file",
    #                 os.path.join(sim_pkg_name,'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
    #                 "-entity",
    #                 name,
    #                 "-robot_namespace",
    #                 namespace,
    #                 "-x",
    #                 str(x),
    #                 "-y",
    #                 str(y),
    #                 "-z",
    #                 "0.01",
    #                 "-Y",
    #                 "3.14159",
    #                 "-unpause",
    #             ],
    #             output="screen",
    #         )

    #         # Advance by 2 meter in x direction for next robot instantiation
    #         x += 2.0

    #         if last_action is None:
    #             # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
    #             ld.add_action(turtlebot_state_publisher)
    #             ld.add_action(spawn_turtlebot3_burger)
                
    #         else:
    #             # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
    #             # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
    #             spawn_turtlebot3_event = RegisterEventHandler(
    #                 event_handler=OnProcessExit(
    #                     target_action=last_action,
    #                     on_exit=[spawn_turtlebot3_burger,
    #                              turtlebot_state_publisher],
    #                 )
    #             )
    #             ld.add_action(spawn_turtlebot3_event)

    #         # Save last instance for next RegisterEventHandler
    #         last_action = spawn_turtlebot3_burger

    #     # Advance by 2 meter in y direction for next robot instantiation
    #     y += 2.0

    # # Start all driving nodes after the last robot is spawned
    # for i in range(COLS):
    #     for j in range(ROWS):
    #         namespace = "/tb" + str(i) + "_" + str(j)
    #         # Create spawn call
    #         drive_turtlebot3_burger = Node(
    #             package="turtlebot3_gazebo",
    #             executable="turtlebot3_drive",
    #             namespace=namespace,
    #             output="screen",
    #             condition=IfCondition(enable_drive),
    #         )

    #         # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
    #         # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
    #         drive_turtlebot3_event = RegisterEventHandler(
    #             event_handler=OnProcessExit(
    #                 target_action=last_action,
    #                 on_exit=[drive_turtlebot3_burger],
    #             )
    #         )
            
    #         ld.add_action(drive_turtlebot3_event)

    # return ld
