import numpy as np
from std_msgs.msg import Float64MultiArray, MultiArrayDimension


from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from pathlib import Path

#copied for simplicity
def msg_to_array(msg:Float64MultiArray):
    """
    """

    row = msg.layout.dim[0].size
    col = msg.layout.dim[1].size

    array = np.array(msg.data).reshape((row, col))

    return array

#source: https://ternaris.gitlab.io/rosbags/

bag_path = Path("gui_test_state_record_04-26-2025")

typestore = get_typestore(Stores.ROS2_HUMBLE)

topic_name = "/agent_states"
msg_list = []
with AnyReader([bag_path], default_typestore=typestore) as reader:
    connections = [x for x in reader.connections if x.topic == topic_name]

    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        msg_list.append(msg)


num_agents = 5
state_dim = 4
data = np.zeros((len(msg_list), num_agents, state_dim))

for msg_idx in range(len(msg_list)):
    msg = msg_list[msg_idx]
    array = msg_to_array(msg)
    data[msg_idx] = array

np.save("agent_states.npy", data)