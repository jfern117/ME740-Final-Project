import numpy as np
from std_msgs.msg import Float64MultiArray, MultiArrayDimension


#numpy array to float64 multiarray helper function
def array_to_msg(array):
    """
    """

    #add the data to the message
    msg = Float64MultiArray()
    msg.data = array.flatten().tolist()

    #add the row/col information to the message
    #This is confusing, but more details can be found by recursively following the message types in 
    #The documentation: https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64MultiArray.html
    row, col = np.shape(array)
    dim1 = MultiArrayDimension(label = "rows", size = row, stride = row*col)
    dim2 = MultiArrayDimension(label = "cols", size = col, stride = col)

    msg.layout.dim.append(dim1)
    msg.layout.dim.append(dim2)

    return msg

def msg_to_array(msg:Float64MultiArray):
    """
    """

    row = msg.layout.dim[0].size
    col = msg.layout.dim[1].size

    array = np.array(msg.data).reshape((row, col))

    return array



