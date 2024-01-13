import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation

from algorithms_rrt import Graph
import occupancy_grid
from scipy.ndimage import convolve

"""
Useful utils for interacting with pybullet
"""

def vector_rotation_from_Z(v1, v2):
    # Ensure the vectors are normalized
    initial_vector = np.array([0, 0, 1.])
    target_vector = (v2 - v1) / np.linalg.norm(v2 - v1)

    # Calculate the rotation quaternion
    rotation_quaternion = Rotation.align_vectors(target_vector.reshape(1,3), initial_vector.reshape(1,3))[0]
    # print(rotation_quaternion)
    return rotation_quaternion.as_quat()

def diff_to_vertical(p1 : np.array, p2 : np.array, angle_type = "euler"):

    # compute the vector between points

    v2 = p2 - p1

    v1 = np.array([0, 0, 1])
    if angle_type == "euler":
        return euler_angles_between_vectors(v1, v2)
    if angle_type == "quat":
        return quat_between_vectors(v1, v2)
    
def euler_angles_between_vectors(v1, v2):
    # Normalize the input vectors
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)

    # Calculate the rotation matrix that aligns v1 with v2
    rotation_matrix = np.dot(v1.reshape(3, 1), v2.reshape(1, 3))

    # Create a Rotation object from the rotation matrix
    rotation = Rotation.from_matrix(rotation_matrix)
    
    # Extract Euler angles (in radians) from the Rotation object
    euler_angles = rotation.as_euler('xyz', degrees=False)
    return euler_angles

def quat_between_vectors(v1, v2):
    # Normalize the input vectors
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)

    # Calculate the rotation matrix that aligns v1 with v2
    rotation_matrix = np.dot(v1.reshape(3, 1), v2.reshape(1, 3))

    # print(rotation_matrix)

    # Create a Rotation object from the rotation matrix
    rotation = Rotation.from_matrix(rotation_matrix)

    # Extract Quat angles (in radians) from the Rotation object
    quat_angles = rotation.as_quat()

    return quat_angles

def calculate_orientation_angles(v1, v2):
    dot_product = np.dot(v1, v2)
    magnitude_v1 = np.linalg.norm(v1)
    magnitude_v2 = np.linalg.norm(v2)

    cosine_angle = dot_product / (magnitude_v1 * magnitude_v2)
    angle_in_radians = np.arccos(np.clip(cosine_angle, -1.0, 1.0))

    # Calculate the cross product to determine the axis of rotation
    cross_product = np.cross(v1, v2)
    axis_magnitude = np.linalg.norm(cross_product)

    # Normalize the axis vector
    axis = cross_product / axis_magnitude

    # Calculate the roll, pitch, and yaw angles in radians
    roll = np.arctan2(axis[1], axis[2])
    pitch = -np.arctan2(axis[0], np.sqrt(axis[1]**2 + axis[2]**2))
    yaw = np.arctan2(np.sin(angle_in_radians), np.cos(angle_in_radians))

    return [roll, pitch, yaw]

def calculate_orientation_quaternion(v1, v2):
    dot_product = np.dot(v1, v2)
    magnitude_v1 = np.linalg.norm(v1)
    magnitude_v2 = np.linalg.norm(v2)

    cosine_angle = dot_product / (magnitude_v1 * magnitude_v2)
    angle_in_radians = np.arccos(np.clip(cosine_angle, -1.0, 1.0))

    cross_product = np.cross(v1, v2)
    axis_magnitude = np.linalg.norm(cross_product)

    axis = cross_product / axis_magnitude

    quaternion = np.array([np.cos(angle_in_radians / 2.0), np.sin(angle_in_radians / 2.0) * axis[0],
                          np.sin(angle_in_radians / 2.0) * axis[1], np.sin(angle_in_radians / 2.0) * axis[2]])

    return quaternion

def dataEdges(graph : Graph):
    """
    Return pos_list, length_list, quat_list based on the edges connection
    """
    pos_list = []
    lengths_list = []
    quat_list = []

    for edge in graph.edgeArray:
        # compute midpoints
        e = np.array(edge)

        x_pos = np.sum(np.array(e[0,:])) / 2.
        y_pos = np.sum(np.array(e[1,:])) / 2.
        z_pos = np.sum(np.array(e[2,:])) / 2.

        pos_list.append([x_pos, y_pos, z_pos])
        lengths_list.append(np.sqrt(np.sum((e[:,0] - e[:,1])**2)))

        quat = vector_rotation_from_Z(e[:,0], e[:,1])
        quat_list.append(quat)

    return  pos_list, lengths_list, quat_list

def dataHierarchy(graph : Graph):
    """
    Return pos_list, length_list, quat_list based on the parent-children connection
    """
    pos_list = []
    lengths_list = []
    quat_list = []

    # for every node but the root
    for n in graph.nodeArray[1:]:

        
        e = np.array([n.parent.pos, n.pos]).T

        x_pos = np.sum(np.array(e[0,:])) / 2.
        y_pos = np.sum(np.array(e[1,:])) / 2.
        z_pos = np.sum(np.array(e[2,:])) / 2.

        pos_list.append([x_pos, y_pos, z_pos])
        lengths_list.append(np.sqrt(np.sum((e[:,0] - e[:,1])**2)))

        quat = vector_rotation_from_Z(e[:,0], e[:,1])
        quat_list.append(quat)

    return  pos_list, lengths_list, quat_list


    

def plotGraph(graph: Graph, path=None, rgba=[0.,0.,0.,0.5], rgba_path=[1., 0., 0.,0.8], plotting="parent"):
    """
    Creates visual cylinders for each edge in the graph.
    If path is given, it is drawn in the rgba_path color.

    As this visualization is for a 3D env in pybullet, assume 3D coordinates in graph

    Plotting method can be:
        - "parent": for each node draw a connection with its parent
        - "edges": for each edge between 2 nodes, plot it 
    """

    pos_list = []
    lengths_list = []
    # rpy_list = []
    # euler_list = []
    quat_list = []

    if plotting == "parent":
        pos_list, lengths_list, quat_list = dataHierarchy(graph)
    elif plotting == "edges":
        pos_list, lengths_list, quat_list = dataEdges(graph)
    else:
        raise ValueError(f"plotting value should be [\"edges\", \"parent\"], but is {plotting}")
    

    bodiesId = []
    for i in range(len(pos_list)):
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_CYLINDER,
                                                radius=0.01,
                                                length=lengths_list[i],
                                                rgbaColor=rgba)

        bodyId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=visualShapeId,
                                 basePosition=pos_list[i], baseOrientation=quat_list[i])
        
        # print("rpy:", rpy_list[i])
        bodiesId.append(bodyId)

    return bodiesId

def plotPointsPath(points: list, rgba = [0.,0.,0.,0.5]):
    """
    points is a list of 3D points as np.array
    """

    pos_list = []
    lengths_list = []
    # rpy_list = []
    # euler_list = []
    quat_list = []
    for i in range(len(points)-1):
        # create edge from 2 points
        e = np.array([points[i], points[i+1]]).T
        print(e.shape)
        # compute midpoints
        x_pos = np.sum(np.array(e[0,:])) / 2.
        y_pos = np.sum(np.array(e[1,:])) / 2.
        z_pos = np.sum(np.array(e[2,:])) / 2.

        pos_list.append([x_pos, y_pos, z_pos])
        lengths_list.append(np.sqrt(np.sum((e[:,0] - e[:,1])**2)))



        quat = vector_rotation_from_Z(e[:,0], e[:,1])
        quat_list.append(quat)


    bodiesId = []
    for i in range(len(pos_list)):
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_CYLINDER,
                                                radius=0.01,
                                                length=lengths_list[i],
                                                rgbaColor=rgba)

        bodyId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=visualShapeId,
                                basePosition=pos_list[i], baseOrientation=quat_list[i])
        
        # print("rpy:", rpy_list[i])
        bodiesId.append(bodyId)

    return bodiesId

def inflate_obstacles_3d(grid, inflation_size):
    # Create a 3D structuring element (kernel)
    kernel = np.ones((inflation_size, inflation_size, inflation_size), dtype=int)

    # Use 3D convolution to inflate the obstacles
    print(grid.shape, kernel.shape)
    inflated_grid = convolve(grid, weights=kernel, mode='constant', cval=0)


    return inflated_grid
