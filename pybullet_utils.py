import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation

from rrt import Graph
import occupancy_grid
"""
Useful utils for interacting with pybullet
"""

def vector_rotation_from_Z(v1, v2):
    # Ensure the vectors are normalized
    initial_vector = np.array([0, 0, 1.])
    target_vector = (v2 - v1) / np.linalg.norm(v2 - v1)

    # Calculate the rotation quaternion
    rotation_quaternion = Rotation.align_vectors(target_vector.reshape(1,3), initial_vector.reshape(1,3))[0]
    print(rotation_quaternion)
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

    print(rotation_matrix)

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

def plotGraph(graph: Graph, path=None, rgba=[0.,0.,0.,0.5], rgba_path=[1., 0., 0.,0.8]):
    """
    Creates visual cylinders for each edge in the graph.
    If path is given, it is drawn in the rgba_path color.

    As this visualization is for a 3D env in pybullet, assume 3D coordinates in graph
    """

    pos_list = []
    lengths_list = []
    rpy_list = []
    euler_list = []
    quat_list = []
    for edge in graph.edgeArray:
        # compute midpoints
        e = np.array(edge)

        x_pos = np.sum(np.array(e[0,:])) / 2.
        y_pos = np.sum(np.array(e[1,:])) / 2.
        z_pos = np.sum(np.array(e[2,:])) / 2.

        pos_list.append([x_pos, y_pos, z_pos])
        lengths_list.append(np.sqrt(np.sum((e[:,0] - e[:,1])**2)))

        # rpy = calculate_orientation_angles( e[:,1] - e[:,0], np.array([0.,0.,1.]))
        # rpy_list.append(rpy)

        # quat = calculate_orientation_quaternion(e[:,0], e[:, 1])
        # quat_list.append(quat)
        # euler_ang = diff_to_vertical(e[:,0], e[:,1])

        # euler_list.append(euler_ang)


        quat = vector_rotation_from_Z(e[:,0], e[:,1])
        quat_list.append(quat)

        # quat = diff_to_vertical(e[:,0], e[:,1], angle_type="quat")
        # quat_list.append(quat)

    print("len pos_list:", len(pos_list))
    print("len lengths_list:", len(lengths_list))
    print("len rpy_list:", len(rpy_list))
    print("len quat_list:", len(quat_list))

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

def createWall(pos, width, height, depth, min_bound = [-2., -2, 0], max_bound = [2., 2., 2.], step=0.2):
    
    wallsID = []
    
    rgba=[0.,0.,0.,0.5]
    
    visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[width/2, height/2, depth/2], rgbaColor=rgba)
    
    collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[width/2, height/2, depth/2])
    
    bodyId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collisionShapeId, baseVisualShapeIndex=visualShapeId,
                             basePosition=pos, baseOrientation=[0,0,0,1])
    
    
    origin = min_bound 
    shape = np.array(max_bound) - np.array(min_bound)

    xgoal = np.arange(pos[0], pos[0]+width, step)
    ygoal = np.arange(pos[1], pos[1]+height, step)
    zgoal = np.arange(pos[2], pos[2]+depth, step)
    
    my_occ_grid = occupancy_grid.OccGrid3D(shape, origin, step)
    
    
    for i in xgoal:
        for j in ygoal:
            for k in zgoal:
                print(i,j,k)
                my_occ_grid.occupyCoords((i, j, k))

    
    return bodyId, my_occ_grid
    