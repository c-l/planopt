from openravepy import *
import os
import errno
import signal
from functools import wraps
import StringIO
import numpy as np
from math import atan2

def base_pose_to_mat(pose):
    # x, y, rot = pose
    x = pose[0,0]
    y = pose[1,0]
    rot = pose[2,0]
    q = quatFromAxisAngle((0, 0, rot)).tolist()
    pos = [x, y, 0]
    # pos = np.vstack((x,y,np.zeros(1)))
    matrix = matrixFromPose(q + pos)
    return matrix

def mat_to_base_pose(mat):
    pose = poseFromMatrix(mat)
    x = pose[4]
    y = pose[5]
    rot = axisAngleFromRotationMatrix(mat)[2]
    return np.array([[x],[y],[rot]])
    # return [x, y, rot]

def obj_pose_from_transform(transform):
    trans = transform[:3,3]
    rot_matrix = transform[:3,:3]
    yaw, pitch, roll = _ypr_from_rot_matrix(rot_matrix)
    # ipdb.set_trace()
    return np.array((yaw, pitch, roll, trans[0], trans[1], trans[2]))


def transform_from_obj_pose(pose):
    alpha, beta, gamma, x, y, z = pose
    Rz, Ry, Rx = _axis_rot_matrices(pose)
    rot_mat = np.dot(Rz, np.dot(Ry, Rx))
    transform = np.eye(4)
    transform[:3,:3] = rot_mat
    transform[:3,3] = [x,y,z]
    return transform

def _axis_rot_matrices(pose):
    from math import cos, sin
    alpha, beta, gamma, x, y, z = pose
    Rz_2d = np.array([[cos(alpha), -sin(alpha)], [sin(alpha), cos(alpha)]])
    Ry_2d = np.array([[cos(beta), sin(beta)], [-sin(beta), cos(beta)]])
    Rx_2d = np.array([[cos(gamma), -sin(gamma)], [sin(gamma), cos(gamma)]])
    I = np.eye(3)
    Rz = I.copy()
    Rz[:2,:2] = Rz_2d
    Ry = I.copy()
    Ry[[[0],[2]],[0,2]] = Ry_2d
    Rx = I.copy()
    Rx[1:3,1:3] = Rx_2d
    # ipdb.set_trace()
    return Rz, Ry, Rx

def _ypr_from_rot_matrix(r):
    # alpha
    yaw = atan2(r[1,0], r[0,0])
    # beta
    pitch = atan2(-r[2,0],np.sqrt(r[2,1]**2+r[2,2]**2))
    # gamma
    roll = atan2(r[2,1], r[2,2])
    # ipdb.set_trace()
    return (yaw, pitch, roll)

class TimeoutError(Exception):
    pass

def timeout(seconds=10, error_message=os.strerror(errno.ETIME)):
    def decorator(func):
        def _handle_timeout(signum, frame):
            raise TimeoutError(error_message)
        def wrapper(*args, **kwargs):
            signal.signal(signal.SIGALRM, _handle_timeout)
            signal.alarm(seconds)
            try:
                result = func(*args, **kwargs)
            finally:
                signal.alarm(0)
            return result
        return wraps(func)(wrapper)
    return decorator


def getStringFile(str):
    strPlanFileH = StringIO.StringIO()
    strPlanFileH.write(str)
    strPlanFileH.seek(0)
    return strPlanFileH


def get_object_limits(obj):
    """Returns the bounding box of an object.

    Returns: min_x, max_x, min_y, max_y, z
    """

    ab = obj.ComputeAABB()
    max_x = ab.pos()[0] + ab.extents()[0]
    min_x = ab.pos()[0] - ab.extents()[0]

    max_y = ab.pos()[1] + ab.extents()[1]
    min_y = ab.pos()[1] - ab.extents()[1]
    z = ab.pos()[2] + ab.extents()[2]

    return min_x, max_x, min_y, max_y, z

def get_object_limits_2(obj):
    """
    Returns the bounding box of an object.
    Returns: min_x, max_x, min_y, max_y, min_z, max_z
    """

    ab = obj.ComputeAABB()
    max_x = ab.pos()[0] + ab.extents()[0]
    min_x = ab.pos()[0] - ab.extents()[0]

    max_y = ab.pos()[1] + ab.extents()[1]
    min_y = ab.pos()[1] - ab.extents()[1]

    max_z = ab.pos()[2] + ab.extents()[2]
    min_z = ab.pos()[2] - ab.extents()[2]

    return min_x, max_x, min_y, max_y, min_z, max_z

def get_object_center(obj):
    min_x, max_x, min_y, max_y, z = get_object_limits(obj)
    return [(min_x + max_x) / 2, (min_y + max_y) / 2, z]

def get_object_height(obj):
    ab = obj.ComputeAABB()
    max_z = ab.pos()[2] + ab.extents()[2]
    min_z = ab.pos()[2] - ab.extents()[2]
    return max_z - min_z

def create_body_at(env, T, name = "crash-test-dummy"):
    body = openravepy.RaveCreateKinBody(env, "")
    body.SetName(name)
    body.InitFromBoxes(np.array([[0,0,0, 0.04, 0.04, 0.1]]), True)
    p = openravepy.poseFromMatrix(T)
    p[-1] += 0.101
