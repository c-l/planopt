from openravepy import *
import os
import errno
import signal
from functools import wraps
import StringIO
import numpy as np

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

