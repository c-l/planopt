from openravepy import *
import os
import errno
import signal
from functools import wraps
import StringIO

def base_pose_to_mat(pose):
    x, y, rot = pose
    q = quatFromAxisAngle((0, 0, rot)).tolist()
    pos = [x, y, 0]
    matrix = matrixFromPose(q + pos)
    return matrix

def mat_to_base_pose(mat):
    pose = poseFromMatrix(mat)
    x = pose[4]
    y = pose[5]
    rot = axisAngleFromRotationMatrix(mat)[2]
    return [x, y, rot]


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
