from openravepy import *

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


