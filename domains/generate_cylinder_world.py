#! /usr/bin/python

import sys
sys.path.append('../src')

import random
import openravepy
import numpy as np
import utils
import time
import getopt
import object_models

TARGET_OBJECT = "object1"
OBJECT_PREFIX = "object"
TABLE_NAMES = {"rll":"rll_table", "small":"table6"}

# Folder containing openrave environments
ENV_PATH = "../environments/"

# Folder containing openrave object models
MODEL_PATH = "../models/"

# Default env file
DEFAULT_ENV_FILE = "created_info.dae"

def execute(num_objects, cylinder_dims=(0.05, 0.25), max_radial_distance=0.73, robot_dist_from_table=0.05, seed=int(time.time()), envFile=ENV_PATH+DEFAULT_ENV_FILE, tabletype='rll', viewer=False):
    # Spawns PR2, table, and num_objects cylinders.
    # cylinder_dims: (radius, height)
    # max_radial_distance: x, y distance from PR2 which you want cylinders to be within
    # robot_dist_from_table: distance between front edge of PR2 and short edge of table (this is approximated)

    assert max_radial_distance > robot_dist_from_table, "Robot too far from table"
    env = openravepy.Environment()
    if viewer:
        env.SetViewer('qtcoin')

    spawn_table_pr2(env, robot_dist_from_table, tabletype)
    rand = np.random.RandomState(seed=seed)
    num_spawned = generate_world(env, TABLE_NAMES[tabletype], num_objects, cylinder_dims, max_radial_distance, rand)
    on_table(env, TABLE_NAMES[tabletype])
    # drop_table = add_drop_table(env, dist_from_robot=(-0.4, 1.5))
    # drop_table_obj = env.GetKinBody("object%d"%(num_spawned-1))
    # drop_table_obj.SetTransform(drop_table.GetTransform())
    # drop_table_obj.SetName("objectdrop")
    # object_models.on_table(drop_table_obj, drop_table, offset=0.02)

    env.Save(envFile)
    with open("seed.txt", 'w+') as f:
        f.write(str(seed))
    print("Environment %s created; seed for %d objects attempted (%d actual): %d" %(envFile, num_objects, num_spawned, seed))
    import ipdb; ipdb.set_trace()
    return env

def generate_world(env, table_name, num_objects, cylinder_dims, max_radial_distance, rand):
    collision_checker = openravepy.RaveCreateCollisionChecker(env, "bullet")
    collision_checker.SetCollisionOptions(openravepy.CollisionOptions.Contacts)
    env.SetCollisionChecker(collision_checker)
    table = env.GetKinBody(table_name)
    count = 0

    for obj_num in range(num_objects):
        obj_name = "object" + repr(obj_num)
        if obj_name == TARGET_OBJECT:
            color = [0.2, 0.2, 0.2]
        else:
            color = [0, 0.8, 0.8]
        create_collision_free_random_cylinder(env, table, cylinder_dims, obj_name, max_radial_distance, rand, color=color)
        body = env.GetKinBody(obj_name)
        if body is not None:
            print("Object %s created on surface %s" % (body.GetName(), table_name))
            count += 1
        else:
            print("Could not generate collision free cylinder for %s" % obj_name)

    return count

def create_collision_free_random_cylinder(env, table, dimensions, obj_name, max_radial_distance, rand, color, num_trials=50):
    radius, height = dimensions
    robot = env.GetRobots()[0]

    for i in xrange(num_trials):
        cylinder = create_cylinder(env, table, radius, height, obj_name, max_radial_distance, rand, color)
        collision = False
        for body in env.GetBodies():
            if body != table and body != robot and env.CheckCollision(body, cylinder):
                collision = True
                break
        if not collision:
            return
        env.Remove(cylinder)

def create_cylinder(env, table, radius, height, body_name, max_radial_distance, rand, color):
    robot_pos = openravepy.poseFromMatrix(env.GetRobots()[0].GetTransform())[4:7]
    min_x, max_x, min_y, max_y, table_height = utils.get_object_limits(table)

    x = rand.uniform(min_x + radius, max_x - radius)
    y = rand.uniform(min_y + radius, max_y - radius)
    # while (x - robot_pos[0])**2 + (y - robot_pos[1])**2 > max_radial_distance:
    #     x = rand.uniform(min_x + radius, max_x - radius)
    #     y = rand.uniform(min_y + radius, max_y - radius)
    z = table_height + height / 2

    t = openravepy.matrixFromPose([1, 0, 0, 0, x, y, z])
    cylinder = object_models.create_cylinder(env, body_name, t, [radius, height], color)
    env.Add(cylinder, False)

    return cylinder

def spawn_table_pr2(env, robot_dist_from_table, tabletype='rll'):
    if tabletype == 'rll':
        thickness = 0.2
        legheight = 0.6
        table = object_models.create_table(env, TABLE_NAMES[tabletype], 2.235, 0.94, thickness, 1.3, 0.6, legheight)
    elif tabletype == 'small':
        thickness = 0.2
        legheight = 0.6
        table = object_models.create_table(env, TABLE_NAMES[tabletype], 0.7 - robot_dist_from_table, 1.5, thickness, 0.2 - robot_dist_from_table, 0.2, legheight)

    x = -utils.get_object_limits(table)[0] + 0.35 + robot_dist_from_table
    y = 0
    z = thickness / 2 + legheight
    table.SetTransform(openravepy.matrixFromPose([1, 0, 0, 0, x, y, z]))
    # env.AddKinBody(table)
    env.AddRobot(table)

    # robot = env.ReadRobotXMLFile("../models/pr2/pr2-head-kinect.xml")
    robot = env.ReadRobotXMLFile("../models/pr2/pr2.zae")
    env.Add(robot)

def on_table(env, table_name):
    table = env.GetKinBody(table_name)
    if table:
      print("Transforming objects onto table")
      for obj in env.GetBodies():
          if obj.GetName().startswith(OBJECT_PREFIX):
              object_models.on_table(obj, table, offset=0.02)

def add_drop_table(env, dist_from_robot):
  thickness = 0.1
  legheight = 0.4
  table = object_models.create_table(env, "table", 0.5, 0.5, thickness, 0.1, 0.1, legheight)

  x, y = dist_from_robot
  z = thickness / 2 + legheight
  rot = openravepy.quatFromAxisAngle((0, 0, 0))
  table.SetTransform(openravepy.matrixFromPose(list(rot) + [x, y, z]))
  env.AddKinBody(table)
  return table

if __name__ == "__main__":
    usage_str = "Usage: python %s -v (for viewer) -n [num_objects] -s [seed] -e [env_filename] -t [rll|small]" %sys.argv[0]
    cylinder_dims = (0.04, 0.25)
    num_objects = None
    seed = int(time.time())
    envFile = ENV_PATH+DEFAULT_ENV_FILE
    tabletype = 'small'
    viewer = False

    try:
        opts, args = getopt.getopt(sys.argv[1:],"vn:s:e:t:")
    except getopt.GetoptError:
        print(usage_str)
        sys.exit(2)

    for opt, arg in opts:
      if opt == "-v":
        print("Setting viewer to True")
        viewer = True
      if opt == "-n":
        num_objects = int(arg)
      elif opt == "-s":
        seed = int(arg)
      elif opt == "-e":
        envFile = arg
      elif opt == "-t":
        tabletype = arg
    if num_objects is None:
      print(usage_str)
      print("Need to provide num_objects")
      sys.exit(2)

    try:
        execute(num_objects, cylinder_dims, seed=seed, envFile=envFile,
                tabletype=tabletype, viewer=viewer)
        if viewer:
            raw_input("press enter to quit")
    except ValueError:
        print(usage_str)
