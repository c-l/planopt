import copy
import numpy as np
import trajoptpy
import utils


class EnvManager(object):
  init_bodies = {}  # key: body name, value: body pointer
  non_robot_links = set()
  tray_stack = []
  tray_stack_height = 0.0
  grabbed = {'rightarm': set(),
             'leftarm': set()}
  object_groups = {}  # key: base body name, values: set of body names (Ex: {'tray': {'object11', object21}})
  drawer_init_x = None
  drawer_open_dist = 0

  body_dofs = {}
  containers = ('tray', 'basket', 'dishwasherdrawer')

  @classmethod
  def init_openrave_state(self, env):
    bodies = env.GetBodies()
    for body in bodies:
      body_name = body.GetName()
      EnvManager.init_bodies[body_name] = body

      if not body.IsRobot():
        for link in body.GetLinks():
          EnvManager.non_robot_links.add(link)

  @classmethod
  def save_openrave_state(self, env):
    data = {'Robots': [],
            'EnvBodies': [],
            'AllBodies': [],
            'EnvManagerState': EnvManager._save_state(),
            'BodyDofs': {}}
    for body in env.GetBodies():
      body_name = body.GetName()
      if body_name not in EnvManager.init_bodies:
        EnvManager.init_bodies[body_name] = body
      if body.IsRobot():
        robot = env.GetRobot(body_name)
        data['Robots'].append((body_name, EnvManager.save_robot_state(robot)))
      else:
        data['EnvBodies'].append((body, body.GetTransform()))
        data['BodyDofs'][body.GetName()] = copy.deepcopy(body.GetDOFValues())
    for body in EnvManager.init_bodies.values():
      data['AllBodies'].append((body, body.GetTransform()))

    return data

  @classmethod
  def restore_openrave_state(self, env, data):
    for robot_name, values in data['Robots']:
      robot = env.GetRobot(robot_name)
      EnvManager.restore_robot_state(robot, values)

      # release all grabbed objects. will restore later.
      for obj in robot.GetGrabbed():
        robot.Release(obj)
        EnvManager.include_robot_grabbed_collisions(robot, obj)

    env_body_names = [b.GetName() for b in env.GetBodies()]
    for body, body_t in data['EnvBodies']:
      if body.GetName() not in env_body_names:
        env.AddKinBody(body)

    for body, body_t in data['AllBodies']:
      body.SetTransform(body_t)

    EnvManager._reset_mutual_collisions(env)

    for robot_name, values in data['Robots']:
      robot = env.GetRobot(robot_name)
      EnvManager.restore_grabbed(robot, values['Grabbed'])

    for body_name, dofs in data['BodyDofs'].items():
      body = env.GetKinBody(body_name)
      n_dofs = len(body.GetDOFValues())
      body.SetDOFValues(dofs, range(n_dofs))

    EnvManager._restore_state(copy.deepcopy(data['EnvManagerState']))

    # restore collision exclusion between bodies in a group if robot is holding base
    for base_body_name, body_names in EnvManager.object_groups.items():
      if base_body_name in EnvManager.grabbed['rightarm'] or \
         base_body_name in EnvManager.grabbed['leftarm']:

        bodies = set()
        bodies.add(base_body_name)
        for bn in body_names:
          bodies.add(bn)

        env_bodies = set([b.GetName() for b in env.GetBodies()])
        EnvManager.exclude_mutual_collisions(env, bodies.intersection(env_bodies))

  @classmethod
  def _save_state(self):
    return {'tray_stack': list(EnvManager.tray_stack),
            'tray_stack_height': EnvManager.tray_stack_height,
            'grabbed': copy.deepcopy(EnvManager.grabbed),
            'object_groups': copy.deepcopy(EnvManager.object_groups),
            'body_dofs': EnvManager.body_dofs,}

  @classmethod
  def _restore_state(self, state):
    # make copies of everything
    EnvManager.tray_stack = list(state['tray_stack'])
    EnvManager.tray_stack_height = state['tray_stack_height']
    EnvManager.grabbed = copy.deepcopy(state['grabbed'])
    EnvManager.object_groups = copy.deepcopy(state['object_groups'])
    EnvManager.body_dofs = state['body_dofs']

  @classmethod
  def save_robot_state(self, robot):
    # make copies of everything
    values = {}
    values['Transform'] = copy.deepcopy(robot.GetTransform())
    values['DOFValues'] = copy.deepcopy(robot.GetDOFValues())
    values['Grabbed'] = copy.deepcopy(EnvManager.grabbed)
    return values

  @classmethod
  def restore_robot_state(self, robot, values):
    robot.SetTransform(copy.deepcopy(values['Transform']))
    robot.SetDOFValues(copy.deepcopy(values['DOFValues']))

  @classmethod
  def include_robot_grabbed_collisions(self, robot, grabbed):
    env = robot.GetEnv()
    cc = trajoptpy.GetCollisionChecker(env)
    for grabbedlink in grabbed.GetLinks():
      cc.IncludeCollisionPair(grabbedlink, grabbedlink)
      for robotlink in robot.GetLinks():
        # for some reason we need both
        cc.IncludeCollisionPair(robotlink, grabbedlink)
        cc.IncludeCollisionPair(grabbedlink, robotlink)

  @classmethod
  def exclude_manip_grabbed_collisions(self, robot, grabbed, manip):
    manip_to_use = robot.GetManipulator(manip)
    env = robot.GetEnv()
    cc = trajoptpy.GetCollisionChecker(env)
    for grabbedlink in grabbed.GetLinks():
      cc.ExcludeCollisionPair(grabbedlink, grabbedlink)
      for maniplink in manip_to_use.GetChildLinks():
        if 'gripper' in maniplink.GetName():
          # for some reason we need both
          cc.ExcludeCollisionPair(maniplink, grabbedlink)
          cc.ExcludeCollisionPair(grabbedlink, maniplink)

  @classmethod
  def _reset_mutual_collisions(self, env):
    cc = trajoptpy.GetCollisionChecker(env)
    env_links = set()
    for b in env.GetBodies():
      for l in b.GetLinks():
        env_links.add(l)

    for link1 in EnvManager.non_robot_links:
      if link1 not in env_links:
        continue
      for link2 in EnvManager.non_robot_links:
        if link2 not in env_links:
          continue
        # for some reason we need both
        cc.IncludeCollisionPair(link1, link2)
        cc.IncludeCollisionPair(link2, link1)

  @classmethod
  def exclude_mutual_collisions(self, env, objects):
    cc = trajoptpy.GetCollisionChecker(env)
    for obj1 in objects:
      for obj2 in objects:
        for obj1link in env.GetKinBody(obj1).GetLinks():
          for obj2link in env.GetKinBody(obj2).GetLinks():
            # for some reason we need both
            cc.ExcludeCollisionPair(obj1link, obj2link)
            cc.ExcludeCollisionPair(obj2link, obj1link)

  @classmethod
  def include_mutual_collisions(self, env, objects):
    cc = trajoptpy.GetCollisionChecker(env)
    for obj1 in objects:
      for obj2 in objects:
        for obj1link in env.GetKinBody(obj1).GetLinks():
          for obj2link in env.GetKinBody(obj2).GetLinks():
            # for some reason we need both
            cc.IncludeCollisionPair(obj1link, obj2link)
            cc.IncludeCollisionPair(obj2link, obj1link)

  @classmethod
  def restore_grabbed(self, robot, grabbed):
    active_manip = robot.GetActiveManipulator()
    for manip, objects in grabbed.items():
      robot.SetActiveManipulator(manip)
      for obj_name in objects:
        obj = robot.GetEnv().GetKinBody(obj_name)
        if obj is not None:
          robot.Grab(obj)
          EnvManager.exclude_manip_grabbed_collisions(robot, obj, manip)
    robot.SetActiveManipulator(active_manip)

  @classmethod
  def grab(self, env, robot, obj, manip):
    EnvManager._grab_helper(robot, obj, manip)
    obj_name = obj.GetName()

    # object group stuff
    if obj_name in EnvManager.object_groups.keys():
      for obj2_name in EnvManager.object_groups[obj_name]:
        obj2 = env.GetKinBody(obj2_name)
        EnvManager._grab_helper(robot, obj2, manip)
      EnvManager.exclude_mutual_collisions(
        env, list(EnvManager.object_groups[obj_name]) + [obj_name])
    EnvManager._object_groups_remove(robot, obj)

    # specific tray stuff
    if obj_name in EnvManager.tray_stack:
      if obj_name == EnvManager.tray_stack[-1]:
        EnvManager.tray_stack.pop()
        if "plate" in obj_name:
          EnvManager.tray_stack_height -= 0.1
        else:
          EnvManager.tray_stack_height -= 0.035
      else:
        raise Exception("Trying to remove an object that isn't at the\
          top of the tray stack!")


  @classmethod
  def _grab_helper(self, robot, obj, manip):
    robot.SetActiveManipulator(manip)
    robot.Grab(obj)
    EnvManager.grabbed[manip].add(obj.GetName())
    EnvManager.exclude_manip_grabbed_collisions(robot, obj, manip)
    utils.close_gripper(robot, manip)

  @classmethod
  def release(self, env, robot, obj, manip, table=None, open_amount=0.54):
    EnvManager._release_helper(robot, obj, manip, open_amount=open_amount)
    obj_name = obj.GetName()

    if table is not None:
      table_name = table.GetName()
      if table_name == 'dishwashersurface': table_name = 'dishwasherdrawer'
      if table_name in EnvManager.containers:
        EnvManager._object_groups_add(robot, obj, table_name)
      if table.GetName() == 'tray':
        EnvManager.tray_stack.append(obj_name)
        EnvManager.putdown_helper(
          robot, obj, table,
          offset=EnvManager.tray_stack_height)
        if "plate" in obj_name:
          EnvManager.tray_stack_height += 0.1
        else:
          EnvManager.tray_stack_height += 0.035

      elif table.GetName() == 'basket':
        EnvManager.putdown_helper(robot, obj, table, use_origin_z=True, offset=0.04) # raise slightly in case flat disks
      elif table.GetName() == 'washer':
        EnvManager.putdown_helper(robot, obj, table, use_origin_z=True, offset=0.64+0.05) # washer origin is at bottom, then add offset to barrel base
      else:
        EnvManager.putdown_helper(robot, obj, table)

    # object group stuff
    if obj_name in EnvManager.object_groups.keys():
      for obj2_name in EnvManager.object_groups[obj_name]:
        obj2 = env.GetKinBody(obj2_name)
        EnvManager._release_helper(robot, obj2, manip, open_amount=open_amount)
      EnvManager.include_mutual_collisions(
        env, list(EnvManager.object_groups[obj_name]) + [obj_name])

    # specific tray stuff
    if obj_name == 'tray':
      offset = 0
      for obj2_name in EnvManager.tray_stack:
        obj2 = env.GetKinBody(obj2_name)
        EnvManager.putdown_helper(robot, obj2, obj, offset=offset)
        if "plate" in obj2_name:
          offset += 0.1
        else:
          offset += 0.035

    # specific drawer stuff
    if 'drawer' in obj_name:
      if obj_name == 'drawer': open_amount = EnvManager.drawer_init_x - obj.GetTransform()[0, 3]
      elif obj_name == 'dishwasherdrawer': open_amount = EnvManager.drawer_init_x - obj.GetTransform()[1, 3]
      if open_amount > 0.0001:
        EnvManager.drawer_open_dist = open_amount
        print("Drawer opened to %f" % EnvManager.drawer_open_dist)

  @classmethod
  def _release_helper(self, robot, obj, manip, open_amount=0.54):
    robot.SetActiveManipulator(manip)
    robot.Release(obj)
    EnvManager.grabbed[manip].remove(obj.GetName())
    EnvManager.include_robot_grabbed_collisions(robot, obj)
    utils.open_gripper(robot, manip, value=open_amount)

  @classmethod
  def _object_groups_add(self, robot, obj, base_body_name):
    # update object_groups and exclude collisions
    if base_body_name not in EnvManager.object_groups:
      EnvManager.object_groups[base_body_name] = set()
    EnvManager.object_groups[base_body_name].add(obj.GetName())

  @classmethod
  def _object_groups_remove(self, robot, obj):
    # update object_groups and include collisions
    obj_name = obj.GetName()
    for base_body_name in EnvManager.object_groups.keys():
      if obj_name in EnvManager.object_groups[base_body_name]:
        EnvManager.object_groups[base_body_name].remove(obj_name)

  @classmethod
  def putdown_helper(self, robot, obj, table, offset=0, use_origin_z=False):
    # transforming the object onto table
    T = obj.GetTransform()

    _, _, _, _, table_z = utils.get_object_limits(table)
    if use_origin_z:
      table_z = table.GetTransform()[2, 3]
    ab = obj.ComputeAABB()
    obj_min_z = ab.pos()[2] - ab.extents()[2]
    diff_z = obj_min_z - table_z
    T[2, 3] -= diff_z
    T[2, 3] += offset
    obj.SetTransform(T)

  @classmethod
  def setup_drawer_object_groups(self, env):
    bodies = env.GetBodies()
    for body in bodies:
      body_name = body.GetName()
      if 'drawer' in body_name and 'outer' not in body_name:
        min_x, max_x, min_y, max_y, min_z, max_z = utils.get_object_limits_2(body)
        inside_set = set()
        for sub_body in bodies:
          sub_body_name = sub_body.GetName()

          # prevent drawer and drawer outer from being added to the set
          if body is not sub_body and 'drawer' not in sub_body_name:
            sub_body_t = sub_body.GetTransform()
            sub_body_x = sub_body_t[0, 3]
            sub_body_y = sub_body_t[1, 3]
            sub_body_z = sub_body_t[2, 3]

            if ((min_x < sub_body_x < max_x) and
               (min_y < sub_body_y < max_y) and
               (min_z < sub_body_z < max_z)):
              inside_set.add(sub_body_name)
        EnvManager.object_groups[body_name] = inside_set


if __name__ == "__main__":
  import openravepy
  import numpy as np
  env = openravepy.Environment()
  env.Load("../environments/created_info.dae")
  EnvManager.init_openrave_state(env)

  obj = env.GetKinBody("object11")
  T = obj.GetTransform()
  T[2, 3] += 10
  obj.SetTransform(T)
  env.Remove(obj)
  saved = EnvManager.save_openrave_state(env)
  obj.SetTransform(np.eye(4))
  EnvManager.restore_openrave_state(env, saved)
  print(env.GetBodies())
  print(obj.GetTransform())
