import numpy as np
from opt.opt_prob import OptProb
from opt.variable import Variable
from opt.constraints import Constraints
from interface.hl_param import HLParam, Traj
from interface.fluents.lin_eq_fluent import LinEqFluent
from interface.fluents.lin_le_fluent import LinLEFluent
from interface.hl_plan import HLPlan
from openravepy import *
import ipdb


def create_cylinder(env, body_name, t, dims, color=[0, 1, 1]):
    infocylinder = KinBody.GeometryInfo()
    infocylinder._type = GeometryType.Cylinder
    infocylinder._vGeomData = dims
    # ipdb.set_trace()
    infocylinder._bVisible = True
    infocylinder._vDiffuseColor = color
    # infocylinder._t[2, 3] = dims[1] / 2

    cylinder = RaveCreateKinBody(env, '')
    cylinder.InitFromGeometries([infocylinder])
    cylinder.SetName(body_name)
    cylinder.SetTransform(t)

    return cylinder


def init_openrave_test_env():
    env = Environment()  # create openrave environment
    env.SetViewer('qtcoin')  # attach viewer (optional)

    obstacles = np.matrix('-0.576036866359447, 0.918128654970760, 1;\
                    -0.806451612903226,-1.07017543859649, 1;\
                    1.01843317972350,-0.988304093567252, 1;\
                    0.640552995391705,0.906432748538011, 1;\
                    -0.576036866359447, 0.918128654970760, -1;\
                    -0.806451612903226,-1.07017543859649, -1;\
                    1.01843317972350,-0.988304093567252, -1;\
                    0.640552995391705,0.906432748538011, -1')

    body = RaveCreateKinBody(env, '')
    vertices = np.array(obstacles)
    indices = np.array([[0, 1, 2], [2, 3, 0], [4, 5, 6], [6, 7, 4], [0, 4, 5],
                        [0, 1, 5], [1, 2, 5], [5, 6, 2], [2, 3, 6], [6, 7, 3],
                        [0, 3, 7], [0, 4, 7]])
    body.InitFromTrimesh(trimesh=TriMesh(vertices, indices), draw=True)
    body.SetName('obstacle')
    for link in body.GetLinks():
        for geom in link.GetGeometries():
            geom.SetDiffuseColor((.9, .9, .9))
    env.AddKinBody(body)

    # create cylindrical object
    transform = np.eye(4)
    transform[0, 3] = -2
    obj = create_cylinder(env, 'obj', np.eye(4), [.35, 2])
    obj.SetTransform(transform)
    env.AddKinBody(obj)

    # import ipdb; ipdb.set_trace() # BREAKPOINT
    env.Load("robot.xml")

    robot = env.GetRobots()[0]
    transform = np.eye(4)
    transform[0, 3] = -1
    robot.SetTransform(transform)
    transparency = 0.7
    # for body in [robot, body, obj]:
    for body in [robot, body]:
        for link in body.GetLinks():
            for geom in link.GetGeometries():
                geom.SetTransparency(transparency)
    # import ctrajoptpy
    # cc = ctrajoptpy.GetCollisionChecker(env)
    # cc.SetContactDistance(np.infty)
    # cc.SetContactDistance(np.infty)
    # collisions = cc.BodyVsBody(robot, obj)
    # for c in collisions:
    # if c.GetDistance() > 0:
    #     distance = c.GetDistance()
    #     print "distance: ", distance

    # import ipdb; ipdb.set_trace() # BREAKPOINT
    return env


def test_move():
    from interface.hl_actions.move import Move
    env = init_openrave_test_env()
    robot = env.GetRobots()[0]

    start = HLParam("start", 3, 1, is_var=False, value=np.array([[-2], [0], [0]]))
    end = HLParam("end", 3, 1, is_var=False, value=np.array([[2], [0], [0]]))

    # env = self.env.CloneSelf(1)  # clones objects in the environment
    robot = env.GetRobots()[0]
    # obj = env.GetKinBody('obj')
    hl_plan = HLPlan(env, robot)
    move = Move(0, hl_plan, env, robot, start, end)

    prob = OptProb()
    model = prob.get_model()

    param_to_var = {}
    params = move.get_params()
    for param in params:
        var = Variable(model, param)
        param_to_var[param] = var
        prob.add_var(var)
    move.traj.resample() # remove later
    model.update()

    constraints = Constraints(model)
    for hla in [move]:
        for fluent in hla.preconditions:
            fluent.pre()
        for fluent in hla.postconditions:
            fluent.post()
        for fluent in hla.preconditions + hla.postconditions:
            lhs = fluent.lhs.to_gurobi_expr(param_to_var)
            rhs = fluent.rhs.to_gurobi_expr(param_to_var)
            model.update()

            if isinstance(fluent, LinLEFluent):
                constraints.add_leq_cntr(lhs, rhs)
            elif isinstance(fluent, LinEqFluent):
                constraints.add_eq_cntr(lhs, rhs)
    prob.add_constraints(constraints)

    for hla in [move]:
        hla.cost.to_gurobi_fn(param_to_var)
        prob.inc_obj(hla.cost)

    prob.convexify(0.1)
    prob.optimize()

    for param, var in param_to_var.items():
        var.update_hl_param()
    traj = np.array([[-2.        , -1.8974359 , -1.79487179, -1.69230769, -1.58974359,
        -1.48717949, -1.38461538, -1.28205128, -1.17948718, -1.07692308,
        -0.97435897, -0.87179487, -0.76923077, -0.66666667, -0.56410256,
        -0.46153846, -0.35897436, -0.25641026, -0.15384615, -0.05128205,
         0.05128205,  0.15384615,  0.25641026,  0.35897436,  0.46153846,
         0.56410256,  0.66666667,  0.76923077,  0.87179487,  0.97435897,
         1.07692308,  1.17948718,  1.28205128,  1.38461538,  1.48717949,
         1.58974359,  1.69230769,  1.79487179,  1.8974359 ,  2.        ],
       [-0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
         0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
         0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
         0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
         0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
         0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
         0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
         0.        ,  0.        ,  0.        ,  0.        , -0.        ],
       [-0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
         0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
         0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
         0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
         0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
         0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
         0.        ,  0.        ,  0.        ,  0.        ,  0.        ,
         0.        ,  0.        ,  0.        ,  0.        , -0.        ]])
    assert np.allclose(traj, param_to_var[move.traj].value)
    move.plot()
    ipdb.set_trace()


test_move()
