import numpy as np
from interface.hl_actions.move import Move
from interface.hl_actions.pick import Pick
from opt.opt_prob import OptProb
from opt.variable import Variable
from opt.constraints import Constraints
from opt.solver import Solver
from interface.hl_param import HLParam, Traj, GP
from interface.fluents.fluent import AndFluent, LinFluent, LinEqFluent, LinLEFluent, FnFluent, FnLEFluent, FnEQFluent
from interface.hl_plan import HLPlan
from openravepy import *
from utils import mat_to_base_pose
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

def make_transparent(body, transparency=0.7):
    for link in body.GetLinks():
        for geom in link.GetGeometries():
            geom.SetTransparency(transparency)

def add_cyl_robot(env):
    env.Load("robot.xml")

    robot = env.GetRobots()[0]
    transform = np.eye(4)
    transform[0, 3] = -1
    robot.SetTransform(transform)
    make_transparent(robot)

def add_obstacle(env):
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
    make_transparent(body)

def add_object(env):
    # create cylindrical object
    transform = np.eye(4)
    transform[0, 3] = -2
    obj = create_cylinder(env, 'obj', np.eye(4), [.35, 2])
    obj.SetTransform(transform)
    env.AddKinBody(obj)
    make_transparent(obj)

def move_test_env():
    env = Environment()  # create openrave environment
    env.SetViewer('qtcoin')  # attach viewer (optional)

    add_cyl_robot(env)
    add_obstacle(env)
    raw_input('continue past warnings')

    return env

def pick_test_env():
    env = Environment()  # create openrave environment
    env.SetViewer('qtcoin')  # attach viewer (optional)

    add_cyl_robot(env)
    add_obstacle(env)
    add_object(env)
    raw_input('continue past warnings')

    return env


def test_no_obstructs_move():
    env = move_test_env()
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
    model.update()

    constraints = Constraints(model)
    for hla in [move]:
        for fluent in hla.preconditions:
            fluent.pre()
        for fluent in hla.postconditions:
            fluent.post()
        for fluent in hla.preconditions + hla.postconditions:
            if isinstance(fluent, LinLEFluent):
                lhs = fluent.lhs.to_gurobi_expr(param_to_var)
                rhs = fluent.rhs.to_gurobi_expr(param_to_var)
                model.update()
                constraints.add_leq_cntr(lhs, rhs)
            elif isinstance(fluent, LinEqFluent):
                lhs = fluent.lhs.to_gurobi_expr(param_to_var)
                rhs = fluent.rhs.to_gurobi_expr(param_to_var)
                model.update()
                constraints.add_eq_cntr(lhs, rhs)
    return constraints
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

def add_fluent_to_constraints(constraints, fluent, param_to_var):
    if isinstance(fluent, AndFluent):
        for subfluent in fluent.fluents:
            add_fluent_to_constraints(constraints, subfluent, param_to_var)
    elif isinstance(fluent, FnFluent):
        fluent.fn.to_gurobi_fn(param_to_var)
        if isinstance(fluent, FnLEFluent):
            constraints.add_nonlinear_ineq_constraint(fluent.fn)
        elif isinstance(fluent, FnEQFluent):
            constraints.add_nonlinear_eq_constraint(fluent.fn)
    elif isinstance(fluent, LinFluent):
        lhs = fluent.lhs.to_gurobi_expr(param_to_var)
        rhs = fluent.rhs.to_gurobi_expr(param_to_var)
        if isinstance(fluent, LinLEFluent):
            constraints.add_leq_cntr(lhs, rhs)
        elif isinstance(fluent, LinEqFluent):
            constraints.add_eq_cntr(lhs, rhs)

def model_cnts_from_hla(model, hlas, param_to_var, priority):
    constraints = Constraints(model)
    for hla in hlas:
        for fluent in hla.preconditions:
            fluent.pre()
        for fluent in hla.postconditions:
            fluent.post()
        for fluent in hla.preconditions + hla.postconditions:
            if fluent.priority == 1:
                import ipdb; ipdb.set_trace()
            if fluent.priority <= priority:
                add_fluent_to_constraints(constraints, fluent, param_to_var)
    return constraints


def test_move():
    env = move_test_env()
    robot = env.GetRobots()[0]

    start = HLParam("start", 3, 1, is_var=False, value=np.array([[-2], [0], [0]]))
    end = HLParam("end", 3, 1, is_var=False, value=np.array([[2], [0], [0]]))

    # env = self.env.CloneSelf(1)  # clones objects in the environment
    robot = env.GetRobots()[0]
    # obj = env.GetKinBody('obj')
    hl_plan = HLPlan(env, robot)
    move_env = env.CloneSelf(1) # clones objects in the environment
    move_robot = move_env.GetRobots()[0]
    move = Move(0, hl_plan, move_env, move_robot, start, end)

    hlas = [move]
    solve_ll_plan(hlas)
    ipdb.set_trace()


def test_pick():
    env = pick_test_env()
    robot = env.GetRobots()[0]

    # start = HLParam("start", 3, 1, is_var=False, value=np.array([[-2], [0], [0]]))
    # end = HLParam("end", 3, 1, is_var=False, value=np.array([[2], [0], [0]]))

    hl_plan = HLPlan(env, robot)
    pick_env = env.CloneSelf(1) # clones objects in the environment
    pick_robot = pick_env.GetRobots()[0]

    rp = HLParam("rp", 3, 1)
    gp = GP("gp", 3, 1)
    pick_obj = pick_env.GetKinBody('obj')
    obj_loc = HLParam("obj_loc", 3, 1, is_var=False, value=mat_to_base_pose(pick_obj.GetTransform()))

    gp.resample()

    pick = Pick(0, hl_plan, env, robot, rp, pick_obj, obj_loc, gp)
    hlas = [pick]

    solve_ll_plan(hlas)
    assert np.allclose(pick.pos.value, np.array([[-1.41840404],[-0.18333333],[ 0.        ]]))


def solve_ll_plan(hlas, priority):
    prob = OptProb()
    model = prob.get_model()

    params = []
    for hla in hlas:
        params += hla.get_params()

    param_to_var = {}
    for param in params:
        var = Variable(model, param)
        param_to_var[param] = var
        prob.add_var(var)
    model.update()

    constraints = model_cnts_from_hla(model, hlas, param_to_var, priority)
    prob.add_constraints(constraints)
    for param, var in param_to_var.items():
        var.set(param.value)

    for hla in hlas:
        if hla.cost != 0.0:
            hla.cost.to_gurobi_fn(param_to_var)
            prob.inc_obj(hla.cost)

    solver = Solver()
    solver.penalty_sqp(prob)

    for param, var in param_to_var.items():
        var.update_hl_param()


def test_pick_and_move():
    env = pick_test_env()
    robot = env.GetRobots()[0]

    # start = HLParam("start", 3, 1, is_var=False, value=np.array([[-2], [0], [0]]))
    # end = HLParam("end", 3, 1, is_var=False, value=np.array([[2], [0], [0]]))

    hl_plan = HLPlan(env, robot)
    pick_env = env.CloneSelf(1) # clones objects in the environment
    pick_robot = pick_env.GetRobots()[0]
    move_env = env.CloneSelf(1) # clones objects in the environment
    move_robot = move_env.GetRobots()[0]

    rp = HLParam("rp", 3, 1)
    end = HLParam("end", 3, 1, is_var=False, value=np.array([[2],[0],[0]]))
    gp = GP("gp", 3, 1)
    pick_obj = pick_env.GetKinBody('obj')
    move_obj = move_env.GetKinBody('obj')
    obj_loc = HLParam("obj_loc", 3, 1, is_var=False, value=mat_to_base_pose(pick_obj.GetTransform()))

    gp.resample()

    pick = Pick(0, hl_plan, env, robot, rp, pick_obj, obj_loc, gp)
    move = Move(0, hl_plan, move_env, move_robot, rp, end, move_obj, gp)
    init_hlas = [pick]
    hlas = [pick, move]

    solve_ll_plan(init_hlas)
    import ipdb; ipdb.set_trace()
    solve_ll_plan(hlas)

    ipdb.set_trace()

def test_pick_and_move_with_cnt_reordering():
    env = pick_test_env()
    robot = env.GetRobots()[0]

    # start = HLParam("start", 3, 1, is_var=False, value=np.array([[-2], [0], [0]]))
    # end = HLParam("end", 3, 1, is_var=False, value=np.array([[2], [0], [0]]))

    hl_plan = HLPlan(env, robot)
    pick_env = env.CloneSelf(1) # clones objects in the environment
    pick_robot = pick_env.GetRobots()[0]
    move_env = env.CloneSelf(1) # clones objects in the environment
    move_robot = move_env.GetRobots()[0]

    rp = HLParam("rp", 3, 1)
    end = HLParam("end", 3, 1, is_var=False, value=np.array([[2],[0],[0]]))
    gp = GP("gp", 3, 1)
    pick_obj = pick_env.GetKinBody('obj')
    move_obj = move_env.GetKinBody('obj')
    obj_loc = HLParam("obj_loc", 3, 1, is_var=False, value=mat_to_base_pose(pick_obj.GetTransform()))

    gp.resample()

    pick = Pick(0, hl_plan, env, robot, rp, pick_obj, obj_loc, gp)
    move = Move(0, hl_plan, move_env, move_robot, rp, end, move_obj, gp)
    hlas = [pick, move]

    solve_ll_plan(hlas, 0)
    import ipdb; ipdb.set_trace()
    solve_ll_plan(hlas, 1)

    ipdb.set_trace()
# test_no_obstructs_move()
# test_move()
# test_pick()
# test_pick_and_move()
test_pick_and_move_with_cnt_reordering()
