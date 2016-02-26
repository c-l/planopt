import numpy as np
import ctrajoptpy
from opt.function import CollisionFn
from interface.fluents.fluent import FnLEFluent
from numdifftools import Jacobian

class NotObstructsPR2(FnLEFluent):
    # def __init__(self, env, hl_action, robot, priority, traj, obj, obj_loc, dsafe=0.05):
    def __init__(self, env, hl_action, robot, priority, traj, obj, obj_loc=None, dsafe=0.01):
        self.env = env
        self.plotting_env = hl_action.hl_plan.env
        self.hl_action = hl_action
        self.traj = traj
        self.obj = obj
        self.obj_loc = obj_loc
        # self.obj_loc = obj_loc
        self.robot = robot
        self.priority = priority

        self.name = "NotObstructsPR2"

        self.robot_dofs = traj.rows
        self.timesteps = traj.cols
        if self.obj_loc is None:
            self.obj_dofs = 0
        else:
            self.obj_dofs = obj_loc.rows
        self.cc = ctrajoptpy.GetCollisionChecker(env)
        self.dsafe = dsafe

    def pre(self):
        traj = self.traj

        g = lambda x: self.collisions(x) # function inequality constraint g(x) <= 0
        # g = lambda x: self.num_check(self.test_collisions, x) # numerical check
        obj_loc_list = []
        if self.obj_loc is not None:
            obj_loc_list = [self.obj_loc for i in range(self.timesteps)]
        self.fn = CollisionFn([self.traj] + obj_loc_list, g)

    # TODO: compute collisions properly
    # @profile
    def collisions(self, traj):
        env = self.env
        T = self.timesteps
        # K = self.K

        # ensure that there's gradient information outside dsafe
        self.cc.SetContactDistance(self.dsafe + .1)

        handles = []

        # if self.obj_loc is not None:
        #     val = np.zeros((2*T, 1))
        #     jac = np.zeros((2*T, traj.size))
        # else:
        # val = np.zeros((T, 1))
        # jac = np.zeros((T, traj.size))
        # if self.obj_loc is not None:
        #     self.obj.set_pose(env, self.obj_loc.value)
        val = None
        jac = None

        for t in range(self.timesteps):
            # xt = self.traj.value[K*t:K*(t+1)]
            # xt = traj[:,t:t+1]
            robot_start_ind = self.robot_dofs * t
            robot_end_ind = self.robot_dofs * (t+1)
            # obj_start_ind = self.robot_dofs * T + self.obj_dofs * t
            # obj_end_ind = self.robot_dofs * T + self.obj_dofs * (t+1)
            # xt = traj[self.robot_dofs*t:self.robot_dofs*(t+1)]
            xt = traj[robot_start_ind:robot_end_ind]
            self.robot.set_pose(env, xt)
            ot = None
            # if self.obj_loc is not None:
            #     # ot = traj[K*(t+T):K*(t+1+T)]
            #     ot = traj[obj_start_ind:obj_end_ind]
            #     self.obj.set_pose(env, ot)
            collisions = self.cc.BodyVsAll(self.robot.get_env_body(env))

            # val_t, robot_jac_t, obj_jac_t = self.calc_grad_and_val(xt, ot, collisions)
            val_t, robot_jac_t = self.calc_grad_and_val(xt, ot, collisions)
            if robot_jac_t is not None:
                assert val_t is not None
                rows = val_t.shape[0]
                jac_t = np.zeros((rows, T*self.robot_dofs))
                jac_t[:, robot_start_ind:robot_end_ind] = robot_jac_t
                if val is None:
                    assert jac is None
                    val = val_t
                    jac = jac_t
                else:
                    assert jac is not None
                    val = np.vstack((val, val_t))
                    jac = np.vstack((jac, jac_t))


                # val[t], jac[t, robot_start_ind:robot_end_ind] = col_val, robot_jac
                # if self.obj_loc is not None:
                #     val[t+T], jac[t+T, obj_start_ind:obj_end_ind] = col_val, obj_jac
                #     # cross terms
                #     jac[t, obj_start_ind:obj_end_ind] = obj_jac
                #     jac[t+T, robot_start_ind:robot_end_ind] = robot_jac

        self.plotting_env.UpdatePublishedBodies()
        handles = []
        # import ipdb; ipdb.set_trace()

        return (val, jac)

    # @profile
    def calc_grad_and_val(self, xt, ot, collisions):
        robot = self.robot.get_env_body(self.env)
        obj = self.obj.get_env_body(self.env)
        robot_grad = None
        obj_grad = None
        vals = []
        robot_grads = []
        for c in collisions:
            linkAParent = c.GetLinkAParentName()
            linkBParent = c.GetLinkBParentName()
            # print "linkAParent: ", linkAParent
            # print "linkBParent: ", linkBParent
            linkA = c.GetLinkAName()
            linkB = c.GetLinkBName()
            # print "linkA", linkA
            # print "linkB", linkB

            linkRobot = None
            linkObj = None
            sign = 1
            if linkAParent == self.robot.name and linkBParent == self.obj.name:
                ptRobot = c.GetPtA()
                linkRobot = linkA
                sign = -1
                ptObj = c.GetPtB()
                linkObj = linkB
            elif linkBParent == self.robot.name and linkAParent == self.obj.name:
                ptRobot = c.GetPtB()
                linkRobot = linkB
                sign = 1
                ptObj = c.GetPtA()
                linkObj = linkA
            else:
                continue

            distance = c.GetDistance()
            normal = c.GetNormal()
            print "linkA", linkA
            print "linkB", linkB
            print "distance: ", distance

            # plotting
            self.plot_collision(ptRobot, ptObj, distance)

            vals.append(self.dsafe - distance)

            robot_link_ind = robot.GetLink(linkRobot).GetIndex()
            robot_jac = robot.CalculateActiveJacobian(robot_link_ind, ptRobot)
            robot_grad = np.dot(sign * normal, robot_jac)

            robot_grads.append(robot_grad)
            # if ot is not None:
            #     obj_link_ind = obj.GetLink(linkObj).GetIndex()
            #     obj_jac = obj.CalculateActiveJacobian(obj_link_ind, ptObj)
            #     import ipdb; ipdb.set_trace()
            #     obj_grad = np.dot(normal, obj_jac)

        if vals:
            vals = np.vstack(vals)
            robot_grads = np.vstack(robot_grads)
        else:
            vals = None
            robot_grads = None
        print "vals: ", vals
        return vals, robot_grads

    def num_check(self, fn, traj):
        val, ana_jac = fn(traj)
        jac_fn = Jacobian(lambda x: fn(x)[0].flatten())
        num_jac = jac_fn(traj.flatten())
        # assert np.allclose(num_jac, ana_jac)
        if not np.allclose(num_jac, ana_jac):
            inds = np.where(np.isclose(num_jac, ana_jac, atol=1e-2) == False)
            print inds
            print "num_jac: {}".format(num_jac[inds])
            print "ana_jac: {}".format(ana_jac[inds])
            # fn(traj, debug=True)
            print "ratio: {}".format(np.divide(num_jac[inds],ana_jac[inds]))
            import ipdb; ipdb.set_trace()

        return (val, num_jac)

    def test_collisions(self, traj):
        env = self.env
        T = self.timesteps
        # K = self.K

        # ensure that there's gradient information outside dsafe
        self.cc.SetContactDistance(self.dsafe + .05)

        handles = []

        # if self.obj_loc is not None:
        #     val = np.zeros((2*T, 1))
        #     jac = np.zeros((2*T, traj.size))
        # else:
        # val = np.zeros((T, 1))
        # jac = np.zeros((T, traj.size))
        # if self.obj_loc is not None:
        #     self.obj.set_pose(env, self.obj_loc.value)
        val = None
        jac = None

        t = self.timesteps - 1
        robot_start_ind = self.robot_dofs * t
        robot_end_ind = self.robot_dofs * (t+1)
        # obj_start_ind = self.robot_dofs * T + self.obj_dofs * t
        # obj_end_ind = self.robot_dofs * T + self.obj_dofs * (t+1)
        xt = traj[robot_start_ind:robot_end_ind]
        self.robot.set_pose(env, xt)
        ot = None
        # if self.obj_loc is not None:
        #     # ot = traj[K*(t+T):K*(t+1+T)]
        #     ot = traj[obj_start_ind:obj_end_ind]
        #     self.obj.set_pose(env, ot)
        collisions = self.cc.BodyVsAll(self.robot.get_env_body(env))

        # val_t, robot_jac_t, obj_jac_t = self.calc_grad_and_val(xt, ot, collisions)
        val_t, robot_jac_t = self.test_calc_grad_and_val(xt, ot, collisions)
        if robot_jac_t is not None:
            assert val_t is not None
            rows = val_t.shape[0]
            jac_t = np.zeros((rows, T*self.robot_dofs))
            jac_t[:, robot_start_ind:robot_end_ind] = robot_jac_t
            if val is None:
                assert jac is None
                val = val_t
                jac = jac_t
            else:
                assert jac is not None
                val = np.vstack((val, val_t))
                jac = np.vstack((jac, jac_t))


            # val[t], jac[t, robot_start_ind:robot_end_ind] = col_val, robot_jac
            # if self.obj_loc is not None:
            #     val[t+T], jac[t+T, obj_start_ind:obj_end_ind] = col_val, obj_jac
            #     # cross terms
            #     jac[t, obj_start_ind:obj_end_ind] = obj_jac
            #     jac[t+T, robot_start_ind:robot_end_ind] = robot_jac

        self.plotting_env.UpdatePublishedBodies()
        handles = []
        # import ipdb; ipdb.set_trace()

        return (val, jac)

    # @profile
    def test_calc_grad_and_val(self, xt, ot, collisions):
        robot = self.robot.get_env_body(self.env)
        obj = self.obj.get_env_body(self.env)
        robot_grad = None
        obj_grad = None
        vals = []
        # robot_grads = []
        link_to_index_map = {'r_gripper_l_finger_tip_link':0, 'r_gripper_r_finger_tip_link':1,
                            'r_gripper_r_finger_link':2, 'r_gripper_l_finger_link':3,
                            'r_gripper_palm_link':4, 'r_wrist_roll_link':5,
                            'r_upper_arm_link':6, 'r_elbow_flex_link':7,
                            'r_forearm_link':8, 'r_wrist_flex_link':9}
        rows = len(link_to_index_map.keys())
        val = np.zeros((rows,1))
        robot_grad = np.zeros((rows, self.robot_dofs))
        for c in collisions:
            linkAParent = c.GetLinkAParentName()
            linkBParent = c.GetLinkBParentName()
            # print "linkAParent: ", linkAParent
            # print "linkBParent: ", linkBParent
            linkA = c.GetLinkAName()
            linkB = c.GetLinkBName()
            # print "linkA", linkA
            # print "linkB", linkB

            linkRobot = None
            linkObj = None
            sign = 1
            if linkAParent == self.robot.name and linkBParent == self.obj.name:
                ptRobot = c.GetPtA()
                linkRobot = linkA
                sign = -1
                ptObj = c.GetPtB()
                linkObj = linkB
            elif linkBParent == self.robot.name and linkAParent == self.obj.name:
                ptRobot = c.GetPtB()
                linkRobot = linkB
                sign = 1
                ptObj = c.GetPtA()
                linkObj = linkA
            else:
                continue

            distance = c.GetDistance()
            normal = c.GetNormal()
            # print "linkA", linkA
            # print "linkB", linkB
            # print "distance: ", distance

            # plotting
            self.plot_collision(ptRobot, ptObj, distance)

            # vals.append(self.dsafe - distance)

            robot_link_ind = robot.GetLink(linkRobot).GetIndex()
            robot_jac = robot.CalculateActiveJacobian(robot_link_ind, ptRobot)
            ind = link_to_index_map[linkRobot]
            robot_grad[ind, :] = np.dot(sign * normal, robot_jac)
            val[ind, 0] = self.dsafe - distance

            # robot_grads.append(robot_grad)
            # if ot is not None:
            #     obj_link_ind = obj.GetLink(linkObj).GetIndex()
            #     obj_jac = obj.CalculateActiveJacobian(obj_link_ind, ptObj)
            #     import ipdb; ipdb.set_trace()
            #     obj_grad = np.dot(normal, obj_jac)

        # if vals:
        #     vals = np.vstack(vals)
        #     robot_grads = np.vstack(robot_grads)
        # else:
        #     vals = None
        #     robot_grads = None
        # print "vals: ", vals
        # return vals, robot_grads
        return val, robot_grad

    def plot_collision(self, ptA, ptB, distance):
        handles = []
        if not np.allclose(ptA, ptB, atol=1e-3):
            if distance < 0:
                handles.append(self.plotting_env.drawarrow(p1=ptA, p2=ptB, linewidth=.001,color=(1,0,0)))
            else:
                handles.append(self.plotting_env.drawarrow(p1=ptA, p2=ptB, linewidth=.001,color=(0,0,0)))
        self.hl_action.add_plot_handles(handles)
