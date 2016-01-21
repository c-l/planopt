(define (problem twocanProb) (:domain robotics)
    (:objects
        can1 - movable
        can2 - movable

        grasp_can1_0 - grasp
        grasp_can2_0 - grasp
        grasp_can1_1 - grasp
        grasp_can2_1 - grasp
        grasp_can1_2 - grasp
        grasp_can2_2 - grasp
        grasp_can1_3 - grasp
        grasp_can2_3 - grasp

        robot_init_loc - pose

        gp_can1_0 - pose
        gp_can2_0 - pose
        gp_can1_1 - pose
        gp_can2_1 - pose
        gp_can1_2 - pose
        gp_can2_2 - pose
        gp_can1_3 - pose
        gp_can2_3 - pose
        pdp_can1_can1_init_loc_0 - pose
        pdp_can2_can2_init_loc_0 - pose
        pdp_can1_can1_init_loc_1 - pose
        pdp_can2_can2_init_loc_1 - pose
        pdp_can1_can1_init_loc_2 - pose
        pdp_can2_can2_init_loc_2 - pose
        pdp_can1_can1_init_loc_3 - pose
        pdp_can2_can2_init_loc_3 - pose
        pdp_can1_goal1_0 - pose
        pdp_can2_goal2_0 - pose
        pdp_can1_goal1_1 - pose
        pdp_can2_goal2_1 - pose
        pdp_can1_goal1_2 - pose
        pdp_can2_goal2_2 - pose
        pdp_can1_goal1_3 - pose
        pdp_can2_goal2_3 - pose

        can1_init_loc - location
        can2_init_loc - location
        goal1 - location
        goal2 - location
    )
    (:init
        (RobotAt robot_init_loc)
        (InManip none none_gp)

        (IsGP gp_can1_0 can1 grasp_can1_0)
        (IsGP gp_can2_0 can2 grasp_can2_0)
        (IsGP gp_can1_1 can1 grasp_can1_1)
        (IsGP gp_can2_1 can2 grasp_can2_1)
        (IsGP gp_can1_2 can1 grasp_can1_2)
        (IsGP gp_can2_2 can2 grasp_can2_2)
        (IsGP gp_can1_3 can1 grasp_can1_3)
        (IsGP gp_can2_3 can2 grasp_can2_3)

        (IsAccessPointFor pdp_can1_can1_init_loc_0 can1 can1_init_loc)
        (IsAccessPointFor pdp_can2_can2_init_loc_0 can2 can2_init_loc)
        (IsAccessPointFor pdp_can1_can1_init_loc_1 can1 can1_init_loc)
        (IsAccessPointFor pdp_can2_can2_init_loc_1 can2 can2_init_loc)
        (IsAccessPointFor pdp_can1_can1_init_loc_2 can1 can1_init_loc)
        (IsAccessPointFor pdp_can2_can2_init_loc_2 can2 can2_init_loc)
        (IsAccessPointFor pdp_can1_can1_init_loc_3 can1 can1_init_loc)
        (IsAccessPointFor pdp_can2_can2_init_loc_3 can2 can2_init_loc)
        (IsAccessPointFor pdp_can1_goal1_0 can1 goal1)
        (IsAccessPointFor pdp_can2_goal2_0 can2 goal2)
        (IsAccessPointFor pdp_can1_goal1_1 can1 goal1)
        (IsAccessPointFor pdp_can2_goal2_1 can2 goal2)
        (IsAccessPointFor pdp_can1_goal1_2 can1 goal1)
        (IsAccessPointFor pdp_can2_goal2_2 can2 goal2)
        (IsAccessPointFor pdp_can1_goal1_3 can1 goal1)
        (IsAccessPointFor pdp_can2_goal2_3 can2 goal2)

        (ObjAt can1 can1_init_loc)
        (ObjAt can2 can2_init_loc)
    )
    (:goal
        ;(RobotAt robot_init_loc)
        ;(InManip can2 gp2)
        ;(RobotAt pick_can1)
        ;(InManip can1 gp1)
        ;(ObjAt can1 goal1)
        ;(and (ObjAt can1 goal1) (InManip can2 gp2))
        (and (ObjAt can1 goal1) (ObjAt can2 goal2))
        ;(ObjAt can2 goal2)
    )
)
