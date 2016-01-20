(define (problem twocanProb) (:domain robotics)
    (:objects
        can1 - movable
        can2 - movable

        g1 - grasp
        g2 - grasp

        robot_init_loc - pose

        gp_can1 - pose
        gp_can2 - pose
        pdp_can1_can1_init_loc - pose
        pdp_can2_can2_init_loc - pose
        pdp_can1_goal1 - pose
        pdp_can2_goal2 - pose

        ;pick_can1 - pose
        ;place_can1 - pose
        ;pick_can2 - pose
        ;place_can2 - pose

        can1_init_loc - location
        can2_init_loc - location
        goal1 - location
        goal2 - location
    )
    (:init
        (RobotAt robot_init_loc)
        (InManip none none_gp)

        (IsGP gp_can1 can1 g1)
        (IsGP gp_can2 can2 g2)

        (IsAccessPointFor pdp_can1_can1_init_loc can1 can1_init_loc)
        (IsAccessPointFor pdp_can2_can2_init_loc can2 can2_init_loc)
        (IsAccessPointFor pdp_can1_goal1 can1 goal1)
        (IsAccessPointFor pdp_can2_goal2 can2 goal2)

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
