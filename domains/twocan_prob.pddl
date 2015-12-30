(define (problem logProb) (:domain robotics)
    (:objects
        can1 - movable
        can2 - movable

        gp1 - grasp
        gp2 - grasp

        robot_init_loc - pose
        pick_can1 - pose
        place_can1 - pose
        pick_can2 - pose
        place_can2 - pose

        can1_init_loc - location
        can2_init_loc - location
        goal1 - location
        goal2 - location
    )
    (:init
        (RobotAt robot_init_loc)
        (InManip none none_gp)

        (IsGP pick_can1 can1 gp1)
        (IsPDP place_can1 can1 gp1)

        (IsGP pick_can2 can2 gp2)
        (IsPDP place_can2 can2 gp2)

        (ObjAt can1 can1_init_loc)
        (ObjAt can2 can2_init_loc)
    )
    (:goal
        ;(RobotAt robot_init_loc)
        ;(InManip can2 gp2)
        ;(RobotAt pick_can1)
        ;(InManip can1 gp1)
        (ObjAt can1 goal1)
        ;(and (ObjAt can1 goal1) (InManip can2 gp2))
        ;(and (ObjAt can1 goal1) (ObjAt can2 goal2))
        ;(ObjAt can2 goal2)
    )
    (:metric minimize (total-cost))
)
