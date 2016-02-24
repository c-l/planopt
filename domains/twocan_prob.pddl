(define (problem twocanProb) (:domain robotics)
    (:objects
        can1 - movable
        can2 - movable

        grasp_can1 - grasp
        grasp_can2 - grasp

        robotinitloc - pose

        gp_can1 - pose
        gp_can2 - pose
        pdp_can1_can1temploc - pose
        pdp_can2_can2temploc - pose
        pdp_can1_goal1 - pose
        pdp_can2_goal2 - pose

        can1initloc - location
        can2initloc - location
        can1temploc - location
        can2temploc - location
        goal1 - location
        goal2 - location
    )
    (:init
        (RobotAt robotinitloc)
        (InManip none none_gp)

        (IsGP gp_can1 can1 grasp_can1)
        (IsGP gp_can2 can2 grasp_can2)

        (IsAccessPointFor pdp_can1_can1temploc can1 can1temploc)
        (IsAccessPointFor pdp_can2_can2temploc can2 can2temploc)
        (IsAccessPointFor pdp_can1_goal1 can1 goal1)
        (IsAccessPointFor pdp_can2_goal2 can2 goal2)

        (ObjAt can1 can1initloc)
        (ObjAt can2 can2initloc)
    )
    (:goal
        (and (ObjAt can1 goal1) (ObjAt can2 goal2))
    )
)
