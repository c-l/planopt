(define (problem twocanProb) (:domain robotics)
    (:objects
        can1 - movable
        can2 - movable
        can3 - movable
        can4 - movable
        can5 - movable
        can6 - movable
        can7 - movable
        ;; can8 - movable
        ;; can9 - movable
        ;; can10 - movable

        grasp_can1 - grasp
        grasp_can2 - grasp
        grasp_can3 - grasp
        grasp_can4 - grasp
        grasp_can5 - grasp
        grasp_can6 - grasp
        grasp_can7 - grasp
        ;; grasp_can8 - grasp
        ;; grasp_can9 - grasp
        ;; grasp_can10 - grasp

        robotinitloc - pose

        gp_can1 - pose
        gp_can2 - pose
        gp_can3 - pose
        gp_can4 - pose
        gp_can5 - pose
        gp_can6 - pose
        gp_can7 - pose
        ;; gp_can8 - pose
        ;; gp_can9 - pose
        ;; gp_can10 - pose
        pdp_can1_can1initloc - pose
        pdp_can2_can2initloc - pose
        pdp_can3_can3initloc - pose
        pdp_can4_can4initloc - pose
        pdp_can5_can5initloc - pose
        pdp_can6_can6initloc - pose
        pdp_can7_can7initloc - pose
        ;; pdp_can8_can8initloc - pose
        ;; pdp_can9_can9initloc - pose
        ;; pdp_can10_can10initloc - pose
        pdp_can1_can1temploc - pose
        pdp_can2_can2temploc - pose
        pdp_can3_can3temploc - pose
        pdp_can4_can4temploc - pose
        pdp_can5_can5temploc - pose
        pdp_can6_can6temploc - pose
        pdp_can7_can7temploc - pose
        ;; pdp_can8_can8temploc - pose
        ;; pdp_can9_can9temploc - pose
        ;; pdp_can10_can10temploc - pose
        pdp_can1_goal1 - pose
        pdp_can2_goal2 - pose

        can1initloc - location
        can2initloc - location
        can3initloc - location
        can4initloc - location
        can5initloc - location
        can6initloc - location
        can7initloc - location
        ;; can8initloc - location
        ;; can9initloc - location
        ;; can10initloc - location
        can1temploc - temploc
        can2temploc - temploc
        can3temploc - temploc
        can4temploc - temploc
        can5temploc - temploc
        can6temploc - temploc
        can7temploc - temploc
        ;; can8temploc - temploc
        ;; can9temploc - temploc
        ;; can10temploc - temploc
        goal1 - temploc
        goal2 - temploc
    )
    (:init
        (RobotAt robotinitloc)
        (InManip none none_gp)

        (IsGP gp_can1 can1 grasp_can1)
        (IsGP gp_can2 can2 grasp_can2)
        (IsGP gp_can3 can3 grasp_can3)
        (IsGP gp_can4 can4 grasp_can4)
        (IsGP gp_can5 can5 grasp_can5)
        (IsGP gp_can6 can6 grasp_can6)
        (IsGP gp_can7 can7 grasp_can7)
        ;; (IsGP gp_can8 can8 grasp_can8)
        ;; (IsGP gp_can9 can9 grasp_can9)
        ;; (IsGP gp_can10 can10 grasp_can10)

        (IsAccessPointFor pdp_can1_can1initloc can1 can1initloc)
        (IsAccessPointFor pdp_can2_can2initloc can2 can2initloc)
        (IsAccessPointFor pdp_can3_can3initloc can3 can3initloc)
        (IsAccessPointFor pdp_can4_can4initloc can4 can4initloc)
        (IsAccessPointFor pdp_can5_can5initloc can5 can5initloc)
        (IsAccessPointFor pdp_can6_can6initloc can6 can6initloc)
        (IsAccessPointFor pdp_can7_can7initloc can7 can7initloc)
        ;; (IsAccessPointFor pdp_can8_can8initloc can8 can8initloc)
        ;; (IsAccessPointFor pdp_can9_can9initloc can9 can9initloc)
        ;; (IsAccessPointFor pdp_can10_can10initloc can10 can10initloc)
        (IsAccessPointForTemp pdp_can1_can1temploc can1 can1temploc)
        (IsAccessPointForTemp pdp_can2_can2temploc can2 can2temploc)
        (IsAccessPointForTemp pdp_can3_can3temploc can3 can3temploc)
        (IsAccessPointForTemp pdp_can4_can4temploc can4 can4temploc)
        (IsAccessPointForTemp pdp_can5_can5temploc can5 can5temploc)
        (IsAccessPointForTemp pdp_can6_can6temploc can6 can6temploc)
        (IsAccessPointForTemp pdp_can7_can7temploc can7 can7temploc)
        ;; (IsAccessPointForTemp pdp_can8_can8temploc can8 can8temploc)
        ;; (IsAccessPointForTemp pdp_can9_can9temploc can9 can9temploc)
        ;; (IsAccessPointForTemp pdp_can10_can10temploc can10 can10temploc)
        (IsAccessPointForTemp pdp_can1_goal1 can1 goal1)
        (IsAccessPointForTemp pdp_can2_goal2 can2 goal2)

        (ObjAt can1 can1initloc)
        (ObjAt can2 can2initloc)
        (ObjAt can3 can3initloc)
        (ObjAt can4 can4initloc)
        (ObjAt can5 can5initloc)
        (ObjAt can6 can6initloc)
        (ObjAt can7 can7initloc)
        ;; (ObjAt can8 can8initloc)
        ;; (ObjAt can9 can9initloc)
        ;; (ObjAt can10 can10initloc)
    )
    (:goal
        (and (ObjAtTemp can1 goal1) (ObjAtTemp can2 goal2))
    )
)
