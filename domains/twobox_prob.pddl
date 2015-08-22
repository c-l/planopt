(define (problem logProb) (:domain robotics)
    (:objects
        box1 - movable
        box2 - movable
        
        gp1 - grasp
        gp2 - grasp

        robot_init_loc - pose
        pick_box1 - pose
        place_box1 - pose
        pick_box2 - pose
        place_box2 - pose

        box1_init_loc - location
        box2_init_loc - location
        goal1 - location
        goal2 - location
    )
    (:init 
        (RobotAt robot_init_loc)
        (InManip none none_gp)

        (IsGP pick_box1 box1 gp1)
        (IsPDP place_box1 box1 gp1)

        (IsGP pick_box2 box2 gp2)
        (IsPDP place_box2 box2 gp2)

        (ObjAt box1 box1_init_loc)
        (ObjAt box2 box2_init_loc)
    )
    (:goal
        ;(RobotAt robot_init_loc)
        ;(RobotAt pick_box1)
        ;(InManip box1 gp1)
        ;(ObjAt box1 goal)
        ;(and (ObjAt box1 goal1) (InManip box2 gp2))
        ;(and (ObjAt box1 goal1) (ObjAt box2 goal2))
        (ObjAt box2 goal2)
    )
    (:metric minimize (total-cost))
)
