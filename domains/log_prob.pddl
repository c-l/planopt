(define (problem logProb) (:domain robotics)
    (:objects
        obj0 - movable
        
        gp1 - grasp

        robot_init_loc - pose
        pick_obj0 - pose
        place_obj0 - pose

        obj_init_loc - location
        goal - location
    )
    (:init 
        (RobotAt robot_init_loc)
        (InManip none none_gp)

        ;(IsMP robot_init_loc pick_obj0)
        ;(IsMP robot_init_loc place_obj0)
        ;(IsMP robot_init_loc pick_obj0)
        ;(IsMP robot_init_loc place_obj0)
        ;(IsMP place_obj0 pick_obj0)
        ;(IsMP pick_obj0 place_obj0)
        ;(IsMP place_obj0 pick_obj0)
        ;(IsMP pick_obj0 place_obj0)

        ;(ISMP pick_obj0 pick_obj0)
        ;(ISMP place_obj0 place_obj0)

        ;(IsPickPose pick_obj0)
        ;(IsPlacePose place_obj0)
        (IsGP pick_obj0 obj0 gp1)
        (IsPDP place_obj0 obj0 gp1)

        (ObjAt obj0 obj_init_loc)
    )
    (:goal
        ;(RobotAt robot_init_loc)
        ;(RobotAt pick_obj0)
        ;(InManip obj0 gp1)
        (ObjAt obj0 goal)
    )
    (:metric minimize (total-cost))
)
