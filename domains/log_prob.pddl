(define (problem logProb) (:domain robotics)
    (:objects
        obj0 - movable
        
        gp1 - grasp

        robot_init_loc - pose
        pick_box1 - pose
        place_box1 - pose

        obj_init_loc - location
        goal - location
    )
    (:init 
        (RobotAt robot_init_loc)
        (InManip none none_gp)

        (IsMP robot_init_loc pick_box1)
        (IsMP robot_init_loc place_box1)
        (IsMP robot_init_loc pick_box1)
        (IsMP robot_init_loc place_box1)
        (IsMP place_box1 pick_box1)
        (IsMP pick_box1 place_box1)
        (IsMP place_box1 pick_box1)
        (IsMP pick_box1 place_box1)

        (ISMP pick_box1 pick_box1)
        (ISMP place_box1 place_box1)

        (IsPickPose pick_box1)
        (IsPlacePose place_box1)

        (ObjAt obj0 obj_init_loc)
    )
    (:goal
        ;(RobotAt robot_init_loc)
        ;(RobotAt pick_box1)
        ;(InManip box1 gp1)
        (ObjAt obj0 goal)
    )
    (:metric minimize (total-cost))
)
