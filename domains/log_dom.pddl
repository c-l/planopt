(define (domain robotics)
	(:requirements :strips :equality :typing)
	(:types movable pose location grasp)
    (:constants none - movable none_gp grasp)
	(:predicates
		(RobotAt ?p - pose)
		(InManip ?obj - movable ?gp - grasp)
        (ObjAt ?obj - movable ?loc - location)
        (IsMP ?l1 - pose ?l2 - pose)

        (IsPickPose ?lrobot - pose)
        (IsPlacePose ?lrobot - pose)
	)

    (:functions (total-cost) - number)
	(:action move
		:parameters (?l1 - pose ?l2 - pose)
		:precondition (and 
                    (RobotAt ?l1)
                    (InManip none none_gp)
                    (IsMP ?l1 ?l2)
        )
		:effect (and 
                    (RobotAt ?l2)
					(not (RobotAt ?l1))
                    (increase (total-cost) 10)
        )
	)

	(:action move_w_obj
		:parameters (?l1 - pose ?l2 - pose ?obj - movable ?gp - grasp)
		:precondition (and 
                    (RobotAt ?l1)
                    (InManip ?obj ?gp)
                    (IsMP ?l1 ?l2)
        )
		:effect (and 
                    (RobotAt ?l2)
					(not (RobotAt ?l1))
                    (increase (total-cost) 10)
        )
	)

   	(:action pick
		:parameters (?lrobot - pose ?obj - movable ?loc - location ?gp - grasp)
		:precondition (and 
                    (IsPickPose ?lrobot)
                    (InManip none none_gp)
					(RobotAt ?lrobot)
                    (ObjAt ?obj ?loc)
        )
		:effect (and 
                    (not (InManip none ?gp))
                    (not (InManip none none_gp))
                    (InManip ?obj ?gp)
					(not (ObjAt ?obj ?loc))
                    (increase (total-cost) 10)
        )
	)

    (:action place
		:parameters (?lrobot - pose ?obj - movable ?loc - location ?gp - grasp)
		:precondition (and 
                    (IsPlacePose ?lrobot)
                    (InManip ?obj ?gp)
					(RobotAt ?lrobot)
        )
		:effect (and 
                    (InManip none none_gp)
                    (not (InManip ?obj ?gp))
                    (ObjAt ?obj ?loc)
                    (increase (total-cost) 10)
        )
	)
)
