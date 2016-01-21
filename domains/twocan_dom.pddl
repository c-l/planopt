(define (domain robotics)
	(:requirements :strips :equality :typing)
	(:types movable pose location grasp)
    (:constants none - movable none_gp - grasp)
	(:predicates
		(RobotAt ?p - pose)
		(InManip ?obj - movable ?gp - grasp)
        (ObjAt ?obj - movable ?loc - location)
        (IsMP ?l1 - pose ?l2 - pose)

        (Obstructs ?obj - movable ?loc - location ?p2 - pose)
        (IsGP ?p - pose ?obj ?gp)
        (IsPDP ?p - pose ?obj ?gp)
        (IsAccessPointFor ?p - pose ?obj - movable ?loc - location)
				(NotUnique ?p - pose)
	)

	(:action move
		:parameters (?l1 - pose ?l2 - pose)
		:precondition (and
                    (RobotAt ?l1)
                    (InManip none none_gp)
                    ;(IsMP ?l1 ?l2)
										(forall (?o -movable)
												(forall (?loc - location)
														(or (not (ObjAt ?o ?loc)) (not (Obstructs ?o ?loc ?l2)))
												)
										)
        )
		:effect (and
                    (RobotAt ?l2)
					(not (RobotAt ?l1))
        )
	)

	(:action move_w_obj
		:parameters (?l1 - pose ?l2 - pose ?obj - movable ?gp - grasp)
		:precondition (and
                    (RobotAt ?l1)
                    (InManip ?obj ?gp)
                    ;(IsMP ?l1 ?l2)
										(forall (?o -movable)
												(forall (?loc - location)
														(or (not (ObjAt ?o ?loc)) (not (Obstructs ?o ?loc ?l2)))
												)
										)
        )
		:effect (and
                    (RobotAt ?l2)
					(not (RobotAt ?l1))
        )
	)

   	(:action pick
		:parameters (?obj - movable ?loc - location ?lrobot - pose ?gp - grasp)
		:precondition (and
                    (IsGP ?lrobot ?obj ?gp)
                    (InManip none none_gp)
					(RobotAt ?lrobot)
                    (ObjAt ?obj ?loc)
										(not (NotUnique ?lrobot))
        )
		:effect (and
                    (not (InManip none none_gp))
                    (InManip ?obj ?gp)
					(not (ObjAt ?obj ?loc))
										(NotUnique ?lrobot)
        )
	)

    (:action place
		:parameters (?obj - movable ?loc - location ?lrobot - pose ?gp - grasp)
		:precondition (and
                    (IsAccessPointFor ?lrobot ?obj ?loc)
                    (InManip ?obj ?gp)
										(not (NotUnique ?lrobot))
					(RobotAt ?lrobot)
        )
		:effect (and
                    (ObjAt ?obj ?loc)
                    (InManip none none_gp)
                    (not (InManip ?obj ?gp))
										(NotUnique ?lrobot)
        )
	)
)
