(define (domain robotics)
	(:requirements :strips :equality :typing)
	(:types movable pose location grasp)
    (:constants none - movable none_gp - grasp)
	(:predicates
		(RobotAt ?p - pose)
		(InManip ?obj - movable ?gp - grasp)
        (ObjAt ?obj - movable ?loc - location)
        (IsMP ?l1 - pose ?l2 - pose)

        (Obstructs ?obj - movable ?p1 - pose ?p2 - pose)
        (IsGP ?p - pose ?obj ?gp)
        (IsPDP ?p - pose ?obj ?gp)
        ;(IsPickPose ?lrobot - pose)
        ;(IsPlacePose ?lrobot - pose)
	)

	(:action move
		:parameters (?l1 - pose ?l2 - pose)
		:precondition (and
                    (RobotAt ?l1)
                    (InManip none none_gp)
                    ;(IsMP ?l1 ?l2)
                    (forall (?o - movable) (and (not (Obstructs ?o ?l1 ?l2)) (not (Obstructs ?o ?l2 ?l1))))
        )
		:effect (and
                    (RobotAt ?l2)
					(not (RobotAt ?l1))
        )
	)

	(:action move_w_obj
		:parameters (?l1 - pose ?l2 - pose ?obj - movable ?loc1 - location ?loc2 - location ?gp - grasp)
		:precondition (and
                    (RobotAt ?l1)
                    (InManip ?obj ?gp)
                    ;(ObjAt ?obj ?loc1)
                    ;(IsMP ?l1 ?l2)
                    (forall (?o - movable) (and (not (Obstructs ?o ?l1 ?l2)) (not (Obstructs ?o ?l2 ?l1))))
        )
		:effect (and
                    (RobotAt ?l2)
					(not (RobotAt ?l1))
					;(not (ObjAt ?obj ?loc1))
                    ;(ObjAt ?obj ?loc2)
        )
	)

   	(:action pick
		:parameters (?lrobot - pose ?obj - movable ?loc - location ?gp - grasp)
		:precondition (and
                    ;(IsPickPose ?lrobot)
                    (IsGP ?lrobot ?obj ?gp)
                    (InManip none none_gp)
					(RobotAt ?lrobot)
                    (ObjAt ?obj ?loc)
        )
		:effect (and
                    (not (InManip none none_gp))
                    (InManip ?obj ?gp)
					(not (ObjAt ?obj ?loc))
        )
	)

    (:action place
		:parameters (?lrobot - pose ?obj - movable ?loc - location ?gp - grasp)
		:precondition (and
                    (IsPDP ?lrobot ?obj ?gp)
                    ;(IsPlacePose ?lrobot)
                    (InManip ?obj ?gp)
					(RobotAt ?lrobot)
        )
		:effect (and
                    (ObjAt ?obj ?loc)
                    (InManip none none_gp)
                    (not (InManip ?obj ?gp))
        )
	)
)
