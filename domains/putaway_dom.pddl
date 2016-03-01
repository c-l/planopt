(define (domain robotics)
	(:requirements :strips :equality :typing)
	(:types movable pose location grasp temploc)
    (:constants none - movable none_gp - grasp)
	(:predicates
		(RobotAt ?p - pose)
		(InManip ?obj - movable ?gp - grasp)
        (ObjAt ?obj - movable ?loc - location)
        (ObjAtTemp ?obj - movable ?tloc - temploc)
        (IsMP ?l1 - pose ?l2 - pose)

        (Obstructs ?obj - movable ?loc - location ?p2 - pose)
        (ObstructsTemp ?obj - movable ?tloc - temploc ?p2 - pose)
        (IsGP ?p - pose ?obj ?gp)
        (IsPDP ?p - pose ?obj ?gp)
        (IsAccessPointFor ?p - pose ?obj - movable ?loc - location)
        (IsAccessPointForTemp ?p - pose ?obj - movable ?tloc - temploc)
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
                                        (forall (?o -movable)
												(forall (?tloc - temploc)
														(or (not (ObjAtTemp ?o ?tloc)) (not (ObstructsTemp ?o ?tloc ?l2)))
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
                    (not (InManip none none_gp))
                    ;(IsMP ?l1 ?l2)
										(forall (?o -movable)
												(forall (?loc - location)
														(or (not (ObjAt ?o ?loc)) (not (Obstructs ?o ?loc ?l2)))
												)
										)
                                        (forall (?o -movable)
												(forall (?tloc - temploc)
														(or (not (ObjAtTemp ?o ?tloc)) (not (ObstructsTemp ?o ?tloc ?l2)))
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
        )
		:effect (and
                    (not (InManip none none_gp))
                    (InManip ?obj ?gp)
					(not (ObjAt ?obj ?loc))
        )
	)

    (:action pick
		:parameters (?obj - movable ?tloc - temploc ?lrobot - pose ?gp - grasp)
		:precondition (and
                    (IsGP ?lrobot ?obj ?gp)
                    (InManip none none_gp)
					(RobotAt ?lrobot)
                    (ObjAtTemp ?obj ?tloc)
        )
		:effect (and
                    (not (InManip none none_gp))
                    (InManip ?obj ?gp)
					(not (ObjAtTemp ?obj ?tloc))
                    (forall (?p - pose) (not (obstructsTemp ?obj ?tloc ?p)))
        )
	)

    (:action place
		:parameters (?obj - movable ?loc - location ?lrobot - pose ?gp - grasp)
		:precondition (and
                    (IsAccessPointFor ?lrobot ?obj ?loc)
                    (InManip ?obj ?gp)
					(RobotAt ?lrobot)
        )
		:effect (and
                    (ObjAt ?obj ?loc)
                    (InManip none none_gp)
                    (not (InManip ?obj ?gp))
        )
	)

    (:action place
		:parameters (?obj - movable ?tloc - temploc ?lrobot - pose ?gp - grasp)
		:precondition (and
                    (IsAccessPointForTemp ?lrobot ?obj ?tloc)
                    (InManip ?obj ?gp)
					(RobotAt ?lrobot)
        )
		:effect (and
                    (ObjAtTemp ?obj ?tloc)
                    (InManip none none_gp)
                    (not (InManip ?obj ?gp))
        )
	)
)
