(define (domain electrification_domain)
    (:requirements :negative-preconditions :disjunctive-preconditions :typing :fluents)

    (:types
        contain movable - object
        robot ball - movable
        robot room - contain
    )

    (:predicates
        (contain_ball ?r - contain)
        (pos ?r - contain ?m - movable)
        (connected ?r1 - room ?r2 - room)
    )

    (:action move
        :parameters ( ?r - robot ?r1 - room ?r2 - room
        )
        :precondition (and
            (pos ?r1 ?r)
            (not (pos ?r2 ?r))
            (or (connected ?r1 ?r2) (connected ?r2 ?r1))
        )
        :effect (and
            (not (pos ?r1 ?r))
            (pos ?r2 ?r)
        )
    )

    (:action pick_ball
        :parameters ( ?r - robot ?b - ball ?r1 - room
        )
        :precondition (and
            (pos ?r1 ?r)
            (pos ?r1 ?b)
            (not (contain_ball ?r))
            (contain_ball ?r1)

        )
        :effect (and
            (not (pos ?r1 ?b))
            (contain_ball ?r)
            (pos ?r ?b)
            (not (contain_ball ?r1))
        )
    )

    (:action drop_ball
        :parameters ( ?r - robot ?b - ball ?r1 - room
        )
        :precondition (and
            (pos ?r1 ?r)
            (pos ?r ?b)
            (contain_ball ?r)
            (not (contain_ball ?r1))

        )
        :effect (and
            (not (pos ?r ?b))
            (contain_ball ?r1)
            (pos ?r1 ?b)
            (not (contain_ball ?r))
        )
    )


        (:action kick_hand
        :parameters ( ?r - robot ?r1 - room ?r2 - room ?b - ball
        )
        :precondition (and
            (pos ?r1 ?r)
            (pos ?r ?b)
            (contain_ball ?r)
            (not (pos ?r2 ?r))
            (not (contain_ball ?r1))
            (not (contain_ball ?r2))
            (or (connected ?r1 ?r2) (connected ?r2 ?r1))
        )
        :effect (and
            (not (pos ?r ?b))
            (contain_ball ?r2)
            (not (contain_ball ?r))
            (pos ?r2 ?b))
        )

        
    (:action kick_ground
        :parameters ( ?r - robot ?r1 - room ?r2 - room ?b - ball
        )
        :precondition (and
            (pos ?r1 ?r)
            (pos ?r1 ?b)
            (not (pos ?r2 ?r))
            (contain_ball ?r1)
            (not (contain_ball ?r2))
            (or (connected ?r1 ?r2) (connected ?r2 ?r1))
        )
        :effect (and
            (not (pos ?r1 ?b))
            (not (contain_ball ?r1))
            (contain_ball ?r2)
            (pos ?r2 ?b)
        )
    )
)