(define (domain balls_domain)
    (:requirements :typing :fluents)

    (:types
        contain movable - object
        robot ball - movable
        robot room - contain
    )

    (:predicates
        (contain_ball ?r - contain)
        (not_contain_ball ?r - contain)
        (pos ?r - contain ?m - movable)
        (not_pos ?r - contain ?m - movable)
        (connected ?r1 - room ?r2 - room)
        (same_room ?r1 - room ?r2 - room)
    )

    (:action move
        :parameters ( ?r - robot ?r1 - room ?r2 - room)
        :precondition (and
            (pos ?r1 ?r)
            (not_pos ?r2 ?r)
            (connected ?r1 ?r2)
        )
        :effect (and
            (not (pos ?r1 ?r))
            (not_pos ?r1 ?r)
            (pos ?r2 ?r)
            (not (not_pos ?r2 ?r))
        )
    )

    (:action pick_ball
        :parameters ( ?r - robot ?b - ball ?r1 - room
        )
        :precondition (and
            (pos ?r1 ?r)
            (pos ?r1 ?b)
            (not_contain_ball ?r)
            (contain_ball ?r1)

        )
        :effect (and
            (not (pos ?r1 ?b))
            (not_pos ?r1 ?b)
            (contain_ball ?r)
            (not (not_contain_ball ?r))
            (pos ?r ?b)
            (not (not_pos ?r ?b))
            (not (contain_ball ?r1))
            (not_contain_ball ?r1)
        )
    )

    (:action drop_ball
        :parameters ( ?r - robot ?b - ball ?r1 - room ?r2 - room)
        :precondition (and
            (pos ?r1 ?r)
            (pos ?r ?b)
            (contain_ball ?r)
            (not_contain_ball ?r1)
            (not_contain_ball ?r2)
            (same_room ?r1 ?r2)
        )
        :effect (and
            (not (pos ?r ?b))
            (not_pos ?r ?b)
            (contain_ball ?r1)
            (not (contain_ball ?r1))
            (not_contain_ball ?r1)
            (pos ?r1 ?b)
            (not (not_pos ?r1 ?b))
            (not (contain_ball ?r))
            (not_contain_ball ?r)
        )
    )


        (:action kick_hand
        :parameters (?r - robot ?r1_1 - room ?r1_2 - room ?r2 - room ?b - ball)
        :precondition (and
            (pos ?r1_1 ?r)
            (pos ?r ?b)
            (contain_ball ?r)
            (not_pos ?r2 ?r)
            (not_contain_ball ?r1_1)
            (not_contain_ball ?r1_2)
            (same_room ?r1_1 ?r1_2)
            (not_contain_ball ?r2)
            (connected ?r1_1 ?r2)
        )
        :effect (and
            (not (pos ?r ?b))
            (not_pos ?r ?b)
            (contain_ball ?r2)
            (not (not_contain_ball ?r2))
            (not (contain_ball ?r))
            (not_contain_ball ?r)
            (pos ?r2 ?b)
            (not (not_pos ?r2 ?b)))
        )

    
    (:action kick_ground
        :parameters ( ?r - robot ?r1_1 - room ?r1_2 - room ?r2 - room ?b - ball
        )
        :precondition (and
            (pos ?r1_1 ?r)
            (pos ?r1_1 ?b)
            (not_pos ?r2 ?r)
            (contain_ball ?r1_1)
            (not_contain_ball ?r1_2)
            (same_room ?r1_1 ?r1_2)
            (not_contain_ball ?r2)
            (connected ?r1_1 ?r2)
        )
        :effect (and
            (not (pos ?r1_1 ?b))
            (not_pos ?r1_1 ?b)
            (not (contain_ball ?r1_1))
            (not_contain_ball ?r1_1)
            (contain_ball ?r2)
            (not (not_contain_ball ?r2))
            (pos ?r2 ?b)
            (not (not_pos ?r2 ?b))
        )
    )
)