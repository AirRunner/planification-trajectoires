(define (problem electrification_problem)
    (:domain electrification_domain)

    (:objects
        rob - robot
        r1_l r1_r r2_l r2_r r3 r4 r5 r6_l r6_r r7_l r7_r - room
        b1 b2 b3 b4 b5 b6 - ball
    )

    (:init
        (connected r1_l r4)
        (connected r1_r r4)
        (connected r2_l r4)
        (connected r2_r r4)
        (connected r3 r4)
        (connected r5 r4)
        (connected r6_l r4)
        (connected r6_r r4)
        (connected r7_l r4)
        (connected r7_r r4)
        (pos r4 rob)

        (pos r1_l b1)
        (contain_ball r1_l)
        (pos r2_r b2)
        (contain_ball r2_r)
        (pos r3 b3)
        (contain_ball r3)
        (pos r5 b4)
        (contain_ball r5)
        (pos r6_l b5)
        (contain_ball r6_l)
        (pos r7_r b6)
        (contain_ball r7_r)

    )

    (:goal
        (and
        (or (pos r1_l b6))
        (or (pos r2_r b5))
        (pos r3 b4)
        (pos r5 b3)
        (or (pos r6_r b2))
        (or (pos r7_r b1))
        (pos r4 rob)
        )
    )
)