(define (problem balls_problem)
    (:domain balls_domain)

    (:objects
        rob - robot
        r1_l r1_r r2_l r2_r r3 r4 r5 r6_l r6_r r7_l r7_r - room
        b1 b2 b3 b4 b5 b6 - ball
    )

    (:init
        (connected r1_l r4) (connected r4 r1_l)
        (connected r1_r r4) (connected r4 r1_r)
        (connected r2_l r4) (connected r4 r2_l)
        (connected r2_r r4) (connected r4 r2_r)
        (connected r3 r4) (connected r4 r3)
        (connected r5 r4) (connected r4 r5)
        (connected r6_l r4) (connected r4 r6_l)
        (connected r6_r r4) (connected r4 r6_r)
        (connected r7_l r4) (connected r4 r7_l)
        (connected r7_r r4) (connected r4 r7_r)



        (different_room r1_r r2_r) (different_room r1_r r2_l) (different_room r1_r r3)
        (different_room r1_r r4) (different_room r1_r r5) (different_room r1_r r6_l)
        (different_room r1_r r6_r) (different_room r1_r r7_l) (different_room r1_r r7_r)

        (different_room r1_l r2_r) (different_room r1_l r2_l) (different_room r1_l r3)
        (different_room r1_l r4) (different_room r1_l r5) (different_room r1_l r6_l)
        (different_room r1_l r6_r) (different_room r1_l r7_l) (different_room r1_l r7_r)

        (different_room r2_r r1_r) (different_room r2_r r1_l) (different_room r2_r r3)
        (different_room r2_r r4) (different_room r2_r r5) (different_room r2_r r6_l)
        (different_room r2_r r6_r) (different_room r2_r r7_l) (different_room r2_r r7_r)

        (different_room r2_l r1_r) (different_room r2_l r1_l) (different_room r2_l r3)
        (different_room r2_l r4) (different_room r2_l r5) (different_room r2_l r6_l)
        (different_room r2_l r6_r) (different_room r2_l r7_l) (different_room r2_l r7_r)

        (different_room r3 r1_r) (different_room r3 r1_l) (different_room r3 r2_r) (different_room r3 r2_l)
        (different_room r3 r4) (different_room r3 r5) (different_room r3 r6_l)
        (different_room r3 r6_r) (different_room r3 r7_l) (different_room r3 r7_r)

        (different_room r4 r1_r) (different_room r4 r1_l) (different_room r4 r2_r) (different_room r4 r2_l)
        (different_room r4 r3) (different_room r4 r5) (different_room r4 r6_l)
        (different_room r4 r6_r) (different_room r4 r7_l) (different_room r4 r7_r)

        (different_room r5 r1_r) (different_room r5 r1_l) (different_room r5 r2_r) (different_room r5 r2_l)
        (different_room r5 r3) (different_room r5 r4) (different_room r5 r6_l)
        (different_room r5 r6_r) (different_room r5 r7_l) (different_room r5 r7_r)

        (different_room r6_r r1_r) (different_room r6_r r1_l) (different_room r6_r r2_r)
        (different_room r6_r r2_l) (different_room r6_r r3) (different_room r6_r r4)
        (different_room r6_r r5) (different_room r6_r r7_l) (different_room r6_r r7_r)

        (different_room r6_l r1_r) (different_room r6_l r1_l) (different_room r6_l r2_l)
        (different_room r6_l r2_r) (different_room r6_l r3) (different_room r6_l r4)
        (different_room r6_l r5) (different_room r6_l r7_l) (different_room r6_l r7_r)

        (different_room r7_r r1_r) (different_room r7_r r1_l) (different_room r7_r r2_r)
        (different_room r7_r r2_r) (different_room r7_r r3) (different_room r7_r r4)
        (different_room r7_r r5) (different_room r7_r r6_l) (different_room r7_r r6_r)

        (different_room r7_l r1_r) (different_room r7_l r1_l) (different_room r7_l r2_r)
        (different_room r7_l r2_l) (different_room r7_l r3) (different_room r7_l r4)
        (different_room r7_l r5) (different_room r7_l r6_l) (different_room r7_l r6_r)
        


        (pos r4 rob)
        (not_pos r1_l rob) (not_pos r1_r rob) (not_pos r2_l rob) (not_pos r2_r rob) (not_pos r3 rob)
        (not_pos r5 rob) (not_pos r6_l rob) (not_pos r6_r rob) (not_pos r7_l rob) (not_pos r7_r rob)


        (not_pos r1_r b1) (not_pos r2_l b1) (not_pos r2_r b1) (not_pos r3 b1) (not_pos r4 b1)
        (not_pos r5 b1) (not_pos r6_l b1) (not_pos r6_r b1) (not_pos r7_r b1) (not_pos r7_l b1)

        (not_pos r1_l b2) (not_pos r1_r b2) (not_pos r2_l b2) (not_pos r3 b2) (not_pos r4 b2)
        (not_pos r5 b2) (not_pos r6_l b2) (not_pos r6_r b2) (not_pos r7_r b2) (not_pos r7_l b2)

        (not_pos r1_l b3) (not_pos r1_r b3) (not_pos r2_l b3) (not_pos r2_r b3) (not_pos r4 b3)
        (not_pos r5 b3) (not_pos r6_l b3) (not_pos r6_r b3) (not_pos r7_r b3) (not_pos r7_l b3)

        (not_pos r1_l b4) (not_pos r1_r b4) (not_pos r2_l b4) (not_pos r2_r b4) (not_pos r3 b4)
        (not_pos r4 b4) (not_pos r6_l b4) (not_pos r6_r b4) (not_pos r7_r b4) (not_pos r7_l b4)

        (not_pos r1_l b5) (not_pos r1_r b5) (not_pos r2_l b5) (not_pos r2_r b5) (not_pos r3 b5)
        (not_pos r4 b5) (not_pos r5 b5) (not_pos r6_r b5) (not_pos r7_r b5) (not_pos r7_l b5)

        (not_pos r1_l b6) (not_pos r1_r b6) (not_pos r2_l b6) (not_pos r2_r b6) (not_pos r3 b6)
        (not_pos r4 b6) (not_pos r5 b6) (not_pos r6_l b6) (not_pos r6_r b6) (not_pos r7_l b6)


        (not_contain_ball r1_r)
        (not_contain_ball r2_l)
        (not_contain_ball r4)
        (not_contain_ball r6_r)
        (not_contain_ball r7_l)


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

    (:goal (and
            (pos r1_l b6)
            ; (pos r2_r b5)
            ; (pos r3 b4)
            ; (pos r5 b3)
            ; (pos r6_r b2)
            ; (pos r7_r b1)
            (pos r4 rob)
        )
    )
)