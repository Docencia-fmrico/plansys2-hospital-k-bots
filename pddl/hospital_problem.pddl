(define (problem robots-nav-1)
    (:domain robots-nav)

    (:objects
        Room1 Room2 Room3 Room4 Room5 Room6 Room7 Room8 Room9 Room10 - room 
        Zone1 Zone2 Zone3 Zone4 Zone5 Zone6 - zone
        Corridor1 Corridor2 Corridor3 Corridor4 - corridor 
        kbot - robot
        Door1 Door2 Door3 Door4 Door5 Door6 Door7 Door8 - door


        Elevator1 Elevator2 - elevator
        Gripper1 Gripper2 - gripper
        Object1 Object2 - object
    )

    (:init
        (robot_at kbot Room1)
        (gripper_at Gripper1 kbot)
        (gripper_free Gripper1)
        (gripper_at Gripper2 kbot)
        (gripper_free Gripper2)
        (object_at Object2 Zone2)

        ;;;;;;;;;;;;;;;;;;;;;;;;;;
        ;;      First floor     ;;
        ;;;;;;;;;;;;;;;;;;;;;;;;;;

        ;--Corridor1: Room5, Room4 and Room3
        (connected Corridor1 Room1)
        (connected Room1 Corridor1)
        (connected Corridor1 Room5)
        (connected Room5 Corridor1)

        (connected_through Room4 Corridor1 Door1)
        (connected_through Corridor1 Room4 Door1)

        (connected_through Room3 Corridor1 Door2)
        (connected_through Corridor1 Room3 Door2)

        ;--Room1 and Room2
        (connected Room1 Room2)
        (connected Room2 Room1)


        ;--Room6
        (connected_through Room6 Room5 Door3)
        (connected_through Room5 Room6 Door3)

    

        ;;;;;;;;;;;;;;;;;;;;;;;;;;
        ;;     Room   Zones     ;;
        ;;;;;;;;;;;;;;;;;;;;;;;;;;

        (connected Room3 Zone1)
        (connected Zone1 Room3)

        (connected Room5 Zone2)
        (connected Zone2 Room5)
        (connected Room5 Zone3)
        (connected Zone3 Room5)

        (connected Room8 Zone5)
        (connected Zone5 Room8)

        (connected Room10 Zone4)
        (connected Zone4 Room10)

        (connected Room9 Zone6)
        (connected Zone6 Room9)

        ;;;;;;;;;;;;;;;;;;;;;;;;;;
        ;;     Doors states     ;;
        ;;;;;;;;;;;;;;;;;;;;;;;;;;

        (door_closed Door1)
        (door_closed Door2)
        (door_closed Door3)
        (door_closed Door4)
        (door_closed Door5)
        (door_closed Door6)
        (door_closed Door7)
        (door_closed Door8)
       
        
    )

    (:goal
        (and
            (object_at Object2 Zone3)
            (robot_at kbot Room6)
        )
    )

)