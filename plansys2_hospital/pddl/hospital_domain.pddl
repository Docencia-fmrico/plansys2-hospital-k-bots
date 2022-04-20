(define (domain robots-nav)
    (:requirements :strips :equality :typing :durative-actions)

    (:types
        room zone corridor - location
        gripper
        robot door object
    )

    (:predicates
        (robot_at ?robot - robot ?loc - location)
        (connected ?from ?to - location)
        (connected_through ?from ?to - location ?door - door)
        (door_open ?door - door)
        (door_closed ?door - door)
        (gripper_at ?gripper - gripper ?robot - robot)
        (gripper_free ?gripper - gripper)
        (object_at ?object - object ?loc - location)
        (robot_carry ?robot - robot ?object - object)
    )

    ;Move between connected locations when there is not door;
    (:durative-action move_directly
        :parameters (?robot - robot ?from ?to - location)
        :duration (= ?duration 5)
        :condition (and 
            (at start (and 
                (robot_at ?robot ?from)
            ))
            (over all (and 
                (connected ?from ?to)
            ))
        )
        :effect (and 
            (at start (and 
                (not (robot_at ?robot ?from))
            ))
            (at end (and 
                (robot_at ?robot ?to)
            ))
        )
    )
    

    ;Move between connected locations when there is door;
    (:durative-action move_through
        :parameters (?robot - robot ?from ?to - location ?door - door)
        :duration(= ?duration 5)
        :condition (and
            (at start (and
                (robot_at ?robot ?from)
                (door_open ?door)
            ))
            (over all 
                (connected_through ?from ?to ?door)
            )
        )
        :effect (and 
            (at start (and 
                (not (robot_at ?robot ?from))
                (robot_at ?robot ?to)
            ))

        )
    )

    (:durative-action open_door
        :parameters (?robot - robot ?from ?to - location ?door - door ?gripper - gripper)
        :duration (= ?duration 5)
        :condition (and 
            (at start (and 
                (gripper_free ?gripper)
                (robot_at ?robot ?from)
                (connected_through ?from ?to ?door)
                (door_closed ?door)
            ))
        )
        :effect (and 
            (at start (and 
                (door_open ?door)
                (not (door_closed ?door))
            ))
        )
    )
    

    (:durative-action close_door
        :parameters (?robot - robot ?from ?at - location ?door - door ?gripper - gripper)
        :duration (= ?duration 5)
        :condition (and 
            (at start (and 
                (gripper_free ?gripper)
                (robot_at ?robot ?at)
                (connected_through ?from ?at ?door)
                (door_open ?door)
            ))
        )
        :effect (and 
            (at start (and 
                (door_closed ?door)
                (not (door_open ?door))
            ))
        )
    )
    
    (:durative-action pick_object
        :parameters (?robot - robot ?object - object ?loc - location ?gripper - gripper)
        :duration (= ?duration 3)
        :condition (and 
            (at start (and 
                (gripper_at ?gripper ?robot)
                (gripper_free ?gripper)
                (object_at ?object ?loc)
                (robot_at ?robot ?loc) 
            ))
        )
        :effect (and 
            (at start (and 
                (not (gripper_free ?gripper))
                (robot_carry ?robot ?object) 
                (not (object_at ?object ?loc))
            ))
        )
    )
    
    (:durative-action drop_object
        :parameters (?robot - robot ?object - object ?loc - location ?gripper - gripper)
        :duration (= ?duration 3)
        :condition (and 
            (at start (and 
                (gripper_at ?gripper ?robot)
                (robot_at ?robot ?loc) 
                (robot_carry ?robot ?object) 
            ))

        )
        :effect (and 
            (at start (and 
                (gripper_free ?gripper)
                (object_at ?object ?loc)
                (not (robot_carry ?robot ?object))
            ))

        )
    )
    
)