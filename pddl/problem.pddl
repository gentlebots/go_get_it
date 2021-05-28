( define ( problem problem_1 )
	( :domain go_get_it )

	(:objects 
		room1_zone - zone 
		room2_zone - zone 
		shelf_zone - zone  
		delivery_zone - zone  
		person_zone - zone  
	)

	( :init
		(is_search_zone shelf_zone)
		(is_pick_zone shelf_zone)
		(are_connected room1_zone room2_zone)
		(are_connected room2_zone shelf_zone)
		(are_connected shelf_zone delivery_zone)
		(are_connected delivery_zone person_zone)

        (is_at room1_zone)
	)
	( :goal
		(and
			(is_at person_zone)
			(is_picked_at shelf_zone)			
		)	
	)
)