( define ( problem problem_1 )
	( :domain go_get_it )

	(:objects 
		jarvis - robot 
		sugar_box - object 
	)

	( :init
		(is_target sugar_box)
		(is_free jarvis)
	)
	( :goal
		(and
			(is_delivered sugar_box)
		)	
	)
)