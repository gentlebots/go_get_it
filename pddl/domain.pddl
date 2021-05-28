(define (domain go_get_it)
(:requirements :strips :typing :fluents :durative-actions :negative-preconditions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types    
    zone
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates    
	( are_connected ?z_from ?z_to - zone)	
	( is_at ?z - zone)	
	( is_search_zone ?z - zone)	
	( is_found_at ?z - zone)
	( is_pick_zone ?z - zone)	
	( is_picked_at ?z - zone)
);; end Predicates ;;;;;;;;;;;;;;;;;;;;


;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
;;(:functions
  
;;);; end Functions ;;;;;;;;;;;;;;;;;;;;

;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;; 
(:durative-action move_from_to
    :parameters (?from ?to - zone)
    :duration ( = ?duration 5)
    :condition (and
        (over all(are_connected ?from ?to))
        (at start(is_at ?from))
    )
    :effect (and      
        (at end(is_at ?to))
        (at start(not(is_at ?from)))
    )
)

(:durative-action search_at
    :parameters ( ?z - zone )
    :duration ( = ?duration 5)
    :condition (and
        (at start(is_at ?z))
        (at start(is_search_zone ?z))
    )
    :effect (and      
        (at end(is_found_at ?z))
    )
)

(:durative-action pick_at
    :parameters ( ?z - zone)
    :duration ( = ?duration 5)
    :condition (and   
        (over all(is_found_at ?z))  
        (over all(is_pick_zone ?z))  
        (at start(is_at ?z))
    )
    :effect (and
        (at end(is_picked_at ?z)) 
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
