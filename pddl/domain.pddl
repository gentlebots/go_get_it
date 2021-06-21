(define (domain go_get_it)
(:requirements :strips :typing :fluents :durative-actions :negative-preconditions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
    object
    robot
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates
	( is_target ?o - object)
	( is_picked ?o - object)
	( is_found ?o - object)
	( is_free ?r - robot)	
	( is_delivered ?o - object) 	

);; end Predicates ;;;;;;;;;;;;;;;;;;;;


;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
;;(:functions
  
;;);; end Functions ;;;;;;;;;;;;;;;;;;;;

;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;; 
(:durative-action search
    :parameters (?o - object)
    :duration ( = ?duration 5)
    :condition (and
    )
    :effect (and
        (at end(is_found ?o))
    )
)

(:durative-action pick
    :parameters (?r - robot ?o - object)
    :duration ( = ?duration 5)
    :condition (and
        (over all(is_found ?o)) 
        (at start(is_free ?r))
    )
    :effect (and      
        (at end(is_picked ?o))
        (at start(not (is_free ?r)))
    )
)

(:durative-action deliver
    :parameters (?r - robot ?o - object)
    :duration ( = ?duration 5)
    :condition (and   
        (over all(is_target ?o))            
        (at start(is_picked ?o))
    )
    :effect (and
        (at end(is_delivered ?o)) 
        (at end(is_free ?r))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
