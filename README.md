# Go, get it!

The original rules can be found at the [Rulebook 2011](https://athome.robocup.org/wp-content/uploads/2018/10/2011_rulebook.pdf#section.4.3)

## Flowchart
![go_get_it](https://user-images.githubusercontent.com/22964725/111549156-04228e00-877c-11eb-9f0e-1f32a715a969.png)


## Start of the task - Navigation and Human Interaction

The robot waits the _start signal_: opening the door in front of him.

Then it waits 1 meter from the door to receive instructions from a referee.

The referee tells the robot to which **room** they should go.

Once the room is defined, the robot has to navigate there taking into account that other robot is doing the same with ohter room, so it may be some encounters.

If one of the robots enters the room of the other robot for more than 30 seconds,  or is interfering with the other robot, the robot must be removed from the scenario and cannot continue with this test.

## Indetifying objects - Navigation and Vision

There will be 8 objects around the room. These items are selected from a list of [predefined objects](https://athome.robocup.org/wp-content/uploads/2018/10/2011_rulebook.pdf#subsection.3.2.4). From this list, 4 objects are selected to be identified and other 4 are classified as _unknown objects_ (must be ignored). From the 4 **goal objects**, 2 will be placed on the floor and other 2 at some height, for example, on a table, a chair...

The robot has to face the object it will identify no further than **50 cm** and identify it, for example, saying its name, displaying something on a screen...

## Grasping objects - Manipulation

Once the item is identified, the robot has to grasp it and lifting  it  up  to  at  least 5 cm for more than 10 seconds

## Handing the object to an operator - Navigation and Manipulation

The robot has to return to the location of the referee that told it where to go and handing the object it has just retrieve to him/her

## Leaving the Arena - Navigation

Once the 4 objects are retrieved, the robot has to succesfully exit the arena and finish the test. The test must not take longer than **5 min**

## Deus Ex Machina

The rulebook doesn't specify any possible help to the robot, except a start button, which is punish

## Score

- [ ] Navigating to the room - 200
- [ ] Identifying an object  - 200
- [ ] Grasping the object    - 200
- [ ] Handing it to the user - 300
- [ ] Leaving the arena      - 100

