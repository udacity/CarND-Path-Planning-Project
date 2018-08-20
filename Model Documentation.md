# Reflection

The model for the Path Planning project is detailed below. The model is capable of travelling over the required 4.32 miles
without incident as is required by the project. The one situation in which the model has been found to possibly be involved
in an incident is when a different car either directly crashes into the vehicle, another car cuts off the vehicle, or
another car is driving between lanes due to issues in the spline path.

## Base Concept

The model has the following three main components:
 
  1. Speed Controller: A controller for determining spacing behind other vehicles.
  2. Lane Controller: A controller for deciding when lanes should be shifted.
  3. Path Controller: A controller for the speed direction of the future path.
  
The first two controllers use information about nearby vehicles to inform the third controller of the current world state.
The third controller then uses that world state to reasonably navigate the world within the predefined limits of the
project.

Below is a more in depth explanation of each system.

## Seed Controller

The Speed Controller takes location data from the sensor fusion set to determine first if there are any vehicles
in the current lane. If the sensors find a vehicle the controller then determines the distance from the car to it. If
the vehicle is too close to the car the controller will proportionally decrease speed to match the other vehicle's speed.
On the other hand, the spacing controller will gradually increase the current speed if no vehicles are currently found in
front of the car and report the new speed to the Path Controller.

## Lane Controller

The Lane Controller decides whether or not it may be useful to actively change lanes. The first step the controller takes
is to check and see if a vehicle is within a reasonable range of the car in the same lane to necessitate a lane change. If
no vehicle is found within the proper range the controller will report that no lane change should occur. Otherwise, the
controller checks adjacent lanes within a certain range to see if there is a reasonable chance that passing will be
effective. If so it reports to the Path Controller.

## Path Controller

Taking the data from the Speed and Lane Controllers, this controller then decides on new path nodes that can accomplish
both the speed changes in the Speed Controller and the lane changes in the Lane Controller. When the system first starts
it generates a path of nodes representing roughly three fourths of a second. Given that producing a large number of nodes
takes considerable amounts of time the system culls any nodes that have already been passed. The controller then only
adds enough nodes to refill the quarter of a second buffer. As the speed value increases the new nodes will have a larger
larger distance between them. The controller will select a destination in another lane when a change of lane is indicated
and will then create a spline between the current end of the node path and the destination location.

## Other Information

It was found that the Frenet coordinates had a slight leftward drift during much of the course so the lane size was
increased to account for this. One major issue found in earlier versions is that the vehicle had issues when two other
vehicles were pacing each other in front of the car. In this circumstance the car could become confused and attempt to
shift back into its previous lane while only partially through the current lane shift. This was overcome by increasing
the distance necessary to qualify as a legitimate lane shift and restricting lane shifts to once a second.

