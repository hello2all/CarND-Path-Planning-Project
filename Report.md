# Path Planning Project Report

## Project Highlights
- Prediction is NOT implemented, raw sensor fusion data is used for planning instead
- Car following model is a replicate of the project walk through
- A simple finite state machine with 3 states is implemented to give lane guidance
- Vehicle able finish a lap with out incidentes

## Implementing Path Planning for Autonomous Driving on Highway

### Coordinates Transformation

Raw location measurements are in global cartision coordinates. For easier handling of these measurements, all coordinates are first transformed into vehicle coordinate system and then transformed into Frenet coordinate system. After planning routes using the Frenet coordinates, they are being transformed again back to gobal cartision coordinates.

### Car-Following Model

The ego vehicle follows a simple, linear car following model which has a constant acceleration rate of 5 m/s^2 when no leading vehicle in detection range (30 meters) and a constant deceleration 2.5 m/s^2 when leading vehicle in detection range and the leading vehicle speed is lower than the ego vehicle.

### Lane Change Model

A simple finite state machine (FSM) is implemented as the lane change decision body. The FSM consists of 3 states: Keep Lane, Lane Change Left and Lane Change Right. It takes inputs from sensor fusion data and ego vehicle states to give a single output of the desired lane and it is only activated when an leading vehicle is visible.

State trasition within FSM is decided by taking the action with least cost. The cost function considers 2 aspects of the surrounding vehicles: the gaps from ego vehicle to vehicles ahead, and the gaps from ego vehicle to vehicles behind, for all available lanes. The gaps ahead address the benifit of lane change, while the gaps behind address the feasibility of lane change. The final cost for state trasition is the weighted sum of these two.



