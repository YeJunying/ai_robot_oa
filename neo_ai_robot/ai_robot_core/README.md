# Neo AI Robot

Task-based AI Robot that utilizes ROS Actionlib.

## Modules to implement

- Global planner: for global path planning.
  - cloud_planner using bluetooth
- Local planner: for local planning, publishing velocity.
  - simple_local_planner: a very simple local planner used for validating prototype design.
- Action servers
  - movebase server: for robot movement.


## Frame transformation

stju lab

3 mercator points:

- left-bottom: 13519942.879317075, 3614351.288322719
- right-bottom: 13519969.448462347,3614351.2136771833
- left-top: 13519943.030940821, 3614374.5497376984

3 pixel points:

- left-bottom: 40, 20
- right-bottom: 887, 20
- left-top: 40, 762


mediasoc no4 building

- left-bottom: 13519502.288865183, 3635906.128036335
- left-top: 13519474.238551205, 3635954.583817144
- right-bottom: 13519546.644757288, 3635931.981149625
