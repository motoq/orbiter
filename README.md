orbiter
=======

6DOF Orbiter Model

This is a relatively simple six degree of freedom simulation. The
vehicle model is for a spacecraft subject to the gravitational
attraction of a rotating central body. The spacecraft can only thrust
along its longitudinal axis, propelling itself forward - the attitude of
the vehicle must be changed to alter direction. Torques can be applied
about each axis (via thrusters, reaction wheels, magic, take your pick).
Note that gyroscopic coupling results in an acceleration about a third
axis when there are angular rates about two others. In addition, due to
a single plane of symmetry in the spacecraft mass distribution, torques
applied about an axis other than the y-axis (pitch) will induce
accelerations along the other axes. This coupling can make the vehicle
very difficult to control if the angular rates are anything but slow. 

Various attitude control system options exist and their level of
sophistication is being developed, along with both batch an filtered
attitude determination modules.
