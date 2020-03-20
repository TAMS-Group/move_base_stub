Move Base Stub
==============

A small node that can be started to have a lightweight simulation of the `move_base`/`amcl` system.

It publishes transforms and estimated poses as [amcl](https://github.com/ros-planning/navigation/tree/melodic-devel/amcl) would to it.

It provides message and action interfaces to communicate with the standard [move_base node](https://github.com/ros-planning/navigation/tree/melodic-devel/move_base).

The implementation directly warps the robot to the goal location without a time delay.

Patches for more extensive simulation are welcome.
