Sharkopter
==

Sharkopter is a custom nanocopter's project built utterly from scratch. While it is strongly inspired by the crazyfly, 
the only common point is that it uses the same motors, and a similar frame shape.

This repository will contain everything that is needed to build it, and the complete custom made software 
to make it fly. One of the main goals of the software is to provide a simple controller, specifically suited to customly
designed copters for which a precise weight distribution isn't a-priori known.

The control algorithm will at least at first based on a PID algorithm, with tools specially made to automatically optimize
its parameters. All you'll have to do is turn on your custom copter on, and after a short calibration 
period, it should be able to stabilize.

Well, I'm talking, but it's all in my mind for now, let's see :D



Building
==

Frame 
===

- Laser cut the frame from the file doc/nanocopter_frame.svg
