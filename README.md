# physics
Physics for robotics, with focus on mechanics, rigid body motion, aerodynamics, and so on.

## Work In Progress

Current status:

* Makefile is clean and compiling.
* Matrix libraries are added.
* Vector libraries are added, wip.
* Euler forward for translation and rotational motion is written.
* Quaterion/Euler angles/Rotation matrix transformations ara added.
* Time is added, running on a loop in main.c, with fixed discrete time stamps.

General TODOs:

* Add dynamics for translational and rotational motion (2nd Order Newton-Euler).
* Add random white noise to DT_S (discere time stamps).
* Improve dynamics with aerodynamics, elasticity, and other factors.
* Add controls library for controling the motion.
    * Classical PID with desired bandwidth and damping characteristics.
    * More advanced methods for aerial robotics (e.g. SE(3), quaternion based, etc).
    * Other recent studies (INDI, ANDI, etc).
* Currently only point mass is considered. Consider other types.
* Add actuator dynamics and their controls too.
* Add sensor models (noise, bias) and sensor fusion libraries
* Where and how to seperate aerial robotics with other robots (e.g. robotic arms, automobies) in a clean, readable and simple way?
* Robotic arm: forward/inverse kinematics and dynamics.
* Humanoids?
* Add better visualization and post-processing (graphs, output files).

You can compile and test them as in the following:

* Clean up

```sh
make clean
```

* Compile

```sh
make
```

* Tests

```sh
make test
```

* Run

```sh
./main
```