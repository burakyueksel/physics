# Content
Physics for robotics, with focus on mechanics, rigid body motion, aerodynamics, and so on.

I stick to float instead of double, for anticipating future usage of the functions in ressource constrained computational units.

## Work In Progress

Current status:

* Makefile is clean and compiling.
* Matrix libraries are added.
* Vector libraries are added, wip.
* Euler forward for translation and rotational motion is written.
* Quaterion/Euler angles/Rotation matrix transformations ara added.
* Time is added, running on a loop in main.c, with fixed discrete time stamps.
* Dynamics for translational and rotational motion (2nd Order Newton-Euler) are added.
* Controls: Classical PID with desired bandwidth and damping characteristics.
* Sensors: Barometer and IMU models are added.

General TODOs:

* Decide for the matrix library: Standford or own?
* Add quadrotor dynamics and control effectiveness matrix
* Add hexa and octo too
* Add fixed-wing? Let's see.
* Add actuator dynamics and their controls too.
* Add random white noise to DT_S (discere time stamps) for technologies (e.g. controls, sensors, where time integration is done not by the reality but by computers, MCUs, etc).
* Improve publish libraries. Add vector publishing too. Control publishing (and hence logging) rate.
* Improve dynamics with aerodynamics, elasticity, and other factors.
* Add controls library for controling the motion.
    * More advanced methods for aerial robotics (e.g. SE(3), quaternion based, etc).
* Currently only point mass is considered. Consider other types.
* Add sensor models (noise, bias) and sensor fusion libraries.
* Add mono and stereo camera models.
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

* Observe:

    * the terminal output for the published variables. Notice that the altitude is controlled with a simple PID.
    * the log.txt for the output of published variables.