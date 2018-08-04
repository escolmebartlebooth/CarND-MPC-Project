# Self-Driving Car Engineer Nanodegree Program: MPC Controller
Author: David Escolme
Date: 05 August 2018

---

## Objectives

To code and tune a model predictive controller so that a simulated car can navigate a track with incident.

## Dependencies (taken from project readme)

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Code Structure

The code has 2 files:

* main.cpp
* mpc.cpp

### main.cpp

This file has boilerplate for interaction with the simulator. In addition, the following tasks are coded up to make the MPC controller operate:

* debug flag (integer): if set to 1 a fair few output parameters are sent to the command line
* polyevalpderivative: this function was added to enable a functional solution for calculating the y coordinate for the differential of the fitted polynomial
* the input states (x, y, v, psi, steering angle, throttle setting) are captured as are the reference waypoints
* using trigonometry the reference way points are converted to car coordinates:
  * x = (x-px)*cos(-psi)-(y-py)*sin(-psi)
  * y = (x-px)*sin(-psi)+(y-py)*cos(-psi)
  * using the equations above, each way point is converted and appended to X and Y waypoint vectors
  * a 3rd order polynomial line is fitted to the transformed way points and the coefficients of the line captured
* the initial state in car coordinates is captured - this makes x, y and psi zero. v is the passed value
* errors are calculated for cross-track and angle
* the state and errors are then transformed using the motion model equations to account for the latency in the actuation of 100ms. In essence, the motion models give the state and errors as if the model has moved on by the latency figure
* the state and errors are passed to the mpc solve method along with the polynomial coefficient and the result captured, which contains the predicted next steering angle and throttle setting along with predicted way points from the optimal solution arrived at by the optimiser
* the steering angle is inverted due to the nature of the simulator and constrained to -1 / +1
* finally both the reference and predicted way points and the adjusted steering angle and throttle values are passed back to the simulator


### mpc.cpp / mpc.h

* The mpc file has 3 main parts:
    * variable setup:
      * N and dt: determine the timestep and number of steps to model the trajectory for optimisation. This is a balance of computational time and making dt small enough to allow for a good fit to the reference trajectory
      * Lf and ref_v: Lf is the distance of the car's centre of gravity to the axle and ref_v ensures hat a minimum velocity is part of the cost calculation to avoid the prediction stopping
      * vector position integers: These are helpers to navigate / populate the vector used to hold state and error variables for the N steps of the optimiser and also the cost value
      * tuning constants for the cost value so that each cost can be weighted
    * FG_eval class
      * This class is used to set up a vector to hold:
        * the function cost
        * the state and actuator values at each step
      * The optimiser will use this class to calculate iterations and cost for each iteration so that an optimal set of actuator settings can be calculated based on the lowest cost
      * The cost function comprises squared errors for cross track and angle trajectory error; the velocity; the actuator settings and the rate of change of actuator settings
    * MPC class with the solve function
      * This class sets up the initial state and the lower and upper bounds of each state and actuator setting
      * The optimiser is then called passing the state, constraints and FG_eval - the result (solution) - will contain the least cost next set of predicted way points and actuator settings 


## Discussion

### Getting started

The classroom version of the code was a good starting point for this project. Although some changes were necessary to make the code work with the simulator:
  * the transformation of state and way points to car coordinates
  * the inversion of the steering angle due to the expected input to the simulator
  * use of a 3rd order polynomial cf. the classroom 1st order - the trajectory of the car warranted the 3rd order line fitting
  * adjusting the initial state for latency using the motion model equations

Once these adjustments were made, the car would travel reasonably well for a few seconds before veering off the track. This then led to the need to tune the cost functions and some variable bounds.

### Getting round the track

There were a number of options to tune the vehicle:
* N and dt: how long the optimiser runs for and how discrete each time step is.
* ref_v: what the minimum velocity should be in the cost function so that stopping is avoided
* cost function parameters: weighting of cost function values
* variable bounds: how large/small the actuation values should be allowed to be

Given the hint in the tuning section of the classroom suggested using a tuning factor for steering to dampen erratic turning actuation.

Also, with a max_throttle setting of -1 and +1, it seemed sensible to try and constrain the maximum velocity by limiting the throttle.

This led to some experiments dampening the steering angle and rate of change of steering angle along with limiting the maximum throttle.

After some 10s of experiments, the following settings allowed the car to go round the track:
* N=25; dt=0.05; ref_v=60; max_throttle=0.75
* steering angle dampening = 1000
* steering angle delta = 500
* acceleration dampening = 100
* acceleration delta = 100

The car driving was much smoother than that obtained with the PID Controller in an earlier project but the top speed was about 25mph...could we go faster...? 

### Getting to go fast



## Conclusions

