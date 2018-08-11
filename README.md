# Self-Driving Car Engineer Nanodegree Program: MPC Controller

Author: David Escolme

Date: 05 August 2018

Updated: 10 August 2018

---

[//]: # (Image References)

[image1]: Receding_Horizon.JPG "Receding Horizon Control"
[image2]: Motion_Model.JPG "Motion Model"

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
d* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Code Structure

The code has 2 files:

* main.cpp:
  * allows for extended debug messages using a debug flag (set 1 turns the extended debug on)
  * contains the boilerplate code to interact with the simulator and optimiser
  * converts reference way points to car coordinates and fits a 3rd order polynomial to the way points
  * models latency of actuation into the current state
  * calculates cross-track and steering angle errors
  * passes information to the optimiser and receives results back
  * passes the optimised actuator settings and way points to the simulator
* mpc.cpp
  * sets up reference variables N, dt, ref_v and Lf
  * FG_eval class to construct the cost function and optimiser vector
  * MPC class which initialises state and performs the optimisation


## Explanation of the MPC Controller

references beyond the Udacity classroom: https://en.wikipedia.org/wiki/Model_predictive_control

For the self-driving car simulation, MPC takes a motion model for the car and at discrete timesteps samples the model state. The MPC controller uses that modelled state at time step t, together with a reference trajectory to seek an optmimum control strategy over a relatively short time horizon to elicit the next discrete set of actuations at time step t+1. The process is repeated at each subsequent time step with the horizon moving one step forward, thus the method is called a receding horizon control.

![alt text][image1]

Image taken from: https://en.wikipedia.org/wiki/Model_predictive_control

The horizon T is determined by the product of 2 factors: N * dt, where N is the number of time_steps and dt is the sampling distance between 2 steps. It is important to choose N and dt wisely. This is discussed in the tuning section.

For the car the motion model used is:

x(t+1) = x(t) + v(t)*cos(psi(t))*dt

y(t+1) = y(t) + v(t)*sin(psi(t))*dt

psi(t+1) = psi(t) - v(t)/Lf*delta(t)*dt

v(t+1) = v(t) + acc(t)*dt

cte(t+1) = cte(t)  + v(t)*sin(epsi(t))*dt

epsi(t+1) = epsi(t) - v(t)/Lf*delta*dt

where:

x = x position of car

y = y position of car

psi = angle of car

v = velocity of car

cte = the cross-track error (positional error)

epsi = the angular error

![alt text][image2]

and:

delta is the actuation value for steering

acc is the actuation value for acceleration/deceleration

and:

Lf is the distance of the center of gravity of the car from the front of the car

and:

cte(t) = polyeval(coeffs, x(t)) - y(t)

epsi(t) = psi(t) - atan(polyevalpderivative(coeffs, x(t)))

where:

polyeval evaluates a polynomial using coefficients that model the reference trajectory

polyevalderivative evaluates the first derivative of that polynomial


Given the current state at time t and the reference trajectory, the controller can calculate an optimium control strategy by seeking an optimum solution for the motion model based on minimizing a cost function over many iterations on the chosen horizon T. The cost function is evaluated and the minimum cost solution is then used as discussed before to choose the time t+1 actuation values.

The cost function is crucial to achieving good control. For my car model, 7 squared cost factors are added together to create a cumulative model cost:

cte: what is the cross track error

epsi: what is the angular error

v - vref: how far from a desired velocity the solution is

delta: how large is the steering angle

acc: how fast is the acceleration

delta rate of change: how different is the steering angle cf. last value

acc rate of change: how different is the acceleration cf. last value

Each of these can then be weighted so that preference is given to one or more costs and the v_ref figure can also be tuned.

Upper / lower bounds are set on the actuation values to sensible physical values (eg. do not allow a steering angle of 180 degrees), so that the car simulation represents a real world model.

In addition, other dynamic system characteristics can be modelled into the system.

In the case of the car simulator, there is a 100ms actuation delay. This was modelled by advancing the initial state and errors using the motion models and a dt of 100ms.


## Tuning

Once the model was set up, the main task was to tune the horizon, reference velocity and cost function weights so that the car could progress around the track as fast, as smooth as possible without crashing.

### Getting started

The classroom version of the code was a good starting point for this project.

Some changes were necessary to make the code work with the simulator:
  * the transformation of state and way points to car coordinates
  * the inversion of the steering angle due to the expected input to the simulator
  * use of a 3rd order polynomial cf. the classroom 1st order - the trajectory of the car warranted the 3rd order line fitting
  * as mentioned above, adjusting the initial state for latency using the motion model equations

Once these adjustments were made, the car would travel reasonably well for a few seconds before veering off the track. This then led to the need to tune the horizon, cost functions and some variable bounds.

### Getting round the track

There were a number of options to tune the vehicle:
* N and dt: how long the optimiser runs for and how discrete each time step is.
* ref_v: what the minimum velocity should be in the cost function so that stopping is avoided
* cost function parameters: weighting of cost function values
* variable bounds: how large/small the actuation values should be allowed to be

The starting point was to choose plausible starting values for N and dt. T should be as long as possible but dt should be as short as possible. If dt is too large, the model will not be able to follow the trajectory well enough to stablise as the predicted trajectory will not be able to fit well enough to the polynomial modelling the reference way points. Conversely, if dt is too small for a fixed T, then the computation cost increases beyond an acceptable amount.

For example, if we assume a fixed velocity of 30 mph and that the car is travelling in a straight line, then the car will travel the following distances for each given value of dt in seconds:

Distance = Velocity * Cos(0) * dt:

* dt = 0.05; distance = 1.5 metres
* dt = 0.1; distance = 3 metres
* dt = 0.5; distance = 15 metres
* dt = 1; distance = 30 metres

Continuing this theme, for N iterations we would travel:

* N = 10, dt = 0.1, distance = 30 metres
* N = 25, dt = 0.1, distance = 75 metres
* N = 10, dt = 0.05, distance = 15 metres
* N = 25, dt = 0.05, distance = 37.5 metres

I originally chose N = 25 and dt = 0.05. Given that computational time is also important, I changed this to N = 10 and dt = 0.1 after advice from a project reviewer and reviewing the effect of N and dt on the motion model of the car.

Once N and dt were chosen, it was time to look for other tuning parameters.

The hint in the tuning section of the classroom suggested using a tuning factor for steering to dampen erratic turning actuation. Also, with a max_throttle setting of -1 and +1, it seemed sensible to try and constrain the maximum velocity by limiting the throttle to a lower set of bounds.

This led to some experiments dampening the steering angle and rate of change of steering angle along with limiting the maximum throttle.

After some 10s of experiments, the following settings allowed the car to go round the track:
* N = 10; dt=0.1; ref_v=50; max_throttle=1.0
* steering angle dampening = 500
* acceleration dampening = 10

all other tuning weights were left as 1.

The car driving was smoother (to the eye) than that obtained with the PID Controller in an earlier project but the top speed was about 50mph (the reference velocity)...so could we go faster...?


### Getting to go fast

To enable the car to go faster, I conducted a further series of experiments, tuning the set of parameters to try and achieve a faster top speed without crashing.

The first step was to change the cost function for velocity would penalise based on the distance from a higher reference velocity; in this case I chose 150 mph. This setting immediately introduced instability into the car once the speed passed 60 mph.

To reduce the instability, I first decreased dt to shorten the sampling time step for the predicted way points and increased N to keep T constant. The idea here was that increased speed meant for a fixed time step more distance was travelled and so by shortening the timestep, the car might be less vulnerable to large changes in direction. As the reference velocity was tripled, I decreased dt by a similar proportion to 0.03. The car moved further around the track but came off at the first bend.

I then increased both the steering angle and rate of change of steering angle cost factors to try to reduce the abruptness of changes of direction. This led to the car making it across the bridge and then coming off the track.

In the end I was only able to get the car to run against a reference velocity of 70 mph with settings of:

N = 30; dt = 0.03; ref_v = 70; steering_angle_cost = 3000; rate_of_angle_change = 3000;


## Conclusions

The MPC controller provides benefits over PID controllers when the underlying system has dynamic characteristics such as actuator latency. These characteristics can be added to the system model and so become part of the model optimisation problem.

Manual tuning was ok for lower speeds but it would be better to conduct a more rigorous and automatic tuning procedure.

The success of the model was also dependent on the computing power of the Laptop I was using. The simulator was subject to increasing latency with higher resolution graphics.

The controller provided a very smooth (to the eye) driving experience.