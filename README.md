# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

# Result

The result is below.  
Please click on the images if you want to see the video:

<a href="http://www.youtube.com/watch?feature=player_embedded&v=46IMkSaVCKA" target="_blank">
<img src="http://img.youtube.com/vi/46IMkSaVCKA/0.jpg" alt="PID Control" width="480" height="360" border="10" />
</a>

# Note 

In order to avoid build failure, I changed "main.cpp".

~~~~
/Users/hideto.kimura/carnd/carnd_term2/carnd_term2_notebook/CarND-MPC-Project/src/main.cpp:84:5: error: no matching member function for call to 'onMessage'
  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                 ^
/usr/local/include/uWS/Group.h:69:10: note: candidate function not viable: no known conversion from '(lambda at
      /Users/hideto.kimura/carnd/carnd_term2/carnd_term2_notebook/CarND-MPC-Project/src/main.cpp:84:15)' to 'std::function<void (WebSocket<true> *, char *, size_t, OpCode)>'
      (aka 'function<void (WebSocket<true> *, char *, unsigned long, uWS::OpCode)>') for 1st argument
    void onMessage(std::function<void(WebSocket<isServer> *, char *, size_t, OpCode)> handler);
                                                          ^
/usr/local/include/uWS/Group.h:69:10: note: candidate function not viable: no known conversion from '(lambda at
      /Users/hideto.kimura/carnd/carnd_term2/carnd_term2_notebook/CarND-MPC-Project/src/main.cpp:84:15)' to 'std::function<void (WebSocket<false> *, char *, size_t, OpCode)>'
      (aka 'function<void (WebSocket<false> *, char *, unsigned long, uWS::OpCode)>') for 1st argument
~~~~

~~~~
h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
ws.send()
↓ (changed to pointer)
h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
ws->send()
~~~~

## The Model

~~~
Kinematic models are simplifications of dynamic models that ignore tire forces, gravity, and mass.
~~~
![vehicle_coord](https://github.com/HidetoKimura/carnd_mpc_project/blob/master/vehicle_coord.png)

~~~
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
~~~

### state
x, y - Position  
psi - Orientation  
v - Speed  

### actuators
delta - Steering angle  
a - Acceleration (throttle/brake combined)  

### error
cte - Cross Track Error  
epsi - Orientation Error  

### environment value
Lf - Distance between the front of the vehicle and its center of gravity  



## Timestep Length and Elapsed Duration (N & dt)

~~~
T is the product of two other variables, N and dt.

N is the number of timesteps in the horizon. dt is how much time elapses between actuations. For example, if N were 20 and dt were 0.5, then T would be 10 seconds.
Horizon
In the case of driving a car, T should be a few seconds, at most. Beyond that horizon, the environment will change enough that it won't make sense to predict any further into the future.

Number of Timesteps
The goal of Model Predictive Control is to optimize the control inputs: [δ,a]. An optimizer will tune these inputs until a low cost vector of control inputs is found. The length of this vector is determined by N:

Thus N determines the number of variables the optimized by MPC. This is also the major driver of computational cost.

Timestep Duration
MPC attempts to approximate a continues reference trajectory by means of discrete paths between actuations. Larger values of dt result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory. This is sometimes called "discretization error".

A good approach to setting N, dt, and T is to first determine a reasonable range for T and then tune dt and N appropriately, keeping the effect of each in mind.
~~~

~~~
size_t N = 12;
double dt = 0.05;
~~~

## Polynomial Fitting and MPC Preprocessing

1. Set N and dt.
2. Fit the polynomial to the waypoints.
3. Calculate initial cross track error and orientation error values.
4. Define the components of the cost function (state, actuators, etc). 
5. Define the model constraints. These are the state update equations defined in the Vehicle Models module

~~~
void conv_vehicle_coordinate(vector<double> &ptsx, vector<double> &ptsy, double px, double py, double psi, 
Eigen::VectorXd &wx, Eigen::VectorXd &wy) {

  for (int i=0; i < wx.size() ; ++i){
    wx(i) =  cos(psi) * (ptsx[i] - px) + sin(psi) * (ptsy[i] - py);
    wy(i) = -sin(psi) * (ptsx[i] - px) + cos(psi) * (ptsy[i] - py);  
  } 
  return ;
}  
~~~
~~~
          conv_vehicle_coordinate(ptsx, ptsy, px, py, psi ,wx, wy);

          auto coeffs = polyfit(wx, wy, 3);

          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);
          
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;
          Output out = mpc.Solve(state, coeffs);
~~~


~~~
// Weights
const double w_cte = 1.0;
const double w_epsi = 1.0;
const double w_v = 1.0;
const double w_delta = 1.0;
const double w_a = 10.0;
const double w_delta_diff = 500.0;
const double w_a_diff = 1.0;

    // The part of the cost based on the reference state.
    for (int i = 0; i < N; i++) {
      fg[0] += w_cte * CppAD::pow(vars[cte_start + i] - ref_cte, 2);
      fg[0] += w_epsi * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
      fg[0] += w_v * CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int i = 0; i < N - 1; i++) {
      fg[0] += w_delta * CppAD::pow(vars[delta_start + i], 2);
      fg[0] += w_a * CppAD::pow(vars[a_start + i], 2);
    }
    // Minimize the value gap between sequential actuations.
    for (int i = 0; i < N - 2; i++) {
      fg[0] += w_delta_diff * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += w_a_diff * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }
    :
    :

    fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
    fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
    fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
    fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
    fg[2 + cte_start + i] =
        cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
    fg[2 + epsi_start + i] =                  
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
~~~


## Model Predictive Control with Latency
~~~
          const int latency_index = 2;

          double steer_value = out.delta.at(latency_index);
          double throttle_value= out.a.at(latency_index);
~~~

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
