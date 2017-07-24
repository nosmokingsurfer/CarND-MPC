# Project Rubric

## Your code should compile.
* The project is simply build with cmake and make commands

## The Model
The model I used has the state dimension equal to six
* x and y coordinates to fix the position of the car in global reference frame
* psi angle to set the orientation with reference to global coordinate frame
* CTE value - the cross track error between car position and cubic spline fitted to the goal trajectory key points
* CTE_psi - the angle between the car global orientation and local tangent line to cubic spline of the goal trajectory

The model has two actuation signals
* longitudal acceleration signal. In range [-1, + 1]
* steering signal. In range [-1, +1]

The state update equations are as follows:


      x(i+1) = x(i) + v(i) * cos(psi(i)) * dt
      y(i+1) = y(i) + v(i) * sin(psi(i)) * dt
      psi(i+1) = psi(i) - v(i)*delta(i)/Lf*dt
      v(i+1) = v(i) + a(i) * dt
      cte(i+1) = (f(i) - y(i)) + v(i)*sin(epsi(i))*dt
      epsi(i+1) = (psi(i) - psides(i)) + v(i) * delta(i)/Lf * dt


Where the x(i), y(i), psi(i), v(i), cte(i), epsi(i) are elements of the state vector at time i, a(i) and delta(i) - control signals at time step (i), dt - the time step between the states (i+1) and (i), Lf - the length between the front and rear wheel axes

f(i) = f(x(i)) = a0 + a1 * x(i) + a2*x(i)^2 + a3*x(i)^3 - spline function in vehicle reference frame coordinates

psides(i) = psides(x(i)) = atan(a1 + 2*a2*x(i) + 3*a3*x(i)^2 - tangent line angle to the trajectory spline function in vehicle reference frame.


## Time step Length and Elapsed Duration (N & dt)
My final result is N = 50 and dt = 0.05. I wanted my model to be able to handle trajectory changes up to 2.5 seconds in the future. And I have practical experience in self-driving cars and I know that it is possible to give control signals at 20Hz frequency i.e. every 50ms. That resulted the N = 50 and dt = 0.05.

To debug my algorithm I used less control points and larger dt - for example I used N = 5 and dt = 0.5, but still the same 2.5 seconds.

It is better to have more control points - the system handles sharp turns better.

## Polynomial Fitting and MPC Preprocessing

I used the third degree spline to fit the waypoints. No other preprocess procedures were used.

          coeffs = polyfit(xvals, yvals, 3);




## Model Predictive Control with Latency

The latency of the system is dramatic in sharp turns - the goal trajectory image in vehicle coordinates starts shaking and the car hardly stabilizes along the middle line. To handle the latency I simply used not the first pare of the control signals from the Solve() result but the 4th. It is similar to the latency to be equal to  200ms.




# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
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
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
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
