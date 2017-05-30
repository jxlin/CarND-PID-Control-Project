# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

**This fork has been heavily modified by olala7846@gmail.com**

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Reflections

This project is all about PID controller. 

### PID concept

* The P(proportional) term allows the car to handle all angles of turns so it won't fall out of the lane directly when it comes to sharp right or left (which will happen when using constant turning rate).
* The D(differential) term stops the car from overshooting and oscillating while turning.
* The I(integral) term helps to compensate the system error (e.g. wrongly calibrated steering) or environmental errors like side wind, or skewed roads.

### Parameter tuning

I started with the twiddle (coordinate descent) algorithm but soon found some problems.

1. The simulator system is stochastic: the simulator was connected to the PID program through a web socket, so many factors could affect the training process. For example, while screen recording on my computer the simulator could react slower to my PID steering commands causing the error to become larger.

2. The search space is very large: there are infinite combinations of parameters and results, the car could bump into a tree or fall into water, it is hard to defined which is better, thus the training process could easily be stuck into some local minimum(e.g. in favor of bumping into a tree near the lane then falling info water far away which in both cases are unacceptable)
3. Parameters are not on the same scale: Ki, Kp and Kd are in different scale, 0.1 difference on differential term may mean a lot but 0.1 difference on the integral term may cause the car to make a U-turn entirely. 

So I ended up training with gradient descent and with following steps.

* Start training with very short distance and slowly increase it during training.
* Define a custom function to let the system learn the correct behavior.
* Handpicked different Delta value for different dimensions.

#####The training process looks like this.
[![IMAGE ALT TEXT](http://img.youtube.com/vi/Z1yGxkrT6uY/0.jpg)](http://www.youtube.com/watch?v=Z1yGxkrT6uY "Training In Action")

#####The results looks like this.

[![IMAGE ALT TEXT](http://img.youtube.com/vi/vydjPrH3dGY/0.jpg)](http://www.youtube.com/watch?v=vydjPrH3dGY "Video Title")