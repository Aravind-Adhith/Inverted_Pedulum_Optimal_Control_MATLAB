# Inverted_Pedulum_Optimal_Control_MATLAB

Course work project for Advanced System Modelling & Control 

## Objective
Analysis of Inverted Pendulum using Optimal Control Techniques in MATLAB.

## Concepts 

- Optimal Control
- Linear Quadratic Regulator
- Proportional Integral Derivative Controller
- Pole Placement
- Linearizing the Non-Linear Dynamics

## Linear system Dyanamics

|Notations|Description|
|:---:|:---:|
|m = 1|Mass of the Pendulum(kg)|
|M = 5|Mass of the Cart(kg)|
|L = 2|Length of the Pendulum(m)|
|g = 10|Gravitational Acceleration(m/s2)|
|d = 1|Damping Coefficient|
|s = 1(At top position)| Position of the Pendulum|

## Pole Placement Results

Three trials were conducted to place the poles at desired locations. The results are shown below.

<img src = resources\Pole-placement-with-output-feedback.png>

### Slow Response | Faster and More Aggressive Response | Very Aggressive Response

<img src = resources\PoleGraphs.PNG>

### Simulation Videos

- Videos for Slow Response can be viewed [here](resources\PolePlace_Trial1.avi)
- Videos for Faster and More Aggressive Response can be viewed [here](resources\PolePlace_Trial2.avi)
- Videos for Very Aggressive Response can be viewed [here](resources\PolePlace_Trial3.avi)

## Linear Quadratic Regulator Results

The LQR function requires a state weighing matrix Q and an actuator weighing matrix R.

The Simulink model for LQR is shown below.

<img src = resources\LQRSimulink.PNG width = 600 height = 400>


The states were observed and plots can be seen below.

<img src = resources\LQRGraph.PNG width = 600 height = 400>

The simulation video can be viewed [here](resources\LQR_Trial.avi)

## Proportional Integrated Derivative (PID) Controller Results

|CL Response | Rise Time | Overshoot | Settling Time | Steady State Error |
|:---:|:---:|:---:|:---:|:---:|
|Kp|Decrease|Increase|Minor Change|Decrease|
|Ki|Decrease|Increase|Increase|Decrease|
|Kd|Minor Change|Decrease|Decrease|No Change|

The states were observed and plots can be seen below.

- For Case 1, Kp = 1, Ki = 1, Kd = 1 : 

<img src = resources\PID_trial_1.jpg width = 400 >

- For Case 2, Kp = 100, Ki = 1, Kd = 1 :

<img src = resources\PID_trial_2.jpg width = 400 >

- For Case 3, Kp = 100, Ki = 1, Kd = 50 :

<img src = resources\PID_trial_3.jpg width = 400 >






