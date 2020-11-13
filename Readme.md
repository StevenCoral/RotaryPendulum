# Rotary Pendulum Project  

## Introduction  
This repository contains files that allows a user to implement the software side of a rotary pendulum system.  
It hold some theoretical data used to derive the state equations, along with some example files from the original sources.  
These files are not very useful and are only kept as reference.  
The main files are:  
* pendulum_arduino.m - a Matlab file that quickly calculates the controller gains needed for the balancing task.  
* s_rotpen_bal.mdl - a Simulink model used to plot the respones of a pendulum under a square-wave rotary arm position reference. It was zipped to protect its binary nature.  
* Arduino\pendulum\pendulum.ino - a C++ source code for downloading into an Arduino (Mega - recommended) that implements a full swing up / balance timed controller.  
  
## Disclaimer
The code is basic and provided as-is, without warranty of any kind. Use at your own risk.  
In general, one should have sufficient knowledge in control theory and optimal control methods in order to understand what's going on.
The system heavily depends on multiple parameters, some of which have to be measured and derived (e.g. the motor constants, gear ratio, encoder CPR, etc.),  
others are just plain difficult to acquire (auch as moments of inertia of various parts).  
Using the given numbers on ANY system different than the one it was tested on will NOT work by any circumstance.


  