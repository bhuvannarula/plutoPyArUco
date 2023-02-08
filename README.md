# plutoPyArUco

Autonomous Control of Pluto Drone using ArUco Marker(s)

# Introduction

**plutoPyArUco** is further built on top of the **plutoPy** package, and includes PID Controllers & Filtering on ArUco Markers Data.

# Contents

- [Getting Started](#getting-started)
    - Pre-Requisites
    - Installation
    - How to Run
- [Usage Instructions](#usage-instructions)
    - Connection
    - Control System
- [Methodology](#methodology)
    - Pose Estimation
    - Controller

# Getting Started

To understand better how the connection and control system of the drone works please go through the [plutoPy](https://github.com/bhuvannarula/plutoPy) package which is a pre-requisite to this package.

For convenience, plutopy package is already present in this repository.

## Prerequisites

Python 3 (tested on ≥ 3.6)
`scipy`
`opencv-contrib-python==4.6.0.66`

## Installation

Clone the GitHub repository given below,
[https://github.com/bhuvannarula/plutoPyArUco](https://github.com/bhuvannarula/plutoPyArUco)

## How to Use?

Basic Logic is based on setting a coordinate, and PID Controller moving the drone to the set coordinate and keeping it stable at that coordinate (target hold). Further actions have been implemented on top of this code.

For further use, refer to [example_AltitudeHold.py](https://github.com/bhuvannarula/plutoPyArUco/blob/main/example_AltitudeHold.py) or [example_MoveRectangle.py](https://github.com/bhuvannarula/plutoPyArUco/blob/main/example_MoveRectangle.py) scripts.


# Implementations

## Basic Functions

**Set Origin**

To set the origin of the system using the ArUco Marker:

`aruco.setOrigin()`

**Set Target**

To move the drone to a desired target:

`aruco.setTarget(X,Y,Z)`

X,Y,Z are the co-ordinates relative to origin.

## Altitude Hold

Altitude Hold has been implemented in ***example_AltitudeHold.py*** file. Refer to the same for further details.

## Rectangular Motion

A Rectangular Motion of drone has been implemented in ***example_MoveRectangle.py*** file. Refer to the same for further details.

# Methodology

## Pose-estimation

We are using the measurements through ArUco markers and using a low pass filter for X & Y co-ordinates and a kalman filter for the Z axis which fuses the data from Accelerometer as well.

## Controller

We are using a PID controller to control the drone to set the pitch, roll, and throttle based on the error in the Target and the estimated position. 