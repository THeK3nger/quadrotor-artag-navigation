# Quadrotor ARTag-based Navigation

## What is it?

This repository contains code and documentation for a **Gazebo/ROS**
simulation of a quadrotor navigation system based on ARTag marker
recognition and a basic PD controller.

The quadrotor can be controlled with voice commands through an
Android application and the Google speech recognition interface.

## Dependencies

This software is compatible with **ROS Fuerte**.

The code makes heavy use of ARTag and the installation of the [CCNY][2] stack
required.

You also need a proper installation of the [LCM][1] communication
server. This software is mandatory for the communication between ROS
and the Android application.

## File Structure

This repository is composed by the following parts:

### The Quadrotor Controller

In the folder `labrob_ros_quadrotor` we can find the core of the
Gazebo/ROS simulation. In this contains the stack for a quadrotor
simulation in Gazebo.

### The Camera Simulation

In the `elective` folder there are the launch files to start the
simulation with the right nodes and the right world configuration.

### Android Software

In the `QaudrotorCommander` folder there are the Eclipse project for the
Android application used to command the quadrotor simulation.

### Speech Recognition and Learning Module

In the folder `quad_speech_control` there are the module that join the
Android application with the ROS simulation.

## Installation

### The Quadrotor Simulation

Copy both the folder `elective` and `labrob_ros_quadrotor` in your ROS workspace.

Compile `labrob_ros_quadrotor` by cd into it and compile it with `rosmake`

To start the simulation you can use one of the two following command:

    roslaunch quadrotor_artag_simulation quad_path.launch
    roslaunch quadrotor_artag_simulation quad_path_noise.launch

### The Android Software

We assume you have an android enabled development environment (more information [here][3]). Import in your IDE the folder `QuadrotorCommander` as a project. To install the application on your devices simply press play. You can then start the application from the launcher on your devices.

### Speech Recognition and Learning Module

TODO: rosmsg gen

To enable the voice control of the quadrotor you first need to add the following line 

    export LCM_DEFAULT_URL="tcpq://127.0.0.1:7700"

to your .bashrc file. Then compile the file `quad_speech_control/LCMServer/LCMServer.java`.
To run the LCMServer, just type:

    java LCMServer

Last you can run the interface between LCM and ros by typing:

    rosrun quad_speech_control controller.py
    rosrun tablet_broker borker.py
    




[1]: https://code.google.com/p/lcm/
[2]: http://www.ros.org/wiki/ccny_vision
[3]: http://developer.android.com/sdk/installing/index.html
