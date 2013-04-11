# Quadrotor ARTag-based Navigation

## What is it?

This repository contains code and documentation for a **Gazeb/ROS**
the simulation of a quadrotor navigation system based on ARTag marker
recognition and a basic PD controller.

The quadrotor can be controlled with voice commands through an
Android application and  the Google speech recognition interface.

## Dependencies

This software is compatible with **ROS Fuerte**.

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

In the `QaudrotorCommander` folder there are the Eclipse project for the Android application used to command the quadrotor simulation.

### Speech Recognition and Learning Module

In the folder `quad_speech_control` there are the module that join the Android application with the ROS simulation.

## Installation


[1]: https://code.google.com/p/lcm/