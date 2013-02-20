#!/bin/bash
rostopic pub -1 /quad0/input/position geometry_msgs/Twist '[0,0,1]' $1

