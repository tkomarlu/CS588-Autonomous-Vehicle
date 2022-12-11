# Final Project: Pedestrian Follower

We leveraged a Object Tracker and PID Controller to make the Autonomous Vehicle follow a selected pedestrian. 

## Approach

* The vehicle first captures the incoming front camera video feed
* The user then draws a bounding box around a specified pedestrian
* The object tracker from [Dlib](https://pypi.org/project/dlib/) allows us to leverage the midpoint and height of the bounding box as input for the PID controller. If the midpoint of the bounding box moves left or right, the PID controller adjusts the steering angle. If the height of the bounding box grows or shrinks, the PID controller reverses or accelerates accordingly. This was implemented with 2 different PID controllers.

## Demo

[![Pedestrian Follower](https://img.youtube.com/vi/Cp3YNAeap5s/0.jpg)](https://www.youtube.com/watch?v=Cp3YNAeap5s)
