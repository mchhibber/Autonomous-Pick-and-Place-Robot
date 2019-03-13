# Autonomous-Pick-and-Place-Robot

Introduction:
A line following, autonomous, RGB detecting, pick and place robot is developed on Arduino Atmega 2560 using AVR C programming for the annual e-Yantra Robotics Competion (sponsored by MHRD under NMEICT).

Path Planning and Line following:
    To make the robot work autonomously without any external control the Arena was represented as a grid with the nodes having defined coordinates. The Robot uses Dijkstra's Shortest Path algorithm to plan the path and a direction maintaining algorithm is devised which allows it to take appropiate turns at specific nodes so as to follow the planned path. Line following is achieved through 3-channel white line sensor.
    
Color Detection and Arm :
    The Robot uses TCS-3200 light to frequency convertor sensor to detect color of the object infront. Red, Green, Blue and Black colors can be detected. Arm is controlled by servo motors making specific degrees turn to allow for pick and place.
    
Lift:
    The lift uses Arduino Nano alongwith IR sensor and 2-limit switches. Limit switches and IR sensors are fed as interrups hence are used to control the lift motor accordingly.
