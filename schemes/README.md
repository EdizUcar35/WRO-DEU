Electromechanical diagrams
====

This directory must contain one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.

## Object detection scheme

![Object detection scheme](object_detection_scheme.png)

This diagram shows the camera-based obstacle-detection setup used in the project. The Raspberry Pi Zero 2 W receives the image stream from the Raspberry Pi Camera Module 3 through the ribbon cable, processes the frame, and uses the result to support the robot's obstacle-handling logic.
