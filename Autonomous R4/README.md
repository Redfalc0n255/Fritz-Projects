An autonomous four-wheel robotic platform designed and built from the ground up.
Created by Ben Fritz

**Overview**


Auto R4 is a fully custom autonomous robotic vehicle developed as a self-directed engineering project.
The goal of the project is to design a mobile platform that can navigate independently, process sensor data in real-time, and integrate mechanical, electrical, and software systems into something that wont fall apart after using it twice. Thus I want all wires connections to be either soldered, crimped, or developed with PCB.


**Core Objectives**


- Build a stable and modular robotic platform capable of differential drive(Using esp32 PWM) and lidar based navigation, using a speaker to talk with enviornment.
- Learn! I'd like to have this project teach me everything neccessary to devlop real world skills like soldering, design, and CAD modeling.
- Multimode Model, I'd like to have a button implmeneting two different drive mechanisms, autonomous mode and pilot mode.

  
**System Architecture**


Control System:	ESP32 Dev Module C - The brain of the project, has 30 pins to support all other boards and connections

Power System:	20 V DeWalt Battery powering the system, will need to use buck adapters to get the voltage to 12V and 5V for the motors and logic respectively

Motor Drivers:	Dual TB6612FNG drivers controlling four SGM25-370 DC motors (approx. 399 RPM), use tank steering.

Audio System:	MAX98357A I2S amplifier for audio feedback.

Sensors:	TF-Mini LiDAR for obstacle detection, mounted on a servo for scanning.

Mechanical Design:	Custom wooden frame, see designs in Onshape, using 3D printed mounting brackets for servos and lidar.


**Component List**

1 ESP32 Dev Module: Central microcontroller; handles logic and control loops.

2 TB6612FNG Motor Drivers:	Independent motor control for each wheel.

4 SGM25-370 Motors:	Four-wheel drive for torque and traction.

1 MAX98357A Audio Amplifier:	Generates digital-to-analog audio output for system feedback.

2 MP1584EN Buck Regulator:	Steps 12 V → 5 V for sensors, ESP32, and peripherals.

1 DeWalt 20V Battery Adapter:	Primary power source, high-capacity (5AH).

1 TF-Mini LiDAR Sensor:	Distance sensing and object avoidance.

1 MG996R Servo:	Rotates LiDAR for scanning a 180° FOV.

9 100 µF Capacitors:	Used for rail stabilization on 5 V and 12 V lines.

4 Protoboard Boards: Soldered together as a large custom CB.

1 Frame: Wooden Frame Approx 10"x17", drilled holes for mounting componenets.



For images and schematics, see other files in github.
