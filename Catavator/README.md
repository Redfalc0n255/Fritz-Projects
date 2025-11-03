Start of Catavator Project.
The Catavator is a small motorized elevator built to lift my cat safely to a spot where she can look down on me like the plebian I am.
This project combines basic mechanical design, an ESP32-DevKitC controller, and a custom board that manages motor control, sensors, and power delivery from a 12 V wall supply.
The Catavator uses a 12 V DC gearmotor with high torque (via gearbox) to move a small platform between two levels.
The system detects when a cat steps on the platform using a load cell, and stops automatically at preset endpoints with limit switches.
A simple protoboard consolidates motor driving, power regulation, and safety circuits.


Core Components
Component	                                           Description
ESP32-DevKitC	                                       Main microcontroller for logic, sensors, and control
Motor Driver (TB6612FNG or BTS7960)	                 Drives the 12 V DC gearmotor
12 V DC Gearmotor + Gearbox	                         Provides high torque for smooth lift motion
Load Cell (TAL220B-50 kg)	                           Detects added weight when cat steps on platform
HX711 Amplifier	                                     Reads load-cell signal
Limit Switches	                                     Define top and bottom stops for safety
Relay (SRD-05VDC-SL-C)	                             Optional emergency stop or power cutoff
12 V → 5 V Buck Converter	                           Steps down logic rail voltage for ESP32 and sensors

Ill be updating this as the project continues.
