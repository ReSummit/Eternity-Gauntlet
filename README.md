# Eternity Gauntlet
An IEEE QP++ Project that aims to replicate the actions of one's arm.

Detailed documentation with instructions on how to reproduce our project: https://docs.google.com/document/d/18Sg7S5h8ZwhW116lG7X9LGoN67Ntuahs0ENhkMbb-kE/edit?usp=sharing

Different sections of project: gyros, flex sensors, servo testing (relaying data to this to control the servos)

Current Progress:
(6/2/2019) Final adjustments to wiring and clean code. Project complete!

Past Updates:
(6/1/2019) Calibrated gyro sensor with servo motor for lateral sweeping movement.
(5/31/2019) Calibrated individual flex sensors to their respective 3D printed fingers.
(5/29/2019) Attached flex sensors to the glove and strung fishing line between servos and fingers.
(5/27/2019) Moved wiring to smaller breadboard and finish combining 3D printed components
(5/25/2019) Combined flex sensor code with gyro code. Both types of sensors are able to control assigned servos through PWM controller.
(5/23/2019) Flex sensors are working and can make each of the 5 servos move, but no integration in gyros and servos with the flex sensors yet. Will eventually move to attaching components to glove and using solder to connect components.
(5/23/2019) Issues so far: Need to send a char to run (gyroscope) program, need to unplug and replug USB periodically (for STM32)
