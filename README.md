This is an Ardduino simple 2-axis robot hand.
It uses the 28BYJ-48 stepper motor to spin the shoulder,
and a SG90 servo motor to move it up/down.

Pinout:

Stepper
IN1-D2
IN2-D3
IN3-D4
IN4-D5
GND-External GND
5V-External 5V

Servo
GND-External GND
VCC-External VCC
Signal-D6

Joystick
GND-Arduino GND
VCC-Arduino 5V
VRx-A0
VRy-A1

The servo and stepper drains so muchenergy if connected to Arduino 5V,
so use a different external 5V input.I use an open cable USB.
The joystick doesn't take much,so connect it to Arduino original 5V.
