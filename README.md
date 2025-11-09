This is an Arduino simple 2-axis robot hand.
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

The servo and stepper combined drains so much energy if connected to Arduino 5V,
so use a different external 5V input.I use an open cable USB and joined both coms with a breadboard.
The joystick doesn't take much power,so connect it to Arduino original 5V.
If you try connecting all three to Arduino's 5V,the arduino willn't turn on,dying because no enough power to keep all 4 on!
