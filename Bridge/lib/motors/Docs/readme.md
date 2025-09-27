/*
CHANGES TO BE MADE && NOTES
PINOUT:
Num 	Label 	  Description
1 	    9 - 24V   Power Supply, +
2 	    PGND 	  Power Supply, GND
3 	    OUT1 	  Motor1_+
4 	    OUT2 	  Motor1_-
5 	    OUT3 	  Motor2_+
6 	    OUT4 	  Motor2_-
7 	    ENA 	  Motor1 PWM
8 	    IN1 	  Motor1 control signal
9 	    IN2 	  Motor1 control signal
10 	    ENB 	  Motor2 PWM
11 	    IN3 	  Motor2 control signal
12 	    IN4 	  Motor2 control signal
13 	    +5V 	  Voltage Reference Input, +5V OR 3.3V

CONTROL METHOD:
IN1 	IN2 	ENA/ ENB 	Motor1/2 Behavior
0 	    0 	    x 	        Stop (brake)
1 	    1 	    x 	        Vacant
1 	    0 	    1 	        Forward 100%
0 	    1 	    1 	        Reverse 100%
1 	    0 	    PWM 	    Forward at PWM speed
0 	    1 	    PWM 	    Reverse at PWM speed

In this table

    IN1 & IN2: the control signal input to change the motor behavior
    "0": TTL_Low
    "1": TTL_High
    "x": Any TTL, and it is default TTL_Low while no PWM signal.
    "ENA/ ENB": PWM speed setting


Note:

IN1 & IN2:
To protect your motor, before switching the motor drive direction, 
first BRAKE the motor by setting IN1=0 & IN2=0. This is especially 
important when the PWM is set to 100% (full speed). The suggested 
braking time is >0.1sec, depending on your motor.

+5V:
This signal is a reference that must be set to the same power supply 
that your microcontroller operates on. Connect this to the 3.3v or 5v 
power supply used by the controller.

*/