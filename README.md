
# **Encoder Tests** #


----------

This repository contains two Ardunio programs that allow testing of the encoders that come with the stock motors of the DPRG club robot (version 2016).

**encoder\_test-1**  This program tests a single encoder (right motor). The motor will rotate 32920 encoder ticks after a delay, which should be 10 shaft revolutions in the forward direction. After a pause, the motor will rotate -32920 encoder ticks (i.e., 10 revolutions in the reverse direction). The sequence will continually repeat. The serial monitor in the Arduino environment is used to monitor the encoder count values. The baud rate for the serial connection is 115200.

**encoder\_test-2** This program is similar to encoder\_test-1 except it tests both encoders (left and right motors).

**Test Circuit Connections**

The test programs have a list of connections that are required to run the program on the Arduino Mega board in the comments#  # of the program.

**note:** You should not power the mega board from the barrel jack or the +5V pin when using the USB connection.

**Evaluating Results**

The motors should rotate very close to 10 revolutions both ways. Encoder counts should go from ~0 to ~32920 (or ~-32920) and back again to 0. The numbers will be sightly off due to inertia and program timing. If you do not see 10 revolutions when conducting the tests, there is an encoder issue.

We have observed 4 potential issues regarding the encoders:


- The spacing between the sensor and the magnet is too large. To correct this issue carefully reduce the gap between the rotating magnet and the sensor body.

-  The magnet is not aligned with the sensor. To correct this issue move the magnet towards the motor body as far as possible without it hitting the circuit board. This should position the sensors closer to the middle of the rotating magnet.

- The  2 encoder sensors are positioned such that the resulting pulse are not in quadrature. This problem can be caused by a cocked sensor or by slight angular misalignment. A cocked sensor will require removing the board and re-soldering the sensor. If the issue is slight angular misalignment, you can often fix the issue by very carefully shifting the sensor's body to the side.

- A sensor has been installed backwards. You may be lucky and be able to re-install the sensor correctly, however the sensors are polarity sensitive. A new sensor may be required. DPRG has replacement sensors. The correct orientation of the sensor is with the beveled sides pointing towards the magnet.  


**Acknowledgments** 

The teensy encoder library created by pcjr from
[http://www.pjrc.com/teensy/td_libs_Encoder.html](http://www.pjrc.com/teensy/td_libs_Encoder.html) is used in the programs.

The motor shield control functions from the MonsterMoto Shield Example Sketch coded by Jim Lindblom were used to drive the motors in the programs.

Test programs were coded by Doug Paradis (paradug).


