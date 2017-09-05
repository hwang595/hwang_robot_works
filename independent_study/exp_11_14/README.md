# force pad sensor reader
This is a simple sample code to drive the force pad sensor for Arduino board. To use this code, you need first to see the hardware connection for this particular force sensor (please refer to this datasheet https://cdn.sparkfun.com/datasheets/Sensors/ForceFlex/2010-10-26-DataSheet-FSR400-Layout2.pdf).

Note that, I did some calibration for this sensor, but it's not totally accurate, if your application doesn't require the accuracy, then it's fine. Otherwise, you may need to consider to use other force sensor or do the calibration by yourself again.

To use this code, simply open it in your Arduino IDE, then download the code to your board and run it. Then in the GUI for serial port embedded in Arduino IDE, you will see the data from the force sensor.
