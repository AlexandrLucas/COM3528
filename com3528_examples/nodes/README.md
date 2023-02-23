# MiRo Specifications

The following documentation will provide an overview of the general topics and the format of the data in the topic that may be found in the MiRo.

## Sensors

### Microphones
MiRo consists of 4 microphones around its body (left, right, center, tail) that are specced as follows: 
* 16-bit @ 20kHz

The message publishes the sampling rate and the data that has been recorded and published in the following topic, `/$robot$/sensors/mics`.

### Cameras

MiRo consists of two cameras, left and right, that can be specced as follows:
* 1280 x 720 @ 15fps
* 640 x 360 @ 25fps
* 320 x 240 @ 35fps

The subscriber for the MiRo's left and right camera is dependent on the following topics, `/$robot$/sensors/camr` and `/$robot$/sensors/caml`. The messages are recorded as a ros image data which can be converted into an opencv2 format in the callback function through the use of CvBridge provided in ROS.

### Sonar

MiRo consists of 1 sonar at its nose that is capable of detecting from a range of 3cm to 1m. The data for the sonar can be subsribed to the following topic, `/$robot$/sensors/sonar`. The data for this ros message is based on the following,
* field_of_view
* min_range: least distance set that the MiRo will be able to detect
* max_range: most distance set that the MiRo will be able to detect
* range: the range of objects detected by MiRo

### Touch

MiRo consists of 28 capacitive touch sensors in its head and body. This is divided into 14 sensors for its body and 14 sensors for its head. The data for the following topics can be subscribed to `/$robot$/sensors/touch_body` and `/$robot$/sensors/touch_head`. The data obtained from the touch sensors is based on binary operations where each of the bit defines whether the sensor detects touch or not. This is then published as a message in terms of decimal.

### Light

MiRo consists of 4 light sensors spread around its body. The message consists of the light intensity in the following order [front, right, left, back] that is published into the topic `/$robot$/sensors/light`.

### Cliff

MiRo consists of two cliff sensors at the left and right front edge of its body. This sensors measure the distance between the MiRo and the floor and it has been published into the topic `/$robot$/sensors/cliff`.

### Motion

MiRo consists of opto sensors in each wheel. This could be used to measure the speed of each wheel and the message is set in the following order, [left, right]. This is then published into the topic `/$robot$/sensors/wheel_speed_opto`.

### Position

MiRo consists of position sensors that can be used to identify the position of the kinematic joints of the MiRo. The position of the MiRo goes in the order [tilt, lift, yaw, pitch] and the messages has been published in the topic `/$robot$/sensors/kinematic_joints`.

### Accelerometer

MiRo consists of accelerometer in its body and head. This would measure the acceleration of the MiRo based on its movement in the x,y,z plane with the messages being published to the topics `/$robot$/sensors/imu_head` and `/$robot$/sensors/imu_body`.

### Voltage

MiRo consists of a voltmeter for its battery with the messages being published to `/$robot$/sensors/battery`.
## Actuators

### Main Wheels

MiRo can move the main wheels based on the topic `/$robot$/control/cmd_vel`. In the ROS messages, the linear is able to set the velocity of the MiRo based on the quaternion angles. This can be converted into euler angles where changes to the x axis can be made for the MiRo to move forward in a straight line. As for the angular, it will make changes to the angular velocity based on the angular distance of the MiRo in the x,y,z plane.

### Body Joints

MiRo can move the joints based on its neck to its body based on the topic `/$robot$/control/kinematic_joints`. This would change the joints based on either its position or velocity and has the following joints for adjustments in the following order, [tilt, lift, yaw, pitch].

### Cosmetic Joints

MiRo can move the cosmetic joints for its eyes, ears, and tail based on the positional data from the topic `/$robot$/control/cosmetic_joints`. The data for this is set in the following order in the array, [tail pitch, tail yaw, left eye, right eye, left ear, right ear].

### Illumination

The illumination for the MiRo consists of six LED lights in the body and this is dependent on the topic `/$robot$/control/illum`. The color code for each of these is dependent on android graphics where each of these color space can be changed in the data for the message. The message is dependent on the following array in the data, [left top, left middle, left bottom, right top, right middle, right bottom].

### Sound Output

The sound output of the MiRo is dependent on the topic `/$robot$/control/stream`. This can be used to produce sound on the miro based on the messages being published, which would include the sampling rate and the data to be produced by the MiRo.