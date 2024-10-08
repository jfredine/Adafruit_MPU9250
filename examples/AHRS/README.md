# AHRS
This is example code for use of an MPU9250 9-DoF accelerometer, gyro,
and magnetometer.  It follows the steps layed out in the [Adafruit AHRS Motion
Sensor Fusion Overview](https://learn.adafruit.com/how-to-fuse-motion-sensor-data-into-ahrs-orientation-euler-quaternions/overview)
with the code for both calibration and AHRS steps included.  The macro
CALIBRATION should be set during compilation to complete the calibration step
and once calibration is complete the code should be built again without this
macro to output orientation data usable with the [Adafruit 3D Model Viewer](https://adafruit.github.io/Adafruit_WebSerial_3DModelViewer/)
