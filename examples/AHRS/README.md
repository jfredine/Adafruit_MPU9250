# AHRS
This is example code for use of an MPU9250 9-DoF accelerometer, gyro,
and magnetometer.  In order for it to work well the magnetic field must be
calibrated.  Use the calibration example and the steps layed out in the
in the [Adafruit AHRS Motion Sensor Fusion Overview](https://learn.adafruit.com/how-to-fuse-motion-sensor-data-into-ahrs-orientation-euler-quaternions/overview)
to do this.  Once calibrated this example will output orientation data usable
with the [Adafruit 3D Model Viewer](https://adafruit.github.io/Adafruit_WebSerial_3DModelViewer/)
