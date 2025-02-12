#include <Arduino.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_MPU9250.h>

#ifdef TARGET_RASPBERRY_PI_PICO
auto &serial = Serial1;
#else
auto &serial = Serial;
#endif

// select either EEPROM or SPI FLASH storage:
#ifdef ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM
Adafruit_Sensor_Calibration_EEPROM cal;
#else
Adafruit_Sensor_Calibration_SDFat cal;
#endif

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

Adafruit_MPU9250 mpu9250;

int init_sensors(void);
void setup_sensors(void);

// Full orientation sensing using NXP/Madgwick/Mahony and a range of 9-DoF
// sensor sets.
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// to Adafruit Unified Sensor interface

// pick your filter! slower == better quality output
// Adafruit_NXPSensorFusion filter; // slowest
// Adafruit_Madgwick filter;  // faster than NXP
Adafruit_Mahony filter;  // fastest/smalleset

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10
//#define AHRS_DEBUG_OUTPUT

uint32_t timestamp;

void setup() {
    serial.begin(115200);
    while (!serial) delay(10);
    delay(1000);

    if (!cal.begin()) {
        serial.println("Failed to initialize calibration helper");
    } else if (!cal.loadCalibration()) {
        serial.println("No calibration loaded/found");
    }

    int retval = init_sensors();
    if (retval == 1) {
        serial.println("Failed to find MPU9250");
        while (1) delay(10);
    } else if (retval == 2) {
        serial.println("Failed to identify MPU9250");
        while (1) delay(10);
    } else if (retval == 3) {
      serial.println("Failed to identify AK8963");
    }

    accelerometer->printSensorDetails();
    gyroscope->printSensorDetails();
    magnetometer->printSensorDetails();

    setup_sensors();
    filter.begin(FILTER_UPDATE_RATE_HZ);
    timestamp = millis();

    Wire.setClock(400000);  // 400KHz
}

void loop() {
    float roll, pitch, heading;
    float gx, gy, gz;
    static uint8_t counter = 0;

    if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
        return;
    }
    timestamp = millis();
    // Read the motion sensors
    sensors_event_t accel, gyro, mag;
    accelerometer->getEvent(&accel);
    gyroscope->getEvent(&gyro);
    magnetometer->getEvent(&mag);
#if defined(AHRS_DEBUG_OUTPUT)
    serial.print("I2C took ");
    serial.print(millis() - timestamp);
    serial.println(" ms");
#endif

    cal.calibrate(mag);
    cal.calibrate(accel);
    cal.calibrate(gyro);
    // Gyroscope needs to be converted from Rad/s to Degree/s
    // the rest are not unit-important
    gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
    gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
    gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

    // Update the SensorFusion filter
    filter.update(gx, gy, gz, accel.acceleration.x, accel.acceleration.y,
                  accel.acceleration.z, mag.magnetic.x, mag.magnetic.y,
                  mag.magnetic.z);
#if defined(AHRS_DEBUG_OUTPUT)
    serial.print("Update took ");
    serial.print(millis() - timestamp);
    serial.println(" ms");
#endif

    // only print the calculated output once in a while
    if (counter++ <= PRINT_EVERY_N_UPDATES) {
        return;
    }
    // reset the counter
    counter = 0;

#if defined(AHRS_DEBUG_OUTPUT)
    serial.print("Raw: ");
    serial.print(accel.acceleration.x, 4);
    serial.print(", ");
    serial.print(accel.acceleration.y, 4);
    serial.print(", ");
    serial.print(accel.acceleration.z, 4);
    serial.print(", ");
    serial.print(gx, 4);
    serial.print(", ");
    serial.print(gy, 4);
    serial.print(", ");
    serial.print(gz, 4);
    serial.print(", ");
    serial.print(mag.magnetic.x, 4);
    serial.print(", ");
    serial.print(mag.magnetic.y, 4);
    serial.print(", ");
    serial.print(mag.magnetic.z, 4);
    serial.println("");
#endif

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    serial.print("Orientation: ");
    serial.print(heading);
    serial.print(", ");
    serial.print(pitch);
    serial.print(", ");
    serial.println(roll);

    float qw, qx, qy, qz;
    filter.getQuaternion(&qw, &qx, &qy, &qz);
    serial.print("Quaternion: ");
    serial.print(qw, 4);
    serial.print(", ");
    serial.print(qx, 4);
    serial.print(", ");
    serial.print(qy, 4);
    serial.print(", ");
    serial.println(qz, 4);

#if defined(AHRS_DEBUG_OUTPUT)
    serial.print("Took ");
    serial.print(millis() - timestamp);
    serial.println(" ms");
#endif
}

int init_sensors(void) {
    int retval = mpu9250.begin();
    if (retval == 0) {
      accelerometer = mpu9250.getAccelerometerSensor();
      gyroscope = mpu9250.getGyroSensor();
      magnetometer = mpu9250.getMagnetometerSensor();
    }

    return retval;
}

void setup_sensors(void) {
    // defaults from sensor creation are OK (?)
}
