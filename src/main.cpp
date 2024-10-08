#include <Arduino.h>
#include <Adafruit_Sensor_Calibration.h>
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

#if defined(CALIBRATION)

/***************************************************************************
  This is an example for the Adafruit AHRS library
  It will look for a supported magnetometer and output
  PJRC Motion Sensor Calibration Tool-compatible serial data

  PJRC & Adafruit invest time and resources providing this open source code,
  please support PJRC and open-source hardware by purchasing products
  from PJRC!

  Written by PJRC, adapted by Limor Fried for Adafruit Industries.
 ***************************************************************************/

void receiveCalibration(void);
uint16_t crc16_update(uint16_t crc, uint8_t a);

sensors_event_t mag_event, gyro_event, accel_event;

int loopcount = 0;

void setup(void) {
    serial.begin(115200);
    while (!serial)
        delay(10);  // will pause Zero, Leonardo, etc until serial console opens
    delay(1000);

    serial.println(F("Adafruit AHRS - IMU Calibration!"));

    serial.println("Calibration filesys test");
    if (!cal.begin()) {
        serial.println("Failed to initialize calibration helper");
        while (1) yield();
    }
    if (!cal.loadCalibration()) {
        serial.println(
            "No calibration loaded/found... will start with defaults");
    } else {
        serial.println("Loaded existing calibration");
    }

    int retval = init_sensors();
    if (retval != 0) {
        if (retval == 1) {
            serial.println("Failed to locate MPU9250 I2C address");
            while (1) delay(10);
        } else if (retval == 2) {
            serial.println("MPU9250 WHOAMI incorrect");
            while (1) delay(10);
        } else if (retval == 3) {
            serial.println("AK89630 WHOAMI incorrect");
            while (1) delay(10);
        } else {
            serial.println("Unknown initialization error");
            while (1) delay(10);
        }
    }

    accelerometer->printSensorDetails();
    gyroscope->printSensorDetails();
    magnetometer->printSensorDetails();

    setup_sensors();

    Wire.setClock(400000);  // 400KHz
}

void loop() {
    magnetometer->getEvent(&mag_event);
    gyroscope->getEvent(&gyro_event);
    accelerometer->getEvent(&accel_event);

    // 'Raw' values to match expectation of MotionCal
    serial.print("Raw:");
    serial.print(int(accel_event.acceleration.x * 8192 / 9.8));
    serial.print(",");
    serial.print(int(accel_event.acceleration.y * 8192 / 9.8));
    serial.print(",");
    serial.print(int(accel_event.acceleration.z * 8192 / 9.8));
    serial.print(",");
    serial.print(int(gyro_event.gyro.x * SENSORS_RADS_TO_DPS * 16));
    serial.print(",");
    serial.print(int(gyro_event.gyro.y * SENSORS_RADS_TO_DPS * 16));
    serial.print(",");
    serial.print(int(gyro_event.gyro.z * SENSORS_RADS_TO_DPS * 16));
    serial.print(",");
    serial.print(int(mag_event.magnetic.x * 10));
    serial.print(",");
    serial.print(int(mag_event.magnetic.y * 10));
    serial.print(",");
    serial.print(int(mag_event.magnetic.z * 10));
    serial.println("");

    // unified data
    serial.print("Uni:");
    serial.print(accel_event.acceleration.x);
    serial.print(",");
    serial.print(accel_event.acceleration.y);
    serial.print(",");
    serial.print(accel_event.acceleration.z);
    serial.print(",");
    serial.print(gyro_event.gyro.x, 4);
    serial.print(",");
    serial.print(gyro_event.gyro.y, 4);
    serial.print(",");
    serial.print(gyro_event.gyro.z, 4);
    serial.print(",");
    serial.print(mag_event.magnetic.x);
    serial.print(",");
    serial.print(mag_event.magnetic.y);
    serial.print(",");
    serial.print(mag_event.magnetic.z);
    serial.println("");

    // magnitude
    serial.print("Magnitude:");
    float magnitude;
    magnitude = sqrt(accel_event.acceleration.x * accel_event.acceleration.x
                     + accel_event.acceleration.y * accel_event.acceleration.y
                     + accel_event.acceleration.z * accel_event.acceleration.z);
    serial.print(magnitude, 4);
    serial.print(",");
    magnitude = sqrt(gyro_event.gyro.x * gyro_event.gyro.x
                     + gyro_event.gyro.y * gyro_event.gyro.y
                     + gyro_event.gyro.z * gyro_event.gyro.z);
    serial.print(magnitude, 4);
    serial.print(",");
    magnitude = sqrt(mag_event.magnetic.x * mag_event.magnetic.x
                     + mag_event.magnetic.y * mag_event.magnetic.y
                     + mag_event.magnetic.z * mag_event.magnetic.z);
    serial.print(magnitude, 4);
    serial.println("");

    loopcount++;
    receiveCalibration();

    // occasionally print calibration
    if (loopcount == 50 || loopcount > 100) {
        serial.print("Cal1:");
        for (int i = 0; i < 3; i++) {
            serial.print(cal.accel_zerog[i], 3);
            serial.print(",");
        }
        for (int i = 0; i < 3; i++) {
            serial.print(cal.gyro_zerorate[i], 3);
            serial.print(",");
        }
        for (int i = 0; i < 3; i++) {
            serial.print(cal.mag_hardiron[i], 3);
            serial.print(",");
        }
        serial.println(cal.mag_field, 3);
        loopcount++;
    }
    if (loopcount >= 100) {
        serial.print("Cal2:");
        for (int i = 0; i < 9; i++) {
            serial.print(cal.mag_softiron[i], 4);
            if (i < 8) serial.print(',');
        }
        serial.println();
        loopcount = 0;
    }

    delay(10);
}

/********************************************************/

byte caldata[68];  // buffer to receive magnetic calibration data
byte calcount = 0;

void receiveCalibration() {
    uint16_t crc;
    byte b, i;

    while (serial.available()) {
        b = serial.read();
        if (calcount == 0 && b != 117) {
            // first byte must be 117
            return;
        }
        if (calcount == 1 && b != 84) {
            // second byte must be 84
            calcount = 0;
            return;
        }
        // store this byte
        caldata[calcount++] = b;
        if (calcount < 68) {
            // full calibration message is 68 bytes
            return;
        }
        // verify the crc16 check
        crc = 0xFFFF;
        for (i = 0; i < 68; i++) {
            crc = crc16_update(crc, caldata[i]);
        }
        if (crc == 0) {
            // data looks good, use it
            float offsets[16];
            memcpy(offsets, caldata + 2, 16 * 4);
            cal.accel_zerog[0] = offsets[0];
            cal.accel_zerog[1] = offsets[1];
            cal.accel_zerog[2] = offsets[2];

            cal.gyro_zerorate[0] = offsets[3];
            cal.gyro_zerorate[1] = offsets[4];
            cal.gyro_zerorate[2] = offsets[5];

            cal.mag_hardiron[0] = offsets[6];
            cal.mag_hardiron[1] = offsets[7];
            cal.mag_hardiron[2] = offsets[8];

            cal.mag_field = offsets[9];

            cal.mag_softiron[0] = offsets[10];
            cal.mag_softiron[1] = offsets[13];
            cal.mag_softiron[2] = offsets[14];
            cal.mag_softiron[3] = offsets[13];
            cal.mag_softiron[4] = offsets[11];
            cal.mag_softiron[5] = offsets[15];
            cal.mag_softiron[6] = offsets[14];
            cal.mag_softiron[7] = offsets[15];
            cal.mag_softiron[8] = offsets[12];

            if (!cal.saveCalibration()) {
                serial.println("**WARNING** Couldn't save calibration");
            } else {
                serial.println("Wrote calibration");
            }
            cal.printSavedCalibration();
            calcount = 0;
            return;
        }
        // look for the 117,84 in the data, before discarding
        for (i = 2; i < 67; i++) {
            if (caldata[i] == 117 && caldata[i + 1] == 84) {
                // found possible start within data
                calcount = 68 - i;
                memmove(caldata, caldata + i, calcount);
                return;
            }
        }
        // look for 117 in last byte
        if (caldata[67] == 117) {
            caldata[0] = 117;
            calcount = 1;
        } else {
            calcount = 0;
        }
    }
}

uint16_t crc16_update(uint16_t crc, uint8_t a) {
    int i;
    crc ^= a;
    for (i = 0; i < 8; i++) {
        if (crc & 1) {
            crc = (crc >> 1) ^ 0xA001;
        } else {
            crc = (crc >> 1);
        }
    }
    return crc;
}
#else

// Full orientation sensing using NXP/Madgwick/Mahony and a range of 9-DoF
// sensor sets.
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// to Adafruit Unified Sensor interface

#include <Adafruit_AHRS.h>

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
#endif

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
