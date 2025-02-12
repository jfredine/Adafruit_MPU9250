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
