/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

  This software may be distributed and modified under the terms of the GNU
  General Public License version 2 (GPL2) as published by the Free Software
  Foundation and appearing in the file GPL2.TXT included in the packaging of
  this file. Please note that GPL2 Section 2[b] requires that all works based
  on this software must also be made publicly available under the terms of
  the GPL2 ("Copyleft").

  Contact information
  -------------------

  Kristian Lauszus, TKJ Electronics
  Web      :  http://www.tkjelectronics.com
  e-mail   :  kristianl@tkjelectronics.com

  Modified by Brian Palmer aug, 2020 - added Moving Averaging filter and added Kalman filter on Grade.
*/

#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Arduino_LSM6DS3.h> // wifi, accelerometer, and gyroscope sensor. see https://www.arduino.cc/en/Reference/Arduino_LSM6DS3 
#include <MovingAverageFilter.h>

// Moving Average filters
MovingAverageFilter movingAverageFilter_x(9);        //
MovingAverageFilter movingAverageFilter_y(9);        // Moving average filters for the accelerometers
MovingAverageFilter movingAverageFilter_z(9);
float moveAvgX, moveAvgY, moveAvgZ;

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanAngleX; // Create the Kalman instances
Kalman kalmanAngleY;
Kalman kalmanGradeX;
Kalman kalmanGradeY;

/* IMU Data */
float accX, accY, accZ;
float gyroX, gyroY, gyroZ; //rotational speed in dps (degrees per second).

double gyroXangle, gyroYangle; // Calculated angle using the gyro only
double gyroXgrade, gyroYgrade; // Calculated grade using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double compGradeX, compGradeY; // Calculated grade using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double kalGradeX, kalGradeY; // Calculated grade using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

void setup() {
  Serial.begin(115200);
  delay(5000);
  Wire.begin();
  IMU.begin();
  //  nano 33 iot Accelerometer sample rate = 104.00 Hz
  //  Serial.print("Accelerometer sample rate = ");
  //  Serial.print(IMU.accelerationSampleRate());
  //  Serial.println(" Hz");

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
  }

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double rollAngle  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitchAngle = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double rollAngle  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitchAngle = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanAngleX.setAngle(rollAngle); // Set starting angle
  kalmanAngleX.setAngle(pitchAngle);
  gyroXangle = rollAngle;
  gyroYangle = pitchAngle;
  compAngleX = rollAngle;
  compAngleY = pitchAngle;

  timer = micros();
  delay(5000);
}

void loop() {
  /* Update all the values */
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accX, accY, accZ); // Query the IMU accelerometer and return the acceleration in g
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ); // Query Gyro for rotational speed in dps (degrees per second)
  }

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double rollRiseRun = atan2(accY, accZ);
  double rollGrade = rollRiseRun * 100;
  double rollAngle = rollRiseRun * RAD_TO_DEG;

  double pitchRiseRun = sqrt(accY * accY + accZ * accZ);
  double pitchGrade = pitchRiseRun * 100;
  double pitchAngle = atan(-accX / pitchRiseRun) * RAD_TO_DEG;

#else // Eq. 28 and 29
  double rollRiseRun = sqrt(accX * accX + accZ * accZ);
  double rollGrade = rollRiseRun * 100;
  double rollAngle = atan(accY / rollRiseRun) * RAD_TO_DEG;;

  double pitchRiseRun = atan2(-accX, accZ);
  double pitchGrade = pitchRiseRun * 100;
  double pitchAngle =  pitchRiseRun * RAD_TO_DEG;
#endif

  // 131 corresponds to a rotation rate of 1 degree per second.
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((rollAngle < -90 && kalAngleX > 90) || (rollAngle > 90 && kalAngleX < -90)) {
    kalmanAngleX.setAngle(rollAngle);
    kalmanGradeX.setAngle(rollGrade);
    compAngleX = rollAngle;
    kalAngleX = rollAngle;
    gyroXangle = rollAngle;
    gyroXgrade = rollGrade;
  } else {
    kalAngleX = kalmanAngleX.getAngle(rollAngle, gyroXrate, dt); // Calculate the angle using a Kalman filter
    kalGradeX = kalmanGradeX.getAngle(rollGrade, gyroXrate, dt); // Calculate the grade using a Kalman filter
  }
  if (abs(kalAngleX) > 90) {
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  }
  kalAngleY = kalmanAngleY.getAngle(pitchAngle, gyroYrate, dt);
  kalGradeY = kalmanGradeY.getAngle(pitchGrade, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitchAngle < -90 && kalAngleY > 90) || (pitchAngle > 90 && kalAngleY < -90)) {
    kalmanAngleY.setAngle(pitchAngle);
    kalmanGradeY.setAngle(pitchGrade);
    compAngleY = pitchAngle;
    kalAngleY = pitchAngle;
    gyroYangle = pitchAngle;
  } else {
    kalAngleY = kalmanAngleY.getAngle(pitchAngle, gyroYrate, dt); // Calculate the angle using a Kalman filter
    kalGradeY = kalmanGradeY.getAngle(pitchGrade, gyroYrate, dt); // Calculate the angle using a Kalman filter
  }
  if (abs(kalAngleY) > 90) {
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  }
  kalAngleX = kalmanAngleX.getAngle(rollAngle, gyroXrate, dt); // Calculate the angle using a Kalman filter
  kalGradeX = kalmanGradeX.getAngle(rollGrade, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  // Calculate gyro angle without any filter
  gyroXangle += gyroXrate * dt;
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  // Calculate the angle using a Complimentary filter
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * rollAngle;
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitchAngle;

  // Calculate the grade using a Complimentary filter
  compGradeX = 0.93 * (compGradeX + gyroXrate * dt) + 0.07 * rollGrade;
  compGradeY = 0.93 * (compGradeY + gyroYrate * dt) + 0.07 * pitchGrade;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  // moving average filter incline

  moveAvgX = movingAverageFilter_x.process(accX);      //
  moveAvgY = movingAverageFilter_y.process(accY);      //   Apply moving average filters to reduce noise
  moveAvgZ = movingAverageFilter_z.process(accZ);      //
  double mvgAvgGradeX = calcGrade(moveAvgX, moveAvgY, moveAvgZ);

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print("accY:"); Serial.print(accY); Serial.print("\t");
  Serial.print("accZ:"); Serial.print(accZ); Serial.print("\t");

  Serial.print("gyroX:"); Serial.print(gyroX); Serial.print("\t");
  Serial.print("gyroY:"); Serial.print(gyroY); Serial.print("\t");
  Serial.print("gyroZ:"); Serial.print(gyroZ); Serial.print("\t");
#endif
  //  Angles
  //  Serial.print("unFiltAngle:"); Serial.print(rollAngle); Serial.print("\t");
  //  Serial.print("gyroXangle:");Serial.print(gyroXangle); Serial.print("\t");
  //  Serial.print("complAngle:"); Serial.print(compAngleX); Serial.print("\t");
  //  Serial.print("kalmanAngle:"); Serial.print(kalAngleX); Serial.print("\t");

  //  Serial.print("pitch:"); Serial.print(pitch); Serial.print("\t");
  //  Serial.print("gyroYangle:");Serial.print(gyroYangle); Serial.print("\t");
  //  Serial.print("compAngleY:"); Serial.print(compAngleY); Serial.print("\t");
  //  Serial.print("kalAngleY:"); Serial.print(kalAngleY); Serial.print("\t");

  // Grades
  Serial.print("unfiltered:"); Serial.print(rollGrade); Serial.print("\t");
  Serial.print("complementaryFilter:"); Serial.print(compGradeX); Serial.print("\t");
  Serial.print("kalmanFilter:"); Serial.print(kalGradeX); Serial.print("\t");
  Serial.print("movingAvg:"); Serial.print(mvgAvgGradeX); //Serial.print("\t");
  
  Serial.print("\r\n");
  delay(2);
}

double calcGrade(float x, float y, float z) {
  double grade = 0;

  // find pitch in radians
  float radpitch = atan2( (y) , sqrt(x * x + z * z) );
  // find the % grade from the pitch
  grade = tan(radpitch) * 100;

  //grade = grade * -1; // flip the sign since its mounted with the USB port on the left.
  return grade;
}
