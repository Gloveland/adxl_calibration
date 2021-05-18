#ifndef MPU_H
#define MPU_H

#include <Arduino.h>
#include <Wire.h>
#include <float.h>

#define GRAVITY_EARTH 9.80665F

#define GENERAL_CONFIG 0x1A      ///< General configuration register

#define  MPU6050_BAND_260_HZ 0x00 ///< Docs imply this disables the filter
#define  MPU6050_BAND_184_HZ 0x01 ///< 184 Hz
#define  MPU6050_BAND_94_HZ  0x02 ///< 94 Hz
#define  MPU6050_BAND_44_HZ  0x03 ///< 44 Hz
#define  MPU6050_BAND_21_HZ  0x04 ///< 21 Hz
#define  MPU6050_BAND_10_HZ  0x05 ///< 10 Hz
#define  MPU6050_BAND_5_HZ   0x06 ///< 5 Hz

#define  ACC_CONFIG_REGISTER 0x1C

#define  MPU6050_RANGE_2_G 0x00   /// 00000   +/- 2g (default value)
#define  MPU6050_RANGE_4_G 0x08   /// 01000   +/- 4g
#define  MPU6050_RANGE_8_G 0x10   /// 10000   +/- 8g
#define  MPU6050_RANGE_16_G 0x18  /// 11000   +/- 16g

#define  GYRO_CONFIG_REGISTER 0x1C

#define  MPU6050_RANGE_250_DEG 0x00  ///< +/- 250 deg/s (default value)
#define  MPU6050_RANGE_500_DEG 0x08  ///< +/- 500 deg/s
#define  MPU6050_RANGE_1000_DEG 0x10 ///< +/- 1000 deg/s
#define  MPU6050_RANGE_2000_DEG 0x18 ///< +/- 2000 deg/s


class Mpu{

    public:
        Mpu();
        void init();
        void calibrate();
        void read();
        ~Mpu();

    private:
        const int MPU = 0x68;
        float AccErrorX, AccErrorY,  AccErrorZ;
        float AccAngleErrorX, AccAngleErrorY;
        float GyroErrorX, GyroErrorY, GyroErrorZ;
        float GyroX, GyroY, GyroZ;
        float PrevGyroX, PrevGyroY, PrevGyroZ;
        float DeviationX, DeviationY,DeviationZ;
        float DegreesDeviationX, DegreesDeviationY, DegreesDeviationZ;
        float calculateAccAngleX(float accX, float accY, float accZ);
        float calculateAccAngleY(float accX, float accY, float accZ);
        float PreviousTime, CurrentTime, ElapsedTime;
        float GyroAngleX, GyroAngleY, GyroAngleZ;
        
    

};

#endif //MPU_H
