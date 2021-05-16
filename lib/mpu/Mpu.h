#ifndef MPU_H
#define MPU_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

class Mpu{
    private:
        Adafruit_MPU6050 mpu;
        sensors_event_t a, g, t;
        float elapsedTime, currentTime, previousTime;
        float gyroAngleX,  gyroAngleY,accAngleX,accAngleY;;
        float roll,pitch,yaw;

        float accErrorX, accErrorY,  accErrorZ;
        float accAngleErrorX, accAngleErrorY;
        float gyroErrorX, gyroErrorY, gyroErrorZ;

        void readAcc(sensors_event_t a);
        void readGyro(sensors_event_t g, float accX, float accY, float accZ);
        void readTemperature(sensors_event_t t);
        float calculateAccAngleX(float accX, float accY, float accZ);
        float calculateAccAngleY(float accX, float accY, float accZ);



    public:
        Mpu();
        void calibrate();
        void init();
        void read();
        ~Mpu();

    
       
   
};

#endif //MPU_H
