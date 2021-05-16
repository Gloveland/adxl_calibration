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
        float gyroX, gyroY, gyroZ;
        float accGyroX, accGyroY, accGyroZ;
        float gyroError = 0.05;

        float gyroXoffset = -0.03;
        float gyroYoffset = -0.44;  
        float gyroZoffset = -0.03;

        void readGyro(sensors_event_t g);
        void readAcc(sensors_event_t a);
        void readTemperature(sensors_event_t t);


    public:
        Mpu();
        void init();
        void read();
        ~Mpu();

    
       
   
};

#endif //MPU_H
