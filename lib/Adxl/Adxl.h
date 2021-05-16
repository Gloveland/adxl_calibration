#ifndef ADXL_H
#define ADXL_H

#include <Arduino.h>
#include <Wire.h>  // Wire library - used for I2C communication

class Adxl{
    public:
        Adxl();
        void begin();
        void calibrate();
        void read();
        ~Adxl();

    private:
        int ADXL345 = 0x53; // The ADXL345 sensor I2C address
        void setOffset(const int8_t offsetX, const int8_t offsetY, const int8_t offsetZ);
        int16_t X_out, Y_out, Z_out;  // Outputs
        int8_t X_offset, Y_offset, Z_offset;
        float Xg, Yg, Zg;
        float roll,pitch,rollF,pitchF=0;
   
};

#endif //ADXL_H
