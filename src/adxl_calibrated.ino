

/*esta prueba funciona mas o menos bien con un data rate de 2g. lee 1 cuando x esta para arriba */

#include <Wire.h>  // Wire library - used for I2C communication
int ADXL345 = 0x53; // The ADXL345 sensor I2C address

int16_t X_out, Y_out, Z_out;  // Outputs
int8_t X_offset, Y_offset, Z_offset;
float Xg, Yg, Zg;

float roll,pitch,rollF,pitchF=0;

void setup() {
    Serial.begin(9600);               // Initiate serial communication for printing the results on the Serial monitor
    Wire.begin();  

    // Set ADXL345 DATA FORMAT 
    Wire.beginTransmission(ADXL345);
    Wire.write(0x31);               // Access DATA_FORMAT Register= 0x31
    Wire.write(8);                 // (8 decimal <=> 0000 1000 binary <=> 0x08 hexadecimal ->  +/- 2g )  // (11 decimal <=> 0000 1011 binary <=> 0x0B hexadecimal -> +/- 16g ) 
    Wire.endTransmission();
    delay(10);

    // Set ADXL345 in measuring mode -> Enable measurement
    Wire.beginTransmission(ADXL345);  // Start communicating with the device 
    Wire.write(0x2D);                 // Access POWER_CTL Register= 0x2D
    Wire.write(8);                    // (8 decimal <=> 0000 1000 binary <=> 0x08 hexadecimal)  Bit D3 High for measuring enable 
    Wire.endTransmission();
    delay(10);


    // Set data rate
    Wire.beginTransmission(ADXL345);  // Start communicating with the device 
    Wire.write(0x2C);                 // Access DATA_RATE Register= 0x2C
    Wire.write((byte)0x0B);           // (11 decimal <=> 00001011 binary <=> 0x0B hexadecimal) 
    Wire.endTransmission();
    delay(10);

    setOffset(0, 0, 0);

    // === Calibration === //
    Serial.println("Type key when Sensor is placed over an horizontal plane: X = 0g, Y = 0g, Z = +1g orientation"); 
    while (!Serial.available()){
      //wait for a character 
    }  

    int16_t X_sum = 0;
    int16_t Y_sum = 0;
    int16_t Z_sum = 0;
    
    Serial.println("Calibrating"); 
    int times = 100;
    for (int i = 0; i <= times; i++) {
        Wire.beginTransmission(ADXL345);
        Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
        X_out = ( Wire.read() |  Wire.read() << 8); // X-axis value
        Y_out = ( Wire.read() |  Wire.read() << 8); // Y-axis value
        Z_out = ( Wire.read() |  Wire.read() << 8); // Z-axis value
        Serial.print("."); 
        //Serial.print("    Xa= "); Serial.print(X_out); 
        //Serial.print("    Ya= "); Serial.print(Y_out); 
        //Serial.print("    Za= "); Serial.println(Z_out);
        X_sum += X_out;
        Y_sum += Y_out;
        Z_sum += Z_out;
        delay(100);
    }
    // between -2 to 2 there are 4 gravities.
    //X_offset = -round((X_sum/times) / 4);
    //Y_offset = -round((Y_sum/times) / 4);
    //Z_offset = -round((Z_sum/times - 256) / 4);

    setOffset(X_offset, Y_offset, Z_offset );
    
    Serial.print("   Xoff= "); Serial.println(X_offset);
    Serial.print("   Yoff= "); Serial.println(Y_offset);
    Serial.print("   Zoff= "); Serial.println(Z_offset);
    delay(11.1);
    while (Serial.available()){
      Serial.read();  // clear the input buffer
    }

    Serial.println("Type key to start mesuring acceleration..."); 
    while (!Serial.available()){
      /* wait for a character */
    }  
    while (Serial.available()){
      Serial.read();  // clear the input buffer
    }
}

void setOffset(int8_t offsetX, int8_t offsetY, int8_t offsetZ){
      //Off-set Calibration
    //X-axis
    Wire.beginTransmission(ADXL345);
    Wire.write(0x1E);
    Wire.write(offsetX);
    Wire.endTransmission();
    delay(10);
    //Y-axis
    Wire.beginTransmission(ADXL345);
    Wire.write(0x1F);
    Wire.write(offsetY);
    Wire.endTransmission();
    delay(10);
    //Z-axis
    Wire.beginTransmission(ADXL345);
    Wire.write(0x20);
    Wire.write(offsetZ);
    Wire.endTransmission();
    delay(10);
}

void loop() {
    // === Read acceleromter data === //
    Wire.beginTransmission(ADXL345);
    Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    X_out = ( Wire.read() | Wire.read() << 8); // X-axis value
    Y_out = ( Wire.read() | Wire.read() << 8); // Y-axis value
    Z_out = ( Wire.read() | Wire.read() << 8); // Z-axis value
    //For a range of +-2g, we need to divide the raw values by 256, according to the datasheet
    Xg =  X_out/ 256.0;
    Yg =  Y_out/ 256.0;
    Zg =  Z_out/ 256.0;
    Serial.print("       Xa= ");Serial.print(X_out);Serial.print("= ");  Serial.print(Xg);Serial.print("= ");Serial.print(Xg * 9.8106);
    Serial.print("       Ya= ");Serial.print(Y_out);Serial.print("= ");  Serial.print(Yg);Serial.print("= ");Serial.print(Yg * 9.8106);
    Serial.print("       Za= ");Serial.print(Z_out);Serial.print("= ");  Serial.print(Zg);Serial.print("= ");Serial.print(Zg * 9.8106);

    // Calculate Roll and Pitch (rotation around X-axis, rotation around Y-axis)
    roll = atan(Y_out / sqrt(pow(X_out, 2) + pow(Z_out, 2))) * 180 / PI;
    pitch = atan(-1 * X_out / sqrt(pow(Y_out, 2) + pow(Z_out, 2))) * 180 / PI;

    // Low-pass filter
    rollF = 0.94 * rollF + 0.06 * roll;
    pitchF = 0.94 * pitchF + 0.06 * pitch;
    
    Serial.print("       roll= ");Serial.print(roll);
    Serial.print("       pitch= ");Serial.println(pitch);
    delay(200);
}
