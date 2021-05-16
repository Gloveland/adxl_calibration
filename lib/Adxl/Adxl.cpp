#include <Adxl.h>

Adxl::Adxl(){

}

void Adxl::init(){
    Wire.begin();  

    // Set ADXL345 DATA FORMAT 
    Wire.beginTransmission(this->ADXL345);
    Wire.write(0x31);               // Access DATA_FORMAT Register= 0x31
    Wire.write(8);                 // (8 decimal <=> 0000 1000 binary <=> 0x08 hexadecimal ->  +/- 2g )  // (11 decimal <=> 0000 1011 binary <=> 0x0B hexadecimal -> +/- 16g ) 
    Wire.endTransmission();
    delay(10);

    // Set ADXL345 in measuring mode -> Enable measurement
    Wire.beginTransmission(this->ADXL345);  // Start communicating with the device 
    Wire.write(0x2D);                 // Access POWER_CTL Register= 0x2D
    Wire.write(8);                    // (8 decimal <=> 0000 1000 binary <=> 0x08 hexadecimal)  Bit D3 High for measuring enable 
    Wire.endTransmission();
    delay(10);


    // Set data rate
    Wire.beginTransmission(this->ADXL345);  // Start communicating with the device 
    Wire.write(0x2C);                 // Access DATA_RATE Register= 0x2C
    Wire.write((byte)0x0B);           // (11 decimal <=> 00001011 binary <=> 0x0B hexadecimal) 
    Wire.endTransmission();
    delay(10);
    this->setOffset(0,0,0);

}

void Adxl::calibrate(){
    int16_t X, Y, Z = 0;
    int16_t X_sum, Y_sum, Z_sum = 0;
    
    Serial.println("===================== Calibrating ===================="); 
    int times = 100;
    for (int i = 0; i <= times; i++) {
        Wire.beginTransmission(this->ADXL345);
        Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(this->ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
        X = ( Wire.read() |  Wire.read() << 8); // X-axis value
        Y = ( Wire.read() |  Wire.read() << 8); // Y-axis value
        Z = ( Wire.read() |  Wire.read() << 8); // Z-axis value
        Serial.print("."); 
        //Serial.print("    Xa= "); Serial.print(X); 
        //Serial.print("    Ya= "); Serial.print(Y); 
        //Serial.print("    Za= "); Serial.println(Z);
        X_sum += X;
        Y_sum += Y;
        Z_sum += Z;
        delay(100);
    }
    Serial.println();
    // between -2 to 2 there are 4 gravities.
    X_offset = -round((X_sum/times) / 4);
    Y_offset = -round((Y_sum/times) / 4);
    Z_offset = -round((Z_sum/times - 256) / 4);
    this->setOffset(X_offset, Y_offset, Z_offset );
    //Serial.print("    X_Offset= "); Serial.print(X_offset); 
    //Serial.print("    Y_Offset= "); Serial.print(Y_offset); 
    //Serial.print("    Z_Offset= "); Serial.println(Z_offset);
}

void Adxl::setOffset(const int8_t offsetX, const int8_t offsetY, const int8_t offsetZ){
    //Off-set Calibration
    //X-axis
    Wire.beginTransmission(this->ADXL345);
    Wire.write(0x1E);
    Wire.write(offsetX);
    Wire.endTransmission();
    delay(10);
    //Y-axis
    Wire.beginTransmission(this->ADXL345);
    Wire.write(0x1F);
    Wire.write(offsetY);
    Wire.endTransmission();
    delay(10);
    //Z-axis
    Wire.beginTransmission(this->ADXL345);
    Wire.write(0x20);
    Wire.write(offsetZ);
    Wire.endTransmission();
    delay(10);
}

void Adxl::read(){
    // === Read acceleromter data === //
    Serial.print("ADXL:");
    Wire.beginTransmission(this->ADXL345);
    Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(this->ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    this->X_out = ( Wire.read() | Wire.read() << 8); // X-axis value
    this->Y_out = ( Wire.read() | Wire.read() << 8); // Y-axis value
    this->Z_out = ( Wire.read() | Wire.read() << 8); // Z-axis value
    //For a range of +-2g, we need to divide the raw values by 256, according to the datasheet
    this->Xg =  this->X_out/ 256.0;
    this->Yg =  this->Y_out/ 256.0;
    this->Zg =  this->Z_out/ 256.0;
    Serial.print("       Xa= ");Serial.print(this->X_out);Serial.print("= ");  Serial.print(this->Xg);Serial.print("= ");Serial.print(this->Xg * 9.8106);
    Serial.print("       Ya= ");Serial.print(this->Y_out);Serial.print("= ");  Serial.print(this->Yg);Serial.print("= ");Serial.print(this->Yg * 9.8106);
    Serial.print("       Za= ");Serial.print(this->Z_out);Serial.print("= ");  Serial.print(this->Zg);Serial.print("= ");Serial.print(this->Zg * 9.8106);

    // Calculate Roll and Pitch (rotation around X-axis, rotation around Y-axis)
    this->roll = atan(this->Y_out / sqrt(pow(this->X_out, 2) + pow(this->Z_out, 2))) * 180 / PI;
    this->pitch = atan(-1 * this->X_out / sqrt(pow(this->Y_out, 2) + pow(this->Z_out, 2))) * 180 / PI;

    // Low-pass filter
    this->rollF = 0.94 * this->rollF + 0.06 * this->roll;
    this->pitchF = 0.94 * this->pitchF + 0.06 * this->pitch;
    
    Serial.print("       roll= ");Serial.print(this->roll);
    Serial.print("       pitch= ");Serial.println(this->pitch);

}

Adxl::~Adxl(){

}

