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
    Serial.println("===================== Calibrating ADXL345 ===================="); 

    int times = 100;
    for (int i = 0; i <= times; i++) {
        Wire.beginTransmission(this->ADXL345);
        Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(this->ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
        X = ( Wire.read() |  Wire.read() << 8); // X-axis value
        Y = ( Wire.read() |  Wire.read() << 8); // Y-axis value
        Z = ( Wire.read() |  Wire.read() << 8); // Z-axis value
        
        X_sum =  X_sum + X;
        Y_sum =  Y_sum + Y;
        Z_sum =  Z_sum + Z;

        this->accX =  (X / 256.0) * 9.8106;
        this->accY =  (Y / 256.0) * 9.8106;
        this->accZ =  (Z / 256.0 )* 9.8106;
        
        Serial.print("."); 
        //Serial.print("    Xa= "); Serial.print(X); 
        //Serial.print("    Ya= "); Serial.print(Y); 
        //Serial.print("    Za= "); Serial.println(Z);
        delay(100);
    }
    // between -2 to 2 there are 4 gravities.
    X_offset = -round((X_sum/times) / 4);
    Y_offset = -round((Y_sum/times) / 4);
    Z_offset = -round((Z_sum/times - 256) / 4);
    Serial.println();
    //Serial.print("    X_Offset= "); Serial.print(X_offset); 
    //Serial.print("    Y_Offset= "); Serial.print(Y_offset); 
    //Serial.print("    Z_Offset= "); Serial.print(Z_offset);
    this->setOffset(X_offset, Y_offset, Z_offset);


    for (int i = 0; i <= times; i++) {
        Wire.beginTransmission(this->ADXL345);
        Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(this->ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
        X = ( Wire.read() |  Wire.read() << 8); // X-axis value
        Y = ( Wire.read() |  Wire.read() << 8); // Y-axis value
        Z = ( Wire.read() |  Wire.read() << 8); // Z-axis value

        this->accX =  (X / 256.0) * 9.8106;
        this->accY =  (Y / 256.0) * 9.8106;
        this->accZ =  (Z / 256.0 )* 9.8106;

        this->accErrorX = this->accErrorX + this->accX;
        this->accErrorY = this->accErrorY + this->accY;
        this->accErrorZ = this->accErrorZ + (this->accZ - 9.8106);

        Serial.print(".");
        Serial.print("  accX=");Serial.print(this->accX);
        Serial.print("  accY=");Serial.print(this->accY);
        Serial.print("  accZ=");Serial.println(this->accZ);
        delay(100);
        
    } 
    this->accErrorX = this->accErrorX/times;
    this->accErrorY = this->accErrorY/times;
    this->accErrorZ = this->accErrorZ/times;
    Serial.print("AccErrorX: ");Serial.println(this->accErrorX);
    Serial.print("AccErrorY: ");Serial.println(this->accErrorY);
    Serial.print("AccErrorZ: ");Serial.println(this->accErrorZ);


    float accAngleX, accAngleY = 0;
    for (int i = 0; i <= times; i++) {
        Wire.beginTransmission(this->ADXL345);
        Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(this->ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
        X = ( Wire.read() |  Wire.read() << 8); // X-axis value
        Y = ( Wire.read() |  Wire.read() << 8); // Y-axis value
        Z = ( Wire.read() |  Wire.read() << 8); // Z-axis value
        
        this->accX =  (X / 256.0) * 9.8106  - this->accErrorX;
        this->accY =  (Y / 256.0) * 9.8106  - this->accErrorY;
        this->accZ =  (Z / 256.0 )* 9.8106  - this->accErrorZ;
        
        accAngleX = this->calculateAccAngleX(this->accX, this->accY, this->accZ);
        accAngleY = this->calculateAccAngleY(this->accX, this->accY, this->accZ);
        
        this->accAngleErrorX = this->accAngleErrorX + accAngleX;
        this->accAngleErrorY = this->accAngleErrorY + accAngleY;

        Serial.print("."); 
        Serial.print("  accX=");Serial.print(this->accX );
        Serial.print("  accY=");Serial.print(this->accY );
        Serial.print("  accZ=");Serial.print(this->accZ );
        Serial.print("  accAngleX=");Serial.print(accAngleX);
        Serial.print("  accAngleY=");Serial.println(accAngleY);
        delay(100);
    } 

    this->accAngleErrorX = this->accAngleErrorX/times;
    this->accAngleErrorY = this->accAngleErrorY/times;
    
    Serial.println("");
    Serial.print("AccAngleErrorX: ");Serial.println(this->accAngleErrorX);
    Serial.print("AccAngleErrorY: ");Serial.println(this->accAngleErrorY);
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
    this->accX =  (this->Xg * 9.8106)  - this->accErrorX;
    this->accY =  (this->Yg * 9.8106)  - this->accErrorY;
    this->accZ =  (this->Zg * 9.8106)  - this->accErrorZ;
    Serial.print("       Ya= ");Serial.print(this->Y_out);Serial.print("= ");  Serial.print(this->Yg);Serial.print("= ");Serial.print(this->accY);
    Serial.print("       Xa= ");Serial.print(this->X_out);Serial.print("= ");  Serial.print(this->Xg);Serial.print("= ");Serial.print(this->accX);
    Serial.print("       Za= ");Serial.print(this->Z_out);Serial.print("= ");  Serial.print(this->Zg);Serial.print("= ");Serial.print(this->accZ);

    // Calculate Roll and Pitch (rotation around X-axis, rotation around Y-axis)
    this->roll  = this->calculateAccAngleX(this->accX, this->accY, this->accZ) - this->accAngleErrorX;
    this->pitch = this->calculateAccAngleY(this->accX, this->accY, this->accZ) - this->accAngleErrorY;

    // Low-pass filter
    this->rollF = 0.94 * this->rollF + 0.06 * this->roll;
    this->pitchF = 0.94 * this->pitchF + 0.06 * this->pitch;
    
    Serial.print("       pitch= ");Serial.print(this->pitch);
    Serial.print("       roll= "); Serial.println(this->roll);

}

float Adxl::calculateAccAngleX(float accX, float accY, float accZ){
    return (atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI);
}

float Adxl::calculateAccAngleY(float accX, float accY, float accZ){
    return (atan(-1 * accX/ sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI); 
}

Adxl::~Adxl(){

}

