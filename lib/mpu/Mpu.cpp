#include <Mpu.h>

int16_t rawAccX, rawAccY, rawAccZ;
float accX, accY, accZ;
float accAngleX, accAngleY;
int16_t rawGyroX, rawGyroY, rawGyroZ;
float gyroX, gyroY, gyroZ;
float roll, pitch, yaw;

Mpu::Mpu(){

}

void Mpu::init(){
    Wire.begin();                             // Initialize comunication
    
    Wire.beginTransmission(this->MPU);       // Start communication with MPU6050 // MPU=0x68
    Wire.write(0x6B);                       // Talk to the register 6B
    Wire.write(0x00);                      // Make reset - place a 0 into the 6B register
    Wire.endTransmission(true);           //end the transmission
    
    /*
    // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
    Wire.beginTransmission(MPU);
    Wire.write(ACC_CONFIG_REGISTER);                  //Talk to the ACCEL_CONFIG register (1C hex)
    Wire.write(MPU6050_RANGE_2_G);                  //Set the register bits as 00010000 = 0x10 (+/- 8g full scale range)
    Wire.endTransmission(true);
    delay(20);

    // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
    Wire.beginTransmission(MPU);
    Wire.write(GYRO_CONFIG_REGISTER);                   // Talk to the GYRO_CONFIG register (1B hex)
    Wire.write(MPU6050_RANGE_250_DEG);                   // Set the register bits as 00010000 (1000deg/s full scale)
    Wire.endTransmission(true);
    delay(20);

    Wire.beginTransmission(MPU);
    Wire.write(GENERAL_CONFIG);                   // Talk to the GYRO_CONFIG register (1B hex)
    Wire.write(MPU6050_BAND_21_HZ);                   // Set the register bits as 00010000 (1000deg/s full scale)
    Wire.endTransmission(true);
    delay(20);
    */
}

void Mpu::calibrate(){
    Serial.println("===================== Calibrating MPU6050 ===================="); 

    float accX, accY, accZ;
    this->AccErrorX = 0.0;
    this->AccErrorY = 0.0;
    this->AccErrorZ = 0.0;
    this->AccAngleErrorX = 0.0;
    this->AccAngleErrorY = 0.0;
    int times = 100.0;
    for (int i = 0; i <= times; i++) {
        Wire.beginTransmission(MPU);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);
        rawAccX = (Wire.read() << 8 | Wire.read()); 
        rawAccY = (Wire.read() << 8 | Wire.read()); 
        rawAccZ = (Wire.read() << 8 | Wire.read()); 

        accX = (rawAccX  / 16384.0) * GRAVITY_EARTH;
        accY = (rawAccY  / 16384.0) * GRAVITY_EARTH;
        accZ = (rawAccZ  / 16384.0) * GRAVITY_EARTH;
        
        this->AccErrorX = this->AccErrorX + accX;
        this->AccErrorY = this->AccErrorY + accY;
        this->AccErrorZ = this->AccErrorZ + (accZ - GRAVITY_EARTH);
        // Sum all readings
        this->AccAngleErrorX = this->AccAngleErrorX + this->calculateAccAngleX(accX, accY, accZ);
        this->AccAngleErrorY = this->AccAngleErrorY + this->calculateAccAngleX(accX, accY, accZ);
        delay(20);
    }
    //Divide the sum by 200 to get the error value
    this->AccErrorX = this->AccErrorX / times;
    this->AccErrorY = this->AccErrorY / times;
    this->AccErrorZ = this->AccErrorZ / times;
    this->AccAngleErrorX = this->AccAngleErrorX / times;
    this->AccAngleErrorY = this->AccAngleErrorY / times;
    Serial.println("");
    Serial.print("AccErrorX: ");Serial.println(this->AccErrorX);
    Serial.print("AccErrorY: ");Serial.println(this->AccErrorY);
    Serial.print("AccErrorZ: ");Serial.println(this->AccErrorZ);
    Serial.print("AccAngleErrorX: ");Serial.println(this->AccAngleErrorX);
    Serial.print("AccAngleErrorY: ");Serial.println(this->AccAngleErrorY);

    float gyroX, gyroY, gyroZ;
    this->GyroErrorX = 0.0;
    this->GyroErrorY = 0.0;
    this->GyroErrorZ = 0.0;
    for (int i = 0; i <= times; i++) {
        Wire.beginTransmission(MPU);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);
        rawGyroX = Wire.read() << 8 | Wire.read();
        rawGyroY = Wire.read() << 8 | Wire.read();
        rawGyroZ = Wire.read() << 8 | Wire.read();
        // Sum all readings
        gyroX = (rawGyroX / 131.0);
        gyroY = (rawGyroY / 131.0);
        gyroZ = (rawGyroZ / 131.0);
        this->GyroErrorX = this->GyroErrorX + gyroX;
        this->GyroErrorY = this->GyroErrorY + gyroY;
        this->GyroErrorZ = this->GyroErrorZ + gyroZ;
        delay(20);
    }
    this->GyroErrorX = this->GyroErrorX / times;
    this->GyroErrorY = this->GyroErrorY / times;
    this->GyroErrorZ = this->GyroErrorZ / times;

    Serial.println("");
    Serial.print("GyroErrorX: ");Serial.println(this->GyroErrorX);
    Serial.print("GyroErrorY: ");Serial.println(this->GyroErrorY);
    Serial.print("GyroErrorZ: ");Serial.println(this->GyroErrorZ);
}

void Mpu::read(){
    // === Read acceleromter data === //
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    rawAccX = (Wire.read() << 8 | Wire.read()); // X-axis value
    rawAccY = (Wire.read() << 8 | Wire.read()); // Y-axis value
    rawAccZ = (Wire.read() << 8 | Wire.read()); // Z-axis value

    accX = (rawAccX  / 16384.0) * GRAVITY_EARTH - this->AccErrorX;
    accY = (rawAccY  / 16384.0) * GRAVITY_EARTH - this->AccErrorY;
    accZ = (rawAccZ  / 16384.0) * GRAVITY_EARTH - this->AccErrorZ;

    Serial.print("   accX: ");Serial.print(accX);
    Serial.print("   accY: ");Serial.print(accY);
    Serial.print("   accZ: ");Serial.print(accZ);
    Serial.print("   m/(seg)^2    ");

    // Calculating Roll and Pitch from the accelerometer data
    accAngleX = this->calculateAccAngleX(accX, accY, accZ) - this->AccAngleErrorX; //from calibration
    accAngleY = this->calculateAccAngleX(accX, accY, accZ) - this->AccAngleErrorY; 

    // === Read gyroscope data === //
    this->PreviousTime = this->CurrentTime;        // Previous time is stored before the actual time read
    this->CurrentTime = millis();            // Current time actual time read
    this->ElapsedTime = (this->CurrentTime - this->PreviousTime) / 1000; // Divide by 1000 to get seconds
    Wire.beginTransmission(MPU);
    Wire.write(0x43); // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
    // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    rawGyroX = (Wire.read() << 8 | Wire.read()); 
    rawGyroY = (Wire.read() << 8 | Wire.read());
    rawGyroZ = (Wire.read() << 8 | Wire.read());

    gyroX = (rawGyroX / 131.0) - this->GyroErrorX; 
    gyroY = (rawGyroY / 131.0) - this->GyroErrorY;
    gyroZ = (rawGyroZ / 131.0) - this->GyroErrorZ;

    Serial.print("   gyroX: ");Serial.print(gyroX);
    Serial.print("   gyroY: ");Serial.print(gyroY);
    Serial.print("   gyroZ: ");Serial.print(gyroZ);
    Serial.print("   degree/seg         ");

    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
    this->GyroAngleX = this->GyroAngleX + gyroX * this->ElapsedTime; // deg/s * s = deg
    this->GyroAngleY = this->GyroAngleY + gyroY * this->ElapsedTime;
    this->GyroAngleZ = this->GyroAngleZ + gyroZ * this->ElapsedTime;
    
    //Serial.print("   angleX: ");Serial.print(this->GyroAngleX);
    //Serial.print("   angleY: ");Serial.print(this->GyroAngleY);



    // Complementary filter - combine acceleromter and gyro angle values
    roll  =  0.96 * this->GyroAngleX + 0.04 * accAngleX;
    pitch =  0.96 * this->GyroAngleY + 0.04 * accAngleY;
    yaw   =  this->GyroAngleZ;
    
    // Print the values on the serial monitor
    //Serial.print(roll);Serial.print("/");
    //Serial.print(pitch);Serial.print("/");
    //Serial.print(yaw);
    Serial.println();
}




float Mpu::calculateAccAngleX(float accX, float accY, float accZ){
    return (atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI);
}

float Mpu::calculateAccAngleY(float accX, float accY, float accZ){
    return (atan(-1 * accX/ sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI); 
}

Mpu::~Mpu(){

}
