#include <Mpu.h>

int16_t rawAccX, rawAccY, rawAccZ;
float accX, accY, accZ;
float accAngleX, accAngleY;
int16_t rawGyroX, rawGyroY, rawGyroZ;
float gyroX, gyroY, gyroZ;
float roll, pitch, yaw;
float degreesX, degreesY, degreesZ;

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
    float minGyroX = FLT_MAX;
    float minGyroY = FLT_MAX;
    float minGyroZ = FLT_MAX;
    float maxGyroX = -FLT_MAX;
    float maxGyroY = -FLT_MAX;
    float maxGyroZ = -FLT_MAX;
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
    
        if(gyroX < minGyroX){
            minGyroX = gyroX;
        }
        if(gyroY < minGyroY){
            minGyroY = gyroY;
        }
        if(gyroZ < minGyroZ){
            minGyroZ = gyroZ;
        }
        if(gyroX > maxGyroX){
            maxGyroX = gyroX;
        }
        if(gyroY > maxGyroY){
            maxGyroY = gyroY;
        }
        if(gyroZ > maxGyroZ){
            maxGyroZ = gyroZ;
        }
        this->GyroErrorX = this->GyroErrorX + gyroX;
        this->GyroErrorY = this->GyroErrorY + gyroY;
        this->GyroErrorZ = this->GyroErrorZ + gyroZ;

        this->CurrentTime = millis(); 
        delay(20);
    }
    this->GyroErrorX = this->GyroErrorX / times;
    this->GyroErrorY = this->GyroErrorY / times;
    this->GyroErrorZ = this->GyroErrorZ / times;

    this->DeviationX = (maxGyroX - minGyroX)/2.0;
    this->DeviationY = (maxGyroY - minGyroY)/2.0;
    this->DeviationZ = (maxGyroZ - minGyroZ)/2.0;

    this->PrevGyroX = 0.0;
    this->PrevGyroY = 0.0;
    this->PrevGyroZ = 0.0;

    Serial.println("");
    Serial.print("GyroErrorX: ");Serial.println(this->GyroErrorX);
    Serial.print("GyroErrorY: ");Serial.println(this->GyroErrorY);
    Serial.print("GyroErrorZ: ");Serial.println(this->GyroErrorZ);

    Serial.print("DeviationX: ");Serial.println(this->DeviationX);
    Serial.print("DeviationY: ");Serial.println(this->DeviationY);
    Serial.print("DeviationZ: ");Serial.println(this->DeviationZ);

    // calculamos la maxima desviacion de los angulos

    float degreesX = 0.0;
    float degreesY = 0.0;
    float degreesZ = 0.0;

    float minDegreesX = FLT_MAX;
    float minDegreesY = FLT_MAX;
    float minDegreesZ = FLT_MAX;

    float maxDegreesX = -FLT_MAX;
    float maxDegreesY = -FLT_MAX;
    float maxDegreesZ = -FLT_MAX;
    
    for (int i = 0; i <= times; i++) {

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

        gyroX = (rawGyroX / 131.0); 
        gyroY = (rawGyroY / 131.0);
        gyroZ = (rawGyroZ / 131.0);


        //Si la diferencia absoluta entre esta medicion y la anterior es menor a la desviacion maxima lo desestimo.
        if(abs(this->PrevGyroX - gyroX) > this->DeviationX){
            this->GyroX = gyroX - this->GyroErrorX; 
        }
        this->PrevGyroX = gyroX;

        if(abs(this->PrevGyroY - gyroY) > this->DeviationY){
            this->GyroY = gyroY - this->GyroErrorY;
        }
        this->PrevGyroY = gyroY;

        if(abs(this->PrevGyroZ - gyroZ) > this->DeviationZ){
            this->GyroZ = gyroZ - this->GyroErrorZ;
        }
        this->PrevGyroZ= gyroZ;

        // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
        degreesX = this->GyroX * this->ElapsedTime; // deg/s * s = deg
        degreesY = this->GyroY * this->ElapsedTime;
        degreesZ = this->GyroZ * this->ElapsedTime;

        Serial.print("DegreesX: ");Serial.println(degreesX);
        Serial.print("DegreesY: ");Serial.println(degreesY);
        Serial.print("DegreesZ: ");Serial.println(degreesZ);

        if(degreesX < minDegreesX){
            minDegreesX = degreesX;
        }
        if(degreesY < minDegreesY){
            minDegreesY = degreesY;
        }
        if(degreesZ < minDegreesZ){
            minDegreesZ = degreesZ;
        }
        if(degreesX > maxDegreesX){
            maxDegreesX = degreesX;
        }
        if(degreesY > maxDegreesY){
            maxDegreesY = degreesY;
        }
        if(degreesZ > maxDegreesZ){
            maxDegreesZ = degreesZ;
        }
        delay(200);
    }

    this->DegreesDeviationX = max(abs(maxDegreesX), abs(minDegreesX));
    this->DegreesDeviationY = max(abs(maxDegreesY), abs(minDegreesY));
    this->DegreesDeviationZ = max(abs(maxDegreesZ), abs(minDegreesZ));

    Serial.print("DegreesDeviationX: ");Serial.println(this->DegreesDeviationX);
    Serial.print("DegreesDeviationY: ");Serial.println(this->DegreesDeviationY);
    Serial.print("DegreesDeviationZ: ");Serial.println(this->DegreesDeviationZ);


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

    //Serial.print("   aX: ");Serial.print(accX);
    //Serial.print("   aY: ");Serial.print(accY);
    //Serial.print("   aZ: ");Serial.print(accZ);
    //Serial.print("   m/(seg)^2    ");

    // Calculating Roll and Pitch from the accelerometer data
    accAngleX = this->calculateAccAngleX(accX, accY, accZ) - this->AccAngleErrorX; //from calibration
    accAngleY = this->calculateAccAngleX(accX, accY, accZ) - this->AccAngleErrorY; 

    //Serial.print("   anX: ");Serial.print(accAngleX );
    //Serial.print("   anY: ");Serial.print(accAngleY );

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

    gyroX = (rawGyroX / 131.0); 
    gyroY = (rawGyroY / 131.0);
    gyroZ = (rawGyroZ / 131.0);


    //Si la diferencia absoluta entre esta medicion y la anterior es menor a la desviacion maxima lo desestimo.
    if(abs(this->PrevGyroX - gyroX) > this->DeviationX){
        this->GyroX = gyroX - this->GyroErrorX; 
    }
    this->PrevGyroX = gyroX;

    if(abs(this->PrevGyroY - gyroY) > this->DeviationY){
        this->GyroY = gyroY - this->GyroErrorY;
    }
    this->PrevGyroY = gyroY;

    if(abs(this->PrevGyroZ - gyroZ) > this->DeviationZ){
        this->GyroZ = gyroZ - this->GyroErrorZ;
    }
    this->PrevGyroZ= gyroZ;

    Serial.print("   gX: ");Serial.print(this->GyroX);
    Serial.print("   gY: ");Serial.print(this->GyroY);
    Serial.print("   gZ: ");Serial.print(this->GyroZ);
    //Serial.print("   degrees/seg         ");

    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees

    degreesX = this->GyroX * this->ElapsedTime; // deg/s * s = deg
    degreesY = this->GyroY * this->ElapsedTime;
    degreesZ = this->GyroZ * this->ElapsedTime;

    if(abs(degreesX) > this->DegreesDeviationX){
        this->GyroAngleX = this->GyroAngleX + degreesX;
    }

    if(abs(degreesY) > this->DegreesDeviationY){
        this->GyroAngleY = this->GyroAngleY + degreesY;
    }

    if(abs(degreesZ) > this->DegreesDeviationZ){
        this->GyroAngleZ = this->GyroAngleZ + degreesZ;
    }
    
    Serial.print("   degX: ");Serial.print(this->GyroAngleX);
    Serial.print("   degY: ");Serial.print(this->GyroAngleY);
    Serial.print("   degZ: ");Serial.print(this->GyroAngleZ);



    // Complementary filter - combine acceleromter and gyro angle values
    roll  = this->GyroAngleX *  0.96 +  accAngleX * 0.04;
    pitch = this->GyroAngleY *  0.96 +  accAngleY * 0.04;
    yaw   = this->GyroAngleZ;


    
    
    // Print the values on the serial monitor
    //Serial.print("  roll:"); Serial.print(roll);
    //Serial.print("  pitch:");Serial.print(pitch);
    //Serial.print("  yaw:"); Serial.print(yaw);
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
