#include <Mpu.h>


Mpu::Mpu(){

}

void Mpu::init(){
    while(!this->mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        delay(10);
    }
    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    if (mpu.getAccelerometerRange() == MPU6050_RANGE_8_G) {
        Serial.println("Accelerometer range set to: +-8G");
    }
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    if(mpu.getGyroRange() == MPU6050_RANGE_500_DEG) {
        Serial.println("Gyro range set to: +- 500 deg/s");
    }
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    if (mpu.getFilterBandwidth() ==  MPU6050_BAND_21_HZ) {
        Serial.println("Filter bandwidth set to: 21 Hz");
    }
    Serial.println("");
    delay(100);
}

void Mpu::calibrate(){
    Serial.println("===================== Calibrating MPU6050 ===================="); 

    float accAngleX,accAngleY;

    int times = 100;
    for (int i = 0; i <= times; i++) {
        mpu.getEvent(&a, &g, &t);

        this->accErrorX = this->accErrorX + a.acceleration.x;
        this->accErrorY = this->accErrorY + a.acceleration.y;
        this->accErrorZ = this->accErrorZ + (a.acceleration.z - 9.8106);

        this->gyroErrorX = this->gyroErrorX  + (g.gyro.x/SENSORS_DPS_TO_RADS);
        this->gyroErrorY = this->gyroErrorY  + (g.gyro.y/SENSORS_DPS_TO_RADS);
        this->gyroErrorZ = this->gyroErrorZ  + (g.gyro.z/SENSORS_DPS_TO_RADS);

        Serial.print(".");
        //Serial.print("  accX=");Serial.print(a.acceleration.x);
        //Serial.print("  accY=");Serial.print(a.acceleration.y);
        //Serial.print("  accZ=");Serial.print(a.acceleration.z);
        //Serial.print("  gx=");Serial.print(g.gyro.x/SENSORS_DPS_TO_RADS);
        //Serial.print("  gy=");Serial.print(g.gyro.y/SENSORS_DPS_TO_RADS);
        //Serial.print("  gz=");Serial.println(g.gyro.z/SENSORS_DPS_TO_RADS);
        
        delay(100);
        
    } 
    this->accErrorX = this->accErrorX/times;
    this->accErrorY = this->accErrorY/times;
    this->accErrorZ = this->accErrorZ/times;
    
    this->gyroErrorX = this->gyroErrorX/times;
    this->gyroErrorY = this->gyroErrorY/times;
    this->gyroErrorZ = this->gyroErrorZ/times;

    //Serial.println("");
    //Serial.print("AccErrorX: ");Serial.println(this->accErrorX);
    //Serial.print("AccErrorY: ");Serial.println(this->accErrorY);
    //Serial.print("AccErrorZ: ");Serial.println(this->accErrorZ);
    //Serial.print("GyroErrorX: ");Serial.println(this->gyroErrorX);
    //Serial.print("GyroErrorY: ");Serial.println(this->gyroErrorY);
    //Serial.print("GyroErrorZ: ");Serial.println(this->gyroErrorZ);

    float accX, accY, accZ = 0;
    for (int i = 0; i <= times; i++) {
        mpu.getEvent(&a, &g, &t);

        accX = a.acceleration.x - this->accErrorX;
        accY = a.acceleration.y - this->accErrorY;
        accZ = a.acceleration.z - this->accErrorZ;

        accAngleX = this->calculateAccAngleX(accX, accY, accZ);
        accAngleY = this->calculateAccAngleY(accX, accY, accZ);

        this->accAngleErrorX = this->accAngleErrorX + accAngleX;
        this->accAngleErrorY = this->accAngleErrorY + accAngleY;

        Serial.print("  accX=");Serial.print(accX);
        Serial.print("  accY=");Serial.print(accY);
        Serial.print("  accZ=");Serial.print(accZ);
        Serial.print("  accAngleX=");Serial.print(accAngleX);
        Serial.print("  accAngleY=");Serial.println(accAngleY);
    }
    this->accAngleErrorX = this->accAngleErrorX/times;
    this->accAngleErrorY = this->accAngleErrorY/times;

    Serial.println("");
    Serial.print("AccAngleErrorX: ");Serial.println(this->accAngleErrorX);
    Serial.print("AccAngleErrorY: ");Serial.println(this->accAngleErrorY);
}

void Mpu::read(){
    mpu.getEvent(&a, &g, &t);
    Serial.print("MPU:");
    //this->readAcc(a);
    float accX, accY, accZ = 0;
    accX = a.acceleration.x - this->accErrorX;
    accY = a.acceleration.y - this->accErrorY;
    accZ = a.acceleration.z - this->accErrorZ; 
    Serial.print("       Xa=          "); Serial.print(accX); 
    Serial.print("       Ya=          "); Serial.print(accY);
    Serial.print("       Za=          "); Serial.print(accZ);
    this->readGyro(g, accX, accY, accZ);
    this->readTemperature(t);

}

void Mpu::readAcc(sensors_event_t a) {
    float accX, accY, accZ = 0;
    accX = a.acceleration.x - this->accErrorX;
    accY = a.acceleration.y - this->accErrorY;
    accZ = a.acceleration.z - this->accErrorZ; 
    Serial.print("       Xa=          "); Serial.print(accX); 
    Serial.print("       Ya=          "); Serial.print(accY);
    Serial.print("       Za=          "); Serial.print(accZ);
}

void Mpu::readGyro(sensors_event_t g, float accX, float accY, float accZ){
    previousTime = currentTime;
    currentTime = millis();            // Current time actual time read
    this->elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds 

    this->gyroAngleX = this->gyroAngleX + ((g.gyro.x/SENSORS_DPS_TO_RADS) - this->gyroErrorX) * this->elapsedTime; // deg/s * s = deg
    this->gyroAngleY = this->gyroAngleY + ((g.gyro.y/SENSORS_DPS_TO_RADS) - this->gyroErrorY) * this->elapsedTime;
    this->yaw        = this->yaw        + ((g.gyro.z/SENSORS_DPS_TO_RADS) - this->gyroErrorZ) * this->elapsedTime;

    accAngleX = this->calculateAccAngleX(accX, accY, accZ) - this->accAngleErrorX; // error from calibation()
    accAngleY = this->calculateAccAngleY(accX, accY, accZ) - this->accAngleErrorY; // 

    //this->roll  = 0.96 * gyroAngleX + 0.04 * accAngleX;
    //this->pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

    Serial.print("         ");
    Serial.print("       roll=");Serial.print(accAngleX);
    Serial.print("       pitch=");Serial.print(accAngleY);
    Serial.print("       yaw=");Serial.print(this->yaw);
}


void Mpu::readTemperature(sensors_event_t t){
    Serial.print("    t= "); Serial.println(t.temperature); 
}

float Mpu::calculateAccAngleX(float accX, float accY, float accZ){
    return (atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI);
}

float Mpu::calculateAccAngleY(float accX, float accY, float accZ){
    return (atan(-1 * accX/ sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI); 
}

Mpu::~Mpu(){

}
