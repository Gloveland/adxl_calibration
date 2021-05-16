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

void Mpu::read(){
    mpu.getEvent(&a, &g, &t);
    Serial.print("MPU:");
    this->readAcc(a);
    this->readGyro(g);
    this->readTemperature(t);

}

void Mpu::readAcc(sensors_event_t a) {
    Serial.print("       Ya=          "); Serial.print(a.acceleration.y); 
    Serial.print("       Xa=          "); Serial.print(a.acceleration.x); 
    Serial.print("       Za=          "); Serial.print(a.acceleration.z);
}

void Mpu::readGyro(sensors_event_t g){
    float gyroX_temp = g.gyro.x;
    //Serial.print(" TEMPX: "); Serial.print(gyroX_temp);
    if(abs(this->gyroX - gyroX_temp) > this->gyroError)  { 
        this->accGyroX -= (gyroX_temp - this->gyroXoffset)/50.00;
    }
    this->gyroX = gyroX_temp;
    
    float gyroY_temp = g.gyro.y;
    //Serial.print(" TEMPY: "); Serial.print(gyroY_temp);
    if(abs(this->gyroY - gyroY_temp) > this->gyroError) {
        this->accGyroY -= (gyroY_temp - this->gyroYoffset)/50.00;
    }
    this->gyroY = gyroY_temp;

    float gyroZ_temp = g.gyro.z;
    //Serial.print(" TEMPZ: "); Serial.print(gyroZ_temp);
    if(abs(this->gyroZ - gyroZ_temp) > this->gyroError) {
        this->accGyroZ -= (gyroZ_temp - this->gyroZoffset)/50.00;
    }
    this->gyroZ = gyroZ_temp;

    Serial.print("         ");
    Serial.print("       roll=");Serial.print(this->accGyroY);
    Serial.print("       pitch=");Serial.print(this->accGyroY);
    Serial.print("       yaw=");Serial.print(this->accGyroY);

}


void Mpu::readTemperature(sensors_event_t t){
    Serial.print("    t= "); Serial.println(t.temperature); 
}

Mpu::~Mpu(){

}
