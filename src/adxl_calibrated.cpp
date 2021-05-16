

/*esta prueba funciona mas o menos bien con un data rate de 2g. lee 1 cuando x esta para arriba */
#include <Adxl.h>
#include <Mpu.h>

Adxl adxl345;
Mpu mpu6050;


void setup() {
    Serial.begin(9600);               // Initiate serial communication for printing the results on the Serial monitor
    mpu6050.init();
    adxl345.init();
    Serial.println();
    Serial.println("Type key when ADXL345 Sensor is placed over an horizontal plane: X = 0g, Y = 0g, Z = +1g orientation"); 
    while (!Serial.available()){
      //wait for a character 
    }  
    // === Calibration === //
    adxl345.calibrate();
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

void loop() {
  adxl345.read();
  mpu6050.read();
  delay(200);
}
