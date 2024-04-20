#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>


// need to initalize each element for ultrasonic and photo-register
// Ultrasonic
// need to figure out the pin #
#define echoPin 11
#define trigPin 12

long distance_in_cm;


// photo register
//variables for light intensity to ADC reading equations 
int int_adc0, int_adc0_c, int_adc0_m; // from the left
int int_adc1, int_adc1_c, int_adc1_m; // from the right
int int_left, int_right;


// Gyro
MPU6050 mpu(Wire);
unsigned long timer = 0;

void setup(){
  SSerial.begin(115200);                                        
  Wire.begin();
  mpu.begin();
  Serial.println("Calculating gyro offset, do not move MPU6050");
  Serial.println("............");
  delay(1000);
  mpu.calcGyroOffsets();                          // This does the calibration
  Serial.println("Done");  
}

void measure_distance(){
  long duration;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance_in_cm = (duration/2.0) / 29.1;
}

void measure_light_intensity(){
  int_left=(analogRead(A0)-int_adc0_c)/int_adc0_m;
  int_right=(analogRead(A1)-int_adc1_c)/int_adc1_m;

  Serial.print("from left: ");
  Serial.println(int_right);
  Serial.print("from right: ");
  Serial.println(int_left);
}

void loop(){
  mpu.update();  
  if((millis()-timer)>10)                         // print data every 10ms
  {                                             
    Serial.print("Pitch (Angle_X) : ");
    Serial.print(mpu.getAngleX());
    Serial.print("  Roll  (Angle_Y) : ");
    Serial.print(mpu.getAngleY());
    Serial.print("  Yaw   (Angle_Z) : ");
    Serial.println(mpu.getAngleZ());
    //Serial.println("==========================");
    timer = millis();  
    //delay(1000);
  }
}