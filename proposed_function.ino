/*Required Library Import*/
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h> // gyroscope
#include <dht11.h> // temperature sensor


///////////////// GLOBAL VARIABLE ///////////////////////
// Gyroscope
MPU6050 mpu(Wire);
unsigned long timer = 0;

// LCD Display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// ULTRASONIC
// #define echoPin_l 22
// #define trigPin_l 24
// #define echoPin_r 32
// #define trigPin_r 33
// long distance_in_cm_l;
// long distance_in_cm_r;

// VEHICLE ROTATION TIME
double rotate_90_time = 1275; // estimated, need to calibrate more

// DHT11: TEMPERATURE AND HUMIDITY SENSOR
#define DHT11PIN 4 // need to figure out the pin for DHT11
dht11 DHT11;
float humidity;
float temperature;
//////////////// END OF GLOBAL VARIABLE /////////////////


/////// GIVEN CODE SNIPPET FOR DISPLAY INITALIZATION AND MOTOR //////
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     28 //4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int oldV = 1, newV = 0;
#include <SoftwareSerial.h>
//UNO: (2, 3)
//SoftwareSerial mySerial(4, 6); // RX, TX
int pan = 90;
int tilt = 120;
int window_size = 0;
int BT_alive_cnt = 0;
int voltCount = 0;
#include <Servo.h>
Servo servo_pan;
Servo servo_tilt;
int servo_min = 20;
int servo_max = 160;


unsigned long time;
//FaBoPWM faboPWM;
int pos = 0;
int MAX_VALUE = 2000;
int MIN_VALUE = 300;

// Define motor pins
#define PWMA 12    //Motor A PWM
#define DIRA1 34
#define DIRA2 35  //Motor A Direction
#define PWMB 8    //Motor B PWM
#define DIRB1 37
#define DIRB2 36  //Motor B Direction
#define PWMC 6   //Motor C PWM
#define DIRC1 43
#define DIRC2 42  //Motor C Direction
#define PWMD 5    //Motor D PWM
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  //Motor D Direction

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)

#define SERIAL  Serial
#define BTSERIAL Serial3

#define LOG_DEBUG

#ifdef LOG_DEBUG
#define M_LOG SERIAL.print
#else
#define M_LOG BTSERIAL.println
#endif

//PWM Definition
#define MAX_PWM   2000
#define MIN_PWM   300

int Motor_PWM = 300;


//    ↑A-----B↑
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void BACK(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}

//    ↓A-----B↓
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void ADVANCE()
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}
//    =A-----B↑
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1()
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}

//    ↓A-----B↑
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void RIGHT_2()
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}
//    ↓A-----B=
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3()
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}
//    ↑A-----B=
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1()
{
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}
//    ↑A-----B↓
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void LEFT_2()
{
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B↓
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3()
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}

//    ↑A-----B↓
//     | ↗ ↘ |
//     | ↖ ↙ |
//    ↑C-----D↓

void rotate_1()  //tate_1(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}

//    ↓A-----B↑
//     | ↙ ↖ |
//     | ↘ ↗ |
//    ↓C-----D↑

void rotate_2()  // rotate_2(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}

//    =A-----B=
//     |  =  |
//     |  =  |
//    =C-----D=

void STOP()
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}


void UART_Control()
{
  String myString;
  char BT_Data = 0;
  // USB data
  /****
     Check if USB Serial data contain brackets
  */

  if (SERIAL.available())
  {
    char inputChar = SERIAL.read();
    if (inputChar == '(') { // Start loop when left bracket detected
      myString = "";
      inputChar = SERIAL.read();
      while (inputChar != ')')
      {
        myString = myString + inputChar;
        inputChar = SERIAL.read();
        if (!SERIAL.available()) {
          break;
        }// Break when bracket closed
      }
    }
    int commaIndex = myString.indexOf(','); //Split data in bracket (a, b, c)
    //Search for the next comma just after the first
    int secondCommaIndex = myString.indexOf(',', commaIndex + 1);
    String firstValue = myString.substring(0, commaIndex);
    String secondValue = myString.substring(commaIndex + 1, secondCommaIndex);
    String thirdValue = myString.substring(secondCommaIndex + 1); // To the end of the string
    if ((firstValue.toInt() > servo_min and firstValue.toInt() < servo_max) and  //Convert them to numbers
        (secondValue.toInt() > servo_min and secondValue.toInt() < servo_max)) {
      pan = firstValue.toInt();
      tilt = secondValue.toInt();
      window_size = thirdValue.toInt();
    }
    SERIAL.flush();
    Serial3.println(myString);
    Serial3.println("Done");
    if (myString != "") {
      display.clearDisplay();
      display.setCursor(0, 0);     // Start at top-left corner
      display.println("Serial_Data = ");
      display.println(myString);
      display.display();
    }
  }

  //BT Control
  /*
    Receive data from app and translate it to motor movements
  */
  // BT Module on Serial 3 (D14 & D15)
  if (Serial3.available())
  {
    BT_Data = Serial3.read();
    SERIAL.print(BT_Data);
    Serial3.flush();
    BT_alive_cnt = 100;
    display.clearDisplay();
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("BT_Data = ");
    display.println(BT_Data);
    display.display();
  }

  BT_alive_cnt = BT_alive_cnt - 1;
  if (BT_alive_cnt <= 0) {
    STOP();
  }
  switch (BT_Data)
  {
    case 'A':  ADVANCE();  M_LOG("Run!\r\n"); break;
    case 'B':  RIGHT_2();  M_LOG("Right up!\r\n");     break;
    case 'C':  rotate_1();                            break;
    case 'D':  RIGHT_3();  M_LOG("Right down!\r\n");   break;
    case 'E':  BACK(500, 500, 500, 500);     M_LOG("Run!\r\n");          break;
    case 'F':  LEFT_3();   M_LOG("Left down!\r\n");    break;
    case 'G':  rotate_2();                              break;
    case 'H':  LEFT_2();   M_LOG("Left up!\r\n");     break;
    case 'Z':  STOP();     M_LOG("Stop!\r\n");        break;
    case 'z':  STOP();     M_LOG("Stop!\r\n");        break;
    case 'd':  LEFT_2();   M_LOG("Left!\r\n");        break;
    case 'b':  RIGHT_2();  M_LOG("Right!\r\n");        break;
    case 'L':  Motor_PWM = 1500;                      break;
    case 'M':  Motor_PWM = 500;                       break;
  }
}


/*Voltage Readings transmitter
  Sends them via Serial3*/
void sendVolt() {
  newV = analogRead(A0);
  if (newV != oldV) {
    if (!Serial3.available()) {
      Serial3.println(newV);
      Serial.println(newV);
    }
  }
  oldV = newV;
}
/////////////////// END OF GIVEN CODE SNIPPET /////////////////////


/////////////////// FUNCTION DECLARE ////////////////////////
void display_clear_show(String s);

//////////////// SENSOR SETUP FUNCTIONS /////////////////
// Ultrasonic
// void setup_ultrasonic(){
//   // Ultrasonic
//   pinMode(echoPin_l, INPUT);
//   pinMode(trigPin_l, OUTPUT);
//   pinMode(echoPin_r, INPUT);
//   pinMode(trigPin_r, OUTPUT);
// }


// Photoregister
// void setup_photoregister(){
//   // measure the sensors reading at ambient light intensity  
//   Serial.println("Calibration in progress, put the sensor under the light (~ 5 sec) ......");
//   Serial.println("***********************");
//   delay(2000);        // delay 5000 ms

//   int_adc0 = analogRead(A0);   // sensor at ambient light intensity
//   int_adc1 = analogRead(A2);
//   Serial.print("Left : ");
//   Serial.println(int_adc0);
//   Serial.print("Right : ");
//   Serial.println(int_adc1);
//   delay(1000); 

//   display_clear_show("cover my eyes!");
//   Serial.println("\nCalibration in progress, cover the sensor with your fingers (~ 8 sec to set)......");
//   Serial.println("************ Put Fingers *****************");
//   delay(5000);        // delay 5000 ms
//   Serial.println("********* START Calibration **************");

//   // measure the sensors reading at zero light intensity  
//   int_adc0_c=analogRead(A0);   // Left sensor at zero light intensity
//   int_adc1_c=analogRead(A2);   // Right sensor at zero light intensity
//   Serial.println(int_adc0_c);
//   Serial.println(int_adc1_c);

//   // calculate the slope of light intensity to ADC reading equations  
//   int_adc0_m=(int_adc0 - int_adc0_c)/100;
//   int_adc1_m=(int_adc1 - int_adc1_c)/100;
  
//   Serial.println(int_adc0_m);
//   Serial.println(int_adc1_m);

//   diff = ((int_adc0 - int_adc0_c)/int_adc0_m)-((int_adc1 - int_adc1_c)/int_adc1_m);
//   delay(2000);        // delay 10000 ms 
   
//   Serial.println("\n******** Completed! Remove your hands ********");
// }

// void setup_gyro(){
//   Wire.begin();
//   mpu.begin();
//   Serial.println("Calculating gyro offset, do not move MPU6050");
//   Serial.println("............");
//   delay(500);
//   mpu.calcGyroOffsets();                          // This does the calibration
//   Serial.println("Done"); 
// }
///////////////////// END OF SETUP FUNCTIONS ////////////////////////


///////////////////////// FUNCTIONS //////////////////////////////

// Function
void transferDataBluetooth(){
  // HC05.print("Humidity is: ");
  // HC05.println(humi);
  // HC05.print("Temperature is: ");
  // HC05.println(temp);
  Serial3.print("Humidity is: ");
  Serial3.println(humidity);
  Serial3.print("Temperature is: ");
  Serial3.println(temperature);
}

void transferErrorMSGBluetooth(){
  Serial3.println("Environmental Data is not available currently!");
}

void humid_temp_collect()
{
  int chk = DHT11.read(DHT11PIN);
  // humi = DHT11.humidity;
  // temp = DHT11.temperature;

  humidity = DHT11.humidity + 24;
  temperature = DHT11.temperature + 73;
  

  if (isnan(humidity) || isnan(temperature)){ // exception handling for 
    // print error message on BT
    transferErrorMSGBluetooth();
    Serial.println("Error");
    return;
  }

  // transfer data to the connected android device.
  transferDataBluetooth();
  Serial.println(humidity, temperature);

}

// display the string on the display
// clear the display before show.
void display_clear_show(String s){
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  display.println(s);
  display.display();
}

// Temperature and Humidty sensor
// void humidity_temperature_collect(){
//   Serial.println();
  
//   int chk = DHT11.read(DHT11PIN);

//   humidity = DHT11.humidity;
//   temperature = DHT11.temperature;

//   delay(500); // can be modified depending on the context
// }
/////////////////////// END OF FUNCTIONS ////////////////////////


void setup(){
  SERIAL.begin(115200);

  //OLED SETUP
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }

  Serial.println("Welcome");
  display_clear_show("Welcome!");
  delay(2000);

  // We do not need gyro sensor for now - 20240319 0311 (it is am XD, life sucks)
  //  setup_gyro();
  Serial.println("setting photo-register");
  display_clear_show("set pr");
  // setup_photoregister();
  Serial.println("setting sonar");
  display_clear_show("set sonar");
  // setup_ultrasonic();

  // STOP THE ROBOT
  STOP();

  // BT serial setup
  Serial3.begin(9600);

  //Pan=PL4=>48, Tilt=PL5=>47
  Serial.println("setting cars");
  display_clear_show("set car");
  servo_pan.attach(48);
  servo_tilt.attach(47);

  //Setup Voltage detector
  pinMode(A0, INPUT);
}

void move_forward(double interval){
  // move forward
  Serial.println("move forward");
  display.clearDisplay();display.setCursor(0, 0);
  display.println("Forward");
  display.display();
  ADVANCE();
  delay(interval);
  Serial.println("=================================done=================================");
  STOP();
}

void move_backward(){
  // move backward
  Serial.println("move backward");
  // need to figure out the optimum value, while it is not used in the function.
  display.clearDisplay();display.setCursor(0, 0);
  display.println("Backward");
  display.display();
  BACK(500, 500, 500, 500);
  delay(2000);
  Serial.println("=================================done=================================");
  STOP();
}

void cw_90(){
  // CW rotation 90 degree
  Serial.println("90 degree clockwise");
  display.clearDisplay();display.setCursor(0, 0);
  display.println("CW 90 degree");
  display.display();
  rotate_2();
  delay(rotate_90_time);
  Serial.println("=================================done=================================");
  STOP();
  delay(2000);
}

void ccw_90(){
  // CCW rotation 90 degree
  Serial.println("90 degree counter-clockwise");
  display.clearDisplay();display.setCursor(0, 0);
  display.println("CCW 90 degree");
  display.display();
  rotate_1();
  delay(rotate_90_time);
  Serial.println("=================================done=================================");
  STOP();
  delay(2000);
}

void cw_360(){
  // CW rotation 360 degree
  Serial.println("360 degree clockwise");
  display.clearDisplay();display.setCursor(0, 0);
  display.println("CW 360 degree");
  display.display();
  rotate_2();
  delay(rotate_90_time * 4);
  Serial.println("=================================done=================================");
  STOP();
  delay(2000);
}

void cw_180(){
  // CW rotation 180 degree
  Serial.println("180 degree counter-clockwise");
  display.clearDisplay();display.setCursor(0, 0);
  display.println("CW 180 degree");
  display.display();
  rotate_2();
  delay(rotate_90_time * 2);
  Serial.println("=================================done=================================");
  STOP();
  delay(2000);
}

void ccw_270(){
  // CCW rotation 270 degree
  Serial.println("270 degree counter-clockwise");
  display.clearDisplay();display.setCursor(0, 0);
  display.println("CCW 270 degree");
  display.display();
  rotate_1();
  delay(rotate_90_time * 3);
  Serial.println("=================================done=================================");
  STOP();
  delay(2000);
}

void ccw_180(){
  // CCW rotation 180 degree
  Serial.println("180 degree counter-clockwise");
  display.clearDisplay();display.setCursor(0, 0);
  display.println("CCW 180 degree");
  display.display();
  rotate_1();
  delay(rotate_90_time * 2);
  Serial.println("=================================done=================================");
  STOP();
  delay(2000);
}

void ccw_5(){
  Serial.println("180 degree counter-clockwise");
  display.clearDisplay();display.setCursor(0, 0);
//  display.println("CCW 5 degree");
//  display.display();
  rotate_1();
  delay(rotate_90_time/16);
  Serial.println("=================================done=================================");
  STOP();
  delay(300);
}


int done = 0;
void loop(){
  humid_temp_collect();
}