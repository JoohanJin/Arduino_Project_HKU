/*
- need to calibrate and figure out about actual data.
*/

#include <dht11.h>
#include <SoftwareSerial.h>

#define DHT11PIN 4 // need to be configured when we integrate this into the car
#define BT_SERIAL_RX 15 // need to be configured when integrating into the car
#define BT_SERIAL_TX 14 // need to be configured when integrating into the car

// declare the dht11 sensor
dht11 DHT11;

// Bluetooth Serial Declaration
SoftwareSerial HC05(BT_SERIAL_RX, BT_SERIAL_TX);


//global variable 
float humi, temp;

void  setup()
{
  // TODO: test with the initialization of each Serial with different baud value -> Done
  Serial.begin(115200);
  HC05.begin(9600);

  // Start the DHT11 temperature/humidity sensor
  DHT11.begin()
}

// Function
void transferDataBluetooth(){
  HC05.print("Humidity is: ");
  HC05.println(humi);
  HC05.print("Temperature is: ");
  HC05.println(temp);
}

void transferErrorMSGBluetooth(){
  HC05.println("Environmental Data is not available currently!");
}

void humid_temp_collect()
{
  humi = DHT11.readHumidity();
  temp = DHT11.readTemperature();

  if (isnan(humi) || isnan(temp)){ // exception handling for 
    // print error message on BT
    transferErrorMSGBluetooth();
    return;
  }

  // transfer data to the connected android device.
  transferDataBluetooth();
}

void loop()
{
  humid_temp_collect_and_send();
}