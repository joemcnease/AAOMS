// Arduino_MKRENV - Version: Latest
#include <Arduino_MKRENV.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"

/*
*/

// Network
char ssid[] = SECRET_SSID;    // WIFI network SSID
char pass[] = SECRET_PASS;    // WIFI network Password
int status = WL_IDLE_STATUS;  // WIFI radio status

// Timer
double timer = 0;             // Timer
double loopTime = 5000;       // Microseconds

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // break
    while (true);
  }
  
  String fv = WiFi.firmwareVersion();
  // Check if wifi firmware is up to date
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.print("WiFi firmware version: ");
    Serial.println(fv);
    Serial.print("WiFi firmware version needed: ");
    Serial.println(WIFI_FIRMWARE_LATEST_VERSION);
    Serial.println("Please upgrade to the latest firmware.");
  }

  // Attempt to connect to wifi
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to network: ");
    Serial.println(ssid);
   
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    
    // Wait 10 seconds for connection:
    delay(10000);
  }
  
  // We are now connected
  Serial.println("Network Status: Connected");
  Serial.println("---------------------------------------");
  printWifiData();
  Serial.println("---------------------------------------");  

  timer = micros();

  if (!ENV.begin()) {
    Serial.println("Failed to initialize MKR ENV shield!");
    while (1);
  }
}

void loop() {
  // get time
  timeSync(loopTime);
  timer = micros();
  
  // read all the sensor values
  float temperature = ENV.readTemperature();
  float humidity    = ENV.readHumidity();
  float pressure    = ENV.readPressure();
  float illuminance = ENV.readIlluminance();
  float uva         = ENV.readUVA();
  float uvb         = ENV.readUVB();
  float uvIndex     = ENV.readUVIndex();
  
  sendToPC(&temperature);
  sendToPC(&humidity);
  sendToPC(&pressure);
  sendToPC(&illuminance);
  sendToPC(&uva);
  sendToPC(&uvb);
  sendToPC(&uvIndex);

  // wait 1 second to print again
  delay(1000);
}

void printWifiData() {
  Serial.println("Board Information:");
  // Print board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  
  Serial.println();
  Serial.println("Network Information:");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  
  // Print received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("Singal Strength: ");
  Serial.print(rssi);
}

void timeSync(unsigned long deltaT) {
  double currTime = micros();
  long timeToDelay = deltaT - (currTime - timer);
  if (timeToDelay > 5000) {
    delay(timeToDelay / 1000);
    delayMicroseconds(timeToDelay % 1000);
  }
  else if (timeToDelay > 0) {
    delayMicroseconds(timeToDelay);
  }
  else {
    // timeToDelay is negative so we start immediately
  }
  timer = currTime + timeToDelay;
}

void sendToPC(int* data) {
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 2);
}

void sendToPC(float* data) {
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 4);
}

void sendToPC(double* data) {
  byte* byteData = (byte*)(data);
  Serial.write(byteData, 8);
}
