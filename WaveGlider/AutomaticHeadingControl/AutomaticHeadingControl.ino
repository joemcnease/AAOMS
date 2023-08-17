#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <AccelStepper.h>
#include <Arduino_MKRENV.h>
#include <Arduino_MKRGPS.h>


#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
AccelStepper stepper(AccelStepper::DRIVER, 9, 8);

char inputBuffer[16];
float target = 0;
float latitude;
float longitude;
float altitude;
float speed;
int satellites;
uint8_t sys, gyro, accel, mag;
bool collectData = true;


int headingToStep(float heading)
{
  return (heading/360)*1600;
}


void displaySensorDetails()
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    "); Serial.print(sensor.max_value);
  Serial.println(" xxx");
  Serial.print("Min Value:    "); Serial.print(sensor.min_value);
  Serial.println(" xxx");
  Serial.print("Resolution:   "); Serial.print(sensor.resolution);
  Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


void displaySensorStatus()
{
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}


void displaySensorOrientation(float x, float y, float z,
                              uint8_t s, uint8_t g, uint8_t a, uint8_t m)
{
  Serial.print("X: "); Serial.print(x, 4);
  Serial.print(", Y: "); Serial.print(y, 4);
  Serial.print(", Z: "); Serial.print(z, 4);
  Serial.print("  ||  ");
  if (!s) { Serial.print("!"); }

  Serial.print("Sys: "); Serial.print(s, DEC);
  Serial.print(", Gyro: "); Serial.print(g, DEC);
  Serial.print(", Accel: "); Serial.print(a, DEC);
  Serial.print(", Mag: "); Serial.print(m, DEC);
  Serial.println("");
}

void displayEnvironmentData(float temp, float humidity, float pressure,
                            float illuminance, float uva, float uvb,
                            float uvindex)
{
  Serial.print("Temp: "); Serial.print(temp, 4);
  Serial.print(", Hum: "); Serial.print(humidity, 4);
  Serial.print(", Pres: "); Serial.print(pressure, 4);
  Serial.print(", Ill: "); Serial.print(illuminance, 4);
  Serial.print(", UVA: "); Serial.print(uva, 4);
  Serial.print(", UVB: "); Serial.print(uvb, 4);
  Serial.print(", UVIdx: "); Serial.print(uvindex, 4);
  Serial.println("");
}

void displayGPSData(float lat, float lon, float alt, float speed, int sats)
{
  Serial.print("Lat: "); Serial.print(lat, 7);
  Serial.print(", Lon: "); Serial.print(lon, 7);
  Serial.print(", Alt: "); Serial.print(alt);
  Serial.print(", Speed: "); Serial.print(speed);
  Serial.print(", # Sats: "); Serial.print(sats);
  Serial.println("");
  Serial.println("");
}


bool sensorUsable(uint8_t &sys, uint8_t &gyro, uint8_t &accel, uint8_t mag)
{
  if (sys > 0 && gyro > 0 && accel > 0 && mag > 0) { return true; }
  return false;
}


void setup()
{
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  pinMode(3, INPUT);
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(1000);

  Serial.begin(9600);

  Serial.println("Orientation Sensor Test");
  Serial.println("");

  if (!ENV.begin()) {
    Serial.println("Failed to initialize MKR ENV Shield!");
    while (1);
  }

  if (!GPS.begin()) {
    Serial.println("Failed to initialize the GPS!");
  }

  if (!bno.begin()) {
    Serial.print("No BNO055 detected. Check wiring and/or I2C address (0x28 or 0x29).");
    while(1);
  }

  delay(1000);
  displaySensorDetails();
  displaySensorStatus();
  bno.setExtCrystalUse(true);

}

void loop()
{
  if (collectData) {
    if (Serial.available() > 0) {
      Serial.readBytes(inputBuffer, sizeof(inputBuffer));
      float temp = atoi(inputBuffer);
      if (temp >= 0 && temp <= 360) {
        target = temp;
        int stepPos = headingToStep(target);
        stepper.moveTo(target);
      }
      memset(inputBuffer, 0, sizeof(inputBuffer));
    }

    // Collect sensor data
    float temperature = ENV.readTemperature();
    float humidity    = ENV.readHumidity();
    float pressure    = ENV.readPressure();
    float illuminance = ENV.readIlluminance();
    float uva         = ENV.readUVA();
    float uvb         = ENV.readUVB();
    float uvIndex     = ENV.readUVIndex();

    // Get gps location if changed
    if (GPS.available()) {
      // read GPS values
      latitude   = GPS.latitude();
      longitude  = GPS.longitude();
      altitude   = GPS.altitude();
      speed      = GPS.speed();
      satellites = GPS.satellites();
    }

    // Determine orientation.
    // NOTE: x = heading, y = pitch, z = roll
    sensors_event_t event;
    bno.getEvent(&event);
    float x = event.orientation.x;
    float y = event.orientation.y;
    float z = event.orientation.z;
    
    // LED for calibration status
    if (sensorUsable(sys, gyro, accel, mag)) {
      digitalWrite(4, HIGH);
      digitalWrite(5, LOW);
    }
    else {
      digitalWrite(4, LOW);
      digitalWrite(5, HIGH);
    }

    // Check calibration status first!
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    displaySensorOrientation(x, y, z, sys, gyro, accel, mag);
    displayEnvironmentData(temperature, humidity, pressure, illuminance, uva, uvb, uvIndex);
    displayGPSData(latitude, longitude, altitude, speed, satellites);
  }

  else {
    while (stepper.distanceToGo() > 0) {
      stepper.run();
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  }

  if (digitalRead(3) == LOW) { collectData = false; }
  else { collectData = true; }
}
