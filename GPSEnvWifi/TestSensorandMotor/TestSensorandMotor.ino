#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <AccelStepper.h>


/*
 * Orientation data collecting and heading control with stepper motor.
 *
 * BNO055 takes up pins: 11 (SCL), 12 (SDA)
 * Easy Stepper Driver takes up pins: 8 (SCK), 9 (MOSI)
*/

// Sample rate delay time
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address 0x28 (could also be 0x29)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Define stepper and pins to use
AccelStepper stepper(AccelStepper::DRIVER, 9, 8);

// Variables for heading control
int pos = 3200;
int pos1 = 3200;
int pos2 = 0;
int stepping = false;


void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
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


void displayCalStatus(void)
{
  /* Get and print calibrations data.
   *  
   * Get calibrations readings for system, gyro, accel, mag
   * 0 readings should be ignored, 3 means 'fully calibrated'
  */
  
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}


void setup(void)
{
  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test");
  Serial.println("");

  pinMode(2, INPUT);

  // Initialize sensors (gyro, accel, mag)
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);

  displaySensorDetails();
  displaySensorStatus();
  bno.setExtCrystalUse(true);

  // Stepper setup
  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(1000);
}


void loop(void)
{
  // Update sensor event (new instrument reading)
  //sensors_event_t event;
  //bno.getEvent(&event);

  // x = heading, y = pitch, z = roll

  // Display orientation data
//  Serial.print("X: ");
//  Serial.print(event.orientation.x, 4);
//  Serial.print("\tY: ");
//  Serial.print(event.orientation.y, 4);
//  Serial.print("\tZ: ");
//  Serial.print(event.orientation.z, 4);
//
//  displayCalStatus();
  // displaySensorStatus();
//  Serial.println("");

  Serial.print("digitalRead(2) = ");
  Serial.println(digitalRead(2));
  // Only run on button press
  if (digitalRead(2) == HIGH)
  {
    // Heading updates
    if (stepper.distanceToGo() == 0)
    {
      delay(500);
      if (pos == pos1) { pos = pos2; }
      else { pos = pos1; }
      stepper.moveTo(pos);
    }
    stepper.run();
  }
  
  //delay(BNO055_SAMPLERATE_DELAY_MS);
}
