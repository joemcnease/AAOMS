#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <AccelStepper.h>
#include <Arduino_MKRENV.h>
#include <Arduino_MKRGPS.h>
#include "Glider.h"


// All singleton instances of glider related objects
EnvironmentSensor env;
GPSSensor gps;
OrientationSensor ori;
Glider glider;

// LEDs
LED green = LED(4, OUTPUT, LOW);
LED red = LED(5, OUTPUT, LOW);

// Button
Button button = Button(3, INPUT, LOW);

// Keeping track of time
unsigned long timer = 0;

// Temporary
const unsigned long updateHeadingTime = 2.;

void setup()
{
  Serial.begin(9600);
  // Wait for serial monitor to open
  // while (!Serial)
  // {
  //   ;
  // }

  // Initialize sensors
  env.setup();
  gps.setup();
  ori.setup();

  // Show sensor details (possible errors, precision, etc.)
  ori.displayDetails();
  ori.displayStatus();

  timer = millis();
}

void loop()
{
  env.readEnvironment();
  env.dislayEnvironment();

  gps.readGPS();
  gps.displayGPS();

  ori.readOrientation();
  ori.readCalibration();
  ori.displayOrientation();
  if (ori.isCalibrated())
  {
    glider.setOrientation(ori.getOrientation());
    red.off();
    green.on();
  }
  else
  {
    green.off();
    red.on();
  }

  Serial.println("");

  // Look for button press to signal new orientation save
  if (button.isPressed())
  {
    Serial.println("");
    Serial.println("");
    Serial.print("Changing follow heading to ");
    Serial.print(ori.getOrientation().heading);
    Serial.println("");
    Serial.println("");
    glider.setFollow(ori.getOrientation());
  }
  
  if ((millis() - timer)/1000 > updateHeadingTime)
  {
   Serial.println("Updating rudder position.");
   glider.updateRudder();

   timer = millis();
  }
  Serial.println("");
  Serial.print("Follow heading:");
  Serial.println(glider.getFollow().heading);
  
  delay(1000);
}
