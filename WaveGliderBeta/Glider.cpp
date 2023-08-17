#include "Arduino.h"
#include "Glider.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <AccelStepper.h>
#include <Arduino_MKRENV.h>
#include <Arduino_MKRGPS.h>
#include "Utils.h"
#include <cmath>


LED::LED(pin_size_t pin, PinMode pMode, PinStatus pStatus)
: _pin(pin)
{
  pinMode(pin, pMode);
  digitalWrite(pin, pStatus);
}

void LED::on()
{
  digitalWrite(_pin, HIGH);
}

void LED::off()
{
  digitalWrite(_pin, LOW);
}


Button::Button(pin_size_t pin, PinMode pMode, PinStatus pPressStatus)
: _pin(pin), _pinPressStatus(pPressStatus)
{
  pinMode(_pin, INPUT);
}

bool Button::isPressed()
{
  if (digitalRead(_pin) == _pinPressStatus)
  {
    return true;
  }
  return false;
}


EnvironmentSensor::EnvironmentSensor()
{

}

void EnvironmentSensor::setup()
{
  if (!ENV.begin())
  {
    Serial.println("Failed to initialize MKR ENV Shield!");
    // TODO: should instead exit gracefully
    while(1);
  }
}

void EnvironmentSensor::dislayEnvironment()
{
  Serial.print("TEMPERATURE: ");
  Serial.print(_environment.temperature, 4);
  Serial.print(", HUMIDITY: ");
  Serial.print(_environment.humidity, 4);
  Serial.print(", PRESSURE: ");
  Serial.print(_environment.pressure, 4);
  Serial.print(", ILLUMINANCE: ");
  Serial.print(_environment.illuminance, 4);
  Serial.print(", UVA: ");
  Serial.print(_environment.uva, 4);
  Serial.print(", UVB: ");
  Serial.print(_environment.uvb, 4);
  Serial.print(", UVINDEX: ");
  Serial.print(_environment.uvindex, 4);
  Serial.println("");
}

void EnvironmentSensor::readEnvironment()
{
  _environment = {
    ENV.readTemperature(),
    ENV.readHumidity(),
    ENV.readPressure(),
    ENV.readIlluminance(),
    ENV.readUVA(),
    ENV.readUVB(),
    ENV.readUVIndex()
  };
}


GPSSensor::GPSSensor()
{

}

void GPSSensor::setup()
{
  if (!GPS.begin())
  {
    Serial.println("Failed to initialize the GPS!");
    // TODO: should instead exit gracefully
    while(1);
  }
}

void GPSSensor::displayGPS()
{
  Serial.print("LATITUDE: ");
  Serial.print(_location.latitude, 7);
  Serial.print(", LONGITUDE: ");
  Serial.print(_location.longitude, 7);
  Serial.print(", ALTITUDE: ");
  Serial.print(_location.altitude);
  Serial.print(", SPEED: ");
  Serial.print(_location.speed);
  Serial.print(", # SATELLITES: ");
  Serial.print(_location.satallites);
  Serial.println("");
}

void GPSSensor::readGPS()
{
  if (GPS.available())
  {
    _location = {
      GPS.latitude(),
      GPS.longitude(),
      GPS.altitude(),
      GPS.speed(),
      GPS.satellites()
    };
  }
}


OrientationSensor::OrientationSensor()
{

}

void OrientationSensor::setup()
{
  if (!_bno.begin())
  {
    Serial.println("No BNO055 detected. Check wiring and/or I2C address (0x28 or 0x29");
    // TODO: should instead exit gracefully
    while(1);
  }

  _bno.setExtCrystalUse(true);
}

void OrientationSensor::displayDetails()
{
  _bno.getSensor(&_sensor);

  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(_sensor.name);
  Serial.print("Driver Version:   ");
  Serial.println(_sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(_sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(_sensor.max_value);
  Serial.println(" xxx");
  Serial.print("Min Value:    ");
  Serial.print(_sensor.min_value);
  Serial.println(" xxx");
  Serial.print("Resolution:   ");
  Serial.print(_sensor.resolution);
  Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
}

void OrientationSensor::displayStatus()
{
  _system_status = _self_test_results = _system_error = 0;
  _bno.getSystemStatus(&_system_status, &_self_test_results, &_system_error);

  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(_system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(_self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(_system_error, HEX);
  Serial.println("");
}

void OrientationSensor::displayOrientation()
{
  Serial.print("HEADING: ");
  Serial.print(_orientation.heading, 4);
  Serial.print(", PITCH: ");
  Serial.print(_orientation.pitch, 4);
  Serial.print(", ROLL: ");
  Serial.print(_orientation.roll, 4);
  Serial.print("  ||  ");
  if (!_system)
  {
    Serial.print("!");
  }
  Serial.print("SYSTEM: ");
  Serial.print(_system, DEC);
  Serial.print(", GYROSCOPE: ");
  Serial.print(_gyroscope, DEC);
  Serial.print(", ACCELEROMETER: ");
  Serial.print(_accelerometer, DEC);
  Serial.print(", MAGNETOMETER: ");
  Serial.print(_magnetometer, DEC);
  Serial.println("");
}

void OrientationSensor::readOrientation()
{
  _bno.getEvent(&_event);
  _orientation = {
    _event.orientation.x,
    _event.orientation.y,
    _event.orientation.z
  };
}

void OrientationSensor::readCalibration()
{
  _bno.getCalibration(&_system, &_gyroscope, &_accelerometer, &_magnetometer);
}

bool OrientationSensor::isCalibrated()
{
  if (_system > 0 && _gyroscope > 0 && _accelerometer > 0 && _magnetometer > 0)
  {
    return true;
  }
  return false;
}

Orientation OrientationSensor::getOrientation()
{
  return _orientation;
}


Rudder::Rudder()
{

}

void Rudder::setup()
{
  _stepper = AccelStepper(AccelStepper::DRIVER, 9, 8);
  _stepper.setMaxSpeed(_maxSpeed);
  _stepper.setAcceleration(_acceleration);
  _stepper.moveTo(0);
}

void Rudder::moveTo(float degree)
{
  Serial.print("Moving to");
  Serial.println(degree);
  _stepper.moveTo((degree/360)*_stepsPerRevolution);
  while (_stepper.distanceToGo() != 0)
  {
    _stepper.run();
  }
}


Glider::Glider()
{
  _rudder.setup();
}

void Glider::setDestination(Location destination)
{
  _destination = destination;
}

void Glider::setDestination(float lat, float lon)
{
  _destination = {
    lat,
    lon,
    0,
    0,
    0
  };
}

void Glider::setOrientation(Orientation orientation)
{
  _orientation = orientation;
}

void Glider::setFollow(Orientation follow)
{
  _follow = follow;
}

void Glider::setFollow(float heading)
{
  _follow = {
    heading, 
    0,
    0
  };
}

Orientation Glider::getFollow()
{
  return _follow;
}

void Glider::updateRudder()
{
  // Move rudder to move glider towards follow heading
  // float degree = smallestSignedAngleBetween( _follow.heading, _orientation.heading)
  float y = std::sin(degreeToRadian(_follow.heading) - degreeToRadian(_orientation.heading));
  float x = std::cos(degreeToRadian(_follow.heading) - degreeToRadian(_orientation.heading));
  float degree = radianToDegree(std::atan2(y, x));
  _rudder.moveTo(degree);
}