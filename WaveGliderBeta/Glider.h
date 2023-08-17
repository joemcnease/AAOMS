#ifndef Glider_h
#define Glider_h

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <AccelStepper.h>
#include <Arduino_MKRENV.h>
#include <Arduino_MKRGPS.h>


#define BNO055_SAMPLERATE_DELAY_MS (100)


struct Orientation
{
  float heading;
  float pitch;
  float roll;
};


struct Location
{
  float latitude;
  float longitude;
  float altitude;
  float speed;
  int satallites;
};


struct Environment
{
  float temperature;
  float humidity;
  float pressure;
  float illuminance;
  float uva;
  float uvb;
  float uvindex;
};


class LED
{
  private:
    pin_size_t _pin;

  public:
    LED(pin_size_t pin, PinMode pinMode, PinStatus pinStatus);

    void on();
    void off();
};


class Button
{
  private:
    pin_size_t _pin;
    PinStatus _pinPressStatus;

  public:
    Button(pin_size_t pin, PinMode pinMode, PinStatus pPressStatus);

    bool isPressed();
};


class EnvironmentSensor
{
  private:
    Environment _environment;

  public:
    EnvironmentSensor();

    void setup();
    void dislayEnvironment();
    void readEnvironment();
};


class GPSSensor
{
  private:
    Location _location;

  public:
    GPSSensor();

    void setup();
    void displayGPS();
    void readGPS();
};


class OrientationSensor
{
  private:
    Adafruit_BNO055 _bno = Adafruit_BNO055(55, 0x28);
    sensor_t _sensor;
    sensors_event_t _event;
    uint8_t _system_status, _self_test_results, _system_error;
    uint8_t _system, _gyroscope, _accelerometer, _magnetometer;
    Orientation _orientation;
  
  public:
    OrientationSensor();

    void setup();
    void displayDetails();
    void displayStatus();
    void displayOrientation();
    void readOrientation();
    void readCalibration();
    bool isCalibrated();
    Orientation getOrientation();
};


class Rudder
{
  private:
    int _stepPosition = 0;
    int _stepsPerRevolution = 1600;
    int _maxStepAttempts = 5000; // A few seconds
    int _maxSpeed = 2000;
    int _acceleration = 1000;
    AccelStepper _stepper;

  public:
    Rudder();

    void setup();
    void moveTo(float degree);
};


class Glider
{
  private:
    Orientation _orientation;
    Orientation _follow;
    Location _location;
    Location _destination;
    Rudder _rudder;

  public:
    Glider();

    void setDestination(Location destination);
    void setDestination(float lat, float lon);
    void setOrientation(Orientation orientation);
    void setFollow(Orientation follow);
    void setFollow(float heading);
    Orientation getFollow();
    void updateRudder();
};


#endif