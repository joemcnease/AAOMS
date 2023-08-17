/*

  Helper classes for representing the glider and glider rudder.

  Very simple implementation of orientation control using the onboard
  gps, 9dof (gyro, accel, mag), and stepper motor.

*/


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
  float elevation;
  float satallites;
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
}


class LED
{
  public:
    LED(int pin, )
}


class OrientationSensor
{
  uint8_t system;
  uint8_t gyroscope;
  uint8_t accelerometer;
  uint8_t magnetometer;

  public:
    OrientationSensor();

    bool isCalibrated();
}


class Rudder
{
  AccelStepper stepper(AccelStepper::DRIVER, 9, 8);

  int stepPos = 0;
  int stepperMaxSpeed = 2000;
  int stepperAccel = 1000;

  public:
    Rudder();

    setup(int maxSpeed, int accel)
    {

    }
}


class Glider
{
  Orientation orientation;
  Location location;
  Location destination;

  Rudder rudder;

  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

  public:
    Glider();

    bool setup();

    Orientation getOrientation();
    Location getLocation();
    Location getDestination();

    void setDestination();

    void displayOrientation();
    void displayLocation();
    void displayDestination();
};