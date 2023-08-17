#include "Utils.h"


float customMod(float a, float n)
{
  return a - std::floor(a/n)*n;
  // return ((int)a % (int)n + (int)n) % (int)n;
}


float smallestSignedAngleBetween(float x, float y)
{
  float a = x - y;
  return customMod(a + 180, 360) - 180;
  // return (float)(((int)a + 180) % 360 - 180);
}


float degreeToRadian(float degree)
{
  return (degree/180)*M_PI;
}


float radianToDegree(float radian)
{
  return (radian/M_PI)*180;
}
