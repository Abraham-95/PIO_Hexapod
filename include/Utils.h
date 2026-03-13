#pragma once

#include <Dynamixel2Arduino.h>
#include <vector>
using std::vector;

//=============Macros=============
#define every(interval)                                                            \
  static uint32_t __every__##interval = millis();                                  \
  if (millis() - __every__##interval >= interval && (__every__##interval = millis()))

//=============Vectors=============
struct Vector2int{
  int x;
  int y;

  Vector2int(int xVal, int yVal) : x(xVal), y(yVal) {}
  Vector2int() : x(0), y(0) {}
};

class Vector2{
public:
  float x;
  float y;
  // constructor
  Vector2();
  Vector2(float newX, float newY);

  String toString();
  float magnitude() const;

  // Function to normalize the vector
  void normalize();

  Vector2 operator+(Vector2 val);
  Vector2 operator*(float val);
  Vector2 operator*(Vector2 val);
  Vector2 rotate(int angle, Vector2 pivot);
};

class Vector3{
public:
  float x;
  float y;
  float z;
  // constructor
  Vector3();
  Vector3(float newX, float newY, float newZ);

  bool operator!=(Vector3 val);
  bool operator==(Vector3 val);

  Vector3 operator*(float val);
  Vector3 operator*(Vector3 val);
  Vector3 operator/(Vector3 val);
  Vector3 operator/(float val);
  Vector3 operator+(Vector3 val);

  String toString();
  Vector3 rotate(int angle, Vector2 pivot);
  float distanceTo(Vector3 v);
};

//=============Matrix 3x3=============//
struct Matrix3x3 {
  float m[3][3];

  Vector3 operator*(const Vector3& v) const {
    return {
      m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
      m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
      m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
    };
  }
};

Matrix3x3 makeRotationMatrix(float roll, float pitch, float yaw);
void printMatrix(const Matrix3x3& m);

//=============Bezier Curve=============
Vector2 GetPointOnBezierCurve(vector<Vector2>& controlPoints, float t);
Vector3 GetPointOnBezierCurve(const Vector3* controlPoints, int count, float t);
int binomialCoefficient(int n, int k);

//=============Helpers=============
float lerp(float a, float b, float f);
Vector2 lerp(Vector2 a, Vector2 b, float f);
Vector3 lerp(Vector3 a, Vector3 b, float f);

float calculateHypotenuse(float x, float y);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

void print_value(String name, float value, bool newLine);
void print_value(String name, String value, bool newLine);
void print_value(String name, Vector3 value, bool newLine);
