#pragma once

#include <ostream>
#include <cmath>

using namespace std;

struct Vector3 {
  float x, y, z;

  Vector3();
  Vector3(float x, float y, float z);

  Vector3 operator+(const Vector3) const;
  Vector3& operator+=(const Vector3);

  Vector3 operator-(const Vector3) const;
  Vector3& operator-=(const Vector3);

  Vector3 operator*(const float) const;

  Vector3 operator*(const Vector3) const; // cross-product

  float dot(const Vector3) const;

  float angle_to(const Vector3) const;

  Vector3 operator/(const float) const;

  Vector3 operator-() const;

  float length() const;

  float length_squared() const;

  Vector3 normalize() const;

  Vector3 symmetric(Vector3) const;

  float distance_to(Vector3) const;

  bool operator==(const Vector3) const;
  bool operator<(const Vector3) const;

  friend ostream& operator<<(ostream& out, const Vector3);
};

ostream& operator<<(ostream& out, const Vector3);

Vector3 operator*(float f, const Vector3);
