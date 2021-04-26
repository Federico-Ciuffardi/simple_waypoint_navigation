#include "Vector3.h"

Vector3::Vector3() {
  this->x = 0;
  this->y = 0;
  this->z = 0;
}

Vector3::Vector3(float x, float y, float z) {
  this->x = x;
  this->y = y;
  this->z = z;
}

Vector3 Vector3::operator+(const Vector3 v) const {
  return Vector3(x + v.x, y + v.y, z + v.z);
}

Vector3& Vector3::operator+=(const Vector3 v) {
  *this = (*this + v);
  return *this;
}

Vector3 Vector3::operator-(const Vector3 v) const {
  return Vector3(x - v.x, y - v.y, z - v.z);
}

Vector3& Vector3::operator-=(const Vector3 v) {
  *this = (*this - v);
  return *this;
}

Vector3 Vector3::operator*(const float f) const {
  return Vector3(f*x, f*y, f*z);
}
Vector3 operator*(float f, const Vector3 v) {
  return Vector3(f*v.x, f*v.y, f*v.z);
}

Vector3 Vector3::operator*(const Vector3 v) const {
  return Vector3(y*v.z - z*v.y, x*v.z - z*v.x, x*v.y - y*v.x);
}

float Vector3::dot(const Vector3 v) const {
  return x * v.x + y * v.y + z * v.z;
}

float Vector3::angle_to(const Vector3 v) const{
  return acos(this->dot(v)/(this->length() * v.length()));
}

Vector3 Vector3::operator/(const float f) const {
  return Vector3(x/f, y/f, z/f);
}

Vector3 Vector3::operator-() const{
  return Vector3() - *this;
}

float Vector3::length() const{
  return sqrt(this->dot(*this));
}

float Vector3::length_squared() const{
  return this->dot(*this);
}

Vector3 Vector3::normalize() const{
  return *this/this->length();
}

Vector3 Vector3::symmetric(Vector3 v) const{
  Vector3 n = v.normalize();
  return 2.0 * n * ( this->dot(n) ) - *this;
}

float Vector3::distance_to(Vector3 v) const{
  return ((*this) - v).length();
}

bool Vector3::operator==(const Vector3 v) const {
  return (x == v.x) && (y == v.y);
}

bool Vector3::operator<(const Vector3 v) const {
  return x < v.x || (x == v.x && y < v.y);
}

ostream& operator<<(ostream& out, const Vector3 v) {
  out << "(" << v.x << "," << v.y << "," << v.z << ")";
  return out;
}
