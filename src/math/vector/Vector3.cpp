#include "Vector3.h"

Vector3::Vector3() : x(0), y(0), z(0) {}

Vector3::Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

float Vector3::length() const {
    return std::sqrt(x * x + y * y + z * z);
}

float Vector3::dot(const Vector3& other) const {
    return x * other.x + y * other.y + z * other.z;
}

Vector3 Vector3::cross(const Vector3& other) const {
    return Vector3(
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    );
}

Vector3& Vector3::operator+=(const Vector3& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

Vector3& Vector3::operator-=(const Vector3& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
}

Vector3& Vector3::operator*=(float scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

Vector3& Vector3::operator/=(float scalar) {
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
}

Vector3 operator+(Vector3 lhs, const Vector3& rhs) {
    lhs += rhs;
    return lhs;
}

Vector3 operator-(Vector3 lhs, const Vector3& rhs) {
    lhs -= rhs;
    return lhs;
}

Vector3 operator*(Vector3 vec, float scalar) {
    vec *= scalar;
    return vec;
}

Vector3 operator*(float scalar, Vector3 vec) {
    vec *= scalar;
    return vec;
}

Vector3 operator/(Vector3 vec, float scalar) {
    vec /= scalar;
    return vec;
}

std::ostream& operator<<(std::ostream& os, const Vector3& vec) {
    os << "Vector3(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
    return os;
}