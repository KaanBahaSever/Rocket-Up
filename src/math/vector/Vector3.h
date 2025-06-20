#ifndef VECTOR3_H
#define VECTOR3_H

#include <cmath>
#include <iostream>

class Vector3 {
public:
    float x, y, z;

    Vector3();
    Vector3(float x, float y, float z);

    float length() const;
    float dot(const Vector3& other) const;
    Vector3 cross(const Vector3& other) const;

    Vector3& operator+=(const Vector3& other);
    Vector3& operator-=(const Vector3& other);
    Vector3& operator*=(float scalar);
    Vector3& operator/=(float scalar);

    friend Vector3 operator+(Vector3 lhs, const Vector3& rhs);
    friend Vector3 operator-(Vector3 lhs, const Vector3& rhs);
    friend Vector3 operator*(Vector3 vec, float scalar);
    friend Vector3 operator*(float scalar, Vector3 vec);
    friend Vector3 operator/(Vector3 vec, float scalar);

    friend std::ostream& operator<<(std::ostream& os, const Vector3& vec);
};

#endif // VECTOR3_H