//  This file is part of the Vector library.
//
//  The Vector library is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  The Vector library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with the Chronos library.  If not, see <http://www.gnu.org/licenses/>.
//
//  Copyright 2013 Kevin Balke (fughilli@gmail.com)

#ifndef VECTOR_X_H
#define VECTOR_X_H

#define VECTORS

#ifdef ARDUINO
#include "Energia.h"
#else
#include <cmath>
using namespace std;
#define PI 3.1415926535897932384626433832795
#define TWO_PI 6.283185307179586476925286766559
#endif

#define COMPARE_ACCURACY 0

#define LIMIT_RAD_RANGE(_RAD_) (((_RAD_)>PI)?((_RAD_)-TWO_PI):(((_RAD_)<(-PI))?((_RAD_)+TWO_PI):(_RAD_)))
#define LIMIT_DEG_RANGE(_DEG_) (((_DEG_)>180)?((_DEG_)-360):(((_DEG_)<(-180))?((_DEG_)+360):(_DEG_)))

/**
ALL ANGLES ARE IN RADIANS
**/

#define DEG2RAD(_DEG) (((_DEG) * 71 / 4068))
#define RAD2DEG(_RAD) (((_RAD) * 4068 / 71))

class Quaternion
{
    public:
    float x, y, z, w;

    Quaternion(float _x, float _y, float _z, float _w);
    Quaternion();

    Quaternion operator+(Quaternion other);
    void operator+=(Quaternion other);
    Quaternion operator-(Quaternion other);
    void operator-=(Quaternion other);
    Quaternion operator*(Quaternion other);
    void operator*=(Quaternion other);

    Quaternion operator*(float scalar);
    void operator*=(float scalar);
    Quaternion operator/(float scalar);
    void operator/=(float scalar);

    bool operator==(Quaternion other);

    Quaternion conjugate();
    float norm();
    Quaternion unit();
    float distance(Quaternion other);
    float dot(Quaternion other);

    Quaternion slerp(Quaternion endpt, float t);     // Perform a spherical linear interpolation between this quaternion and an end quaternion

    static const Quaternion zero;
    static const Quaternion identity;
    static const Quaternion i;
    static const Quaternion j;
    static const Quaternion k;
};

class Vector2d
{
public:
    float x, y;
    Vector2d(float _x, float _y);
    Vector2d();
    Vector2d operator+(Vector2d const &other) const;// Adds two Vector2d's together (component-wise)
    void operator+=(Vector2d const &other);			// Adds two Vector2d's together (component-wise)
    Vector2d operator-(Vector2d const &other) const;// Subtracts a Vector2d from another Vector2d (component-wise)
    void operator-=(Vector2d const &other);			// Subtracts a Vector2d from another Vector2d (component-wise)
    Vector2d operator*(float scalar) const;		// Scalar multiply
    void operator*=(float scalar);				// Scalar multiply
    Vector2d operator/(float scalar) const;		// Scalar divide
    void operator/=(float scalar);				// Scalar divide

    bool operator==(Vector2d other);            // Boolean compare

    float dot(Vector2d const &other) const;		// Compute the dot product of two Vector2d's
    Vector2d rotate(float theta) const;			// Rotates the vector around the origin by an angle theta

    Vector2d project(Vector2d axis);            // Project a vector onto another

    Vector2d lerp(Vector2d endpt, float t);     // Perform a linear interpolation between this position vector and an endpoint vector

    float angleTo(Vector2d const &other) const;	// Compute the angle between two Vector2d's
    float magnitude() const;					// Compute the magnitude of the Vector2d
    Vector2d unit() const;					    // Compute the normalized form of the Vector2d
    float theta() const;						// Compute the angle the Vector2d points along (polar theta)
    static const Vector2d zero;
    static const Vector2d i;
    static const Vector2d j;
    static const Vector2d one;
};

class Vector3d
{
public:
    float x, y, z;
    Vector3d(float _x, float _y, float _z);
    Vector3d();
    Vector3d operator+(Vector3d other);			// Adds two Vector3d's together (component-wise)
    void operator+=(Vector3d other);			// Adds two Vector3d's together (component-wise)
    Vector3d operator-(Vector3d other);			// Subtracts a Vector3d from another Vector3d (component-wise)
    void operator-=(Vector3d other);			// Subtracts a Vector3d from another Vector3d (component-wise)
    Vector3d operator*(float scalar);			// Scalar multiply
    void operator*=(float scalar);				// Scalar multiply
    Vector3d operator/(float scalar);			// Scalar divide
    void operator/=(float scalar);				// Scalar divide

    bool operator==(Vector3d other);            // Boolean compare

    float dot(Vector3d other);					// Compute dot product of two Vector3d's
    Vector3d cross(Vector3d other);				// Compute the cross product of two Vector3d's (orthogonal Vector3d)

    Vector3d project(Vector3d axis);            // Project a vector onto another

    Vector3d lerp(Vector3d endpt, float t);     // Perform a linear interpolation between this position vector and an endpoint vector

    Vector3d rotate(Vector3d axis, float theta);// Rotate the vector around the axis axis by the angle theta (+theta is CCW if vector points towards observer)
    Vector3d rotate(Quaternion rot_q);          // Rotate the vector by a quaternion

    float angleTo(Vector3d other);				// Compute the angle between two Vector3d's
    Quaternion quaternionTo(Vector3d other);    // Compute the quaternion rotation between two vectors

    Quaternion rotationAroundAxis(float theta); // compute the quaternion rotation of angle theta around a vector

    float magnitude();							// Compute the magnitude of the Vector3d
    Vector3d unit();							// Compute the normalized form of the Vector3d
    float theta();								// Compute the angle the (x,y) components of the Vector3d point along (spherical theta)
    float rho();								// Compute the angle of elevation of the Vector3d out of the (x,y) plane (spherical rho)
    static const Vector3d zero;
    static const Vector3d i;
    static const Vector3d j;
    static const Vector3d k;
    static const Vector3d one;
};

#ifdef ARDUINO
void printVector2d(Vector2d arg);
void printVector3d(Vector3d arg);
#endif

#endif
