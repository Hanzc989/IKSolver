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

#include "Vector.h"

Quaternion::Quaternion(float _x, float _y, float _z, float _w)
{
    this->x = _x;
    this->y = _y;
    this->z = _z;
    this->w = _w;
}

Quaternion::Quaternion()
{
    Quaternion(0,0,0,0);
}

Quaternion Quaternion::operator+(Quaternion other)
{
    Quaternion ret(other.x + this->x, other.y + this->y, other.z + this->z, other.w + this->w);
    return ret;
}

void Quaternion::operator+=(Quaternion other)
{
    this->x += other.x;
    this->y += other.y;
    this->z += other.z;
    this->w += other.w;
}

Quaternion Quaternion::operator-(Quaternion other)
{
    Quaternion ret(other.x - this->x, other.y - this->y, other.z - this->z, other.w - this->w);
    return ret;
}

void Quaternion::operator-=(Quaternion other)
{
    this->x -= other.x;
    this->y -= other.y;
    this->z -= other.z;
    this->w -= other.w;
}

Quaternion Quaternion::operator*(Quaternion other)
{
    Quaternion ret(
        this->w*other.x + this->x*other.w + this->y*other.z - this->z*other.y,
        this->w*other.y - this->x*other.z + this->y*other.w + this->z*other.x,
        this->w*other.z + this->x*other.y - this->y*other.x + this->z*other.w,
        this->w*other.w - this->x*other.x - this->y*other.y - this->z*other.z);
    return ret;
}

void Quaternion::operator*=(Quaternion other)
{
    this->x = this->w*other.x + this->x*other.w + this->y*other.z - this->z*other.y;
    this->y = this->w*other.y - this->x*other.z + this->y*other.w + this->z*other.x;
    this->z = this->w*other.z + this->x*other.y - this->y*other.x + this->z*other.w;
    this->w = this->w*other.w - this->x*other.x - this->y*other.y - this->z*other.z;
}

Quaternion Quaternion::operator*(float scalar)
{
    Quaternion ret(
        this->x * scalar,
        this->y * scalar,
        this->z * scalar,
        this->w * scalar
    );
    return ret;
}

void Quaternion::operator*=(float scalar)
{
    this->x *= scalar;
    this->y *= scalar;
    this->z *= scalar;
    this->w *= scalar;
}

Quaternion Quaternion::operator/(float scalar)
{
    Quaternion ret(
        this->x / scalar,
        this->y / scalar,
        this->z / scalar,
        this->w / scalar
    );
    return ret;
}

void Quaternion::operator/=(float scalar)
{
    this->x /= scalar;
    this->y /= scalar;
    this->z /= scalar;
    this->w /= scalar;
}

bool Quaternion::operator==(Quaternion other)
{
#if(COMPARE_ACCURACY > 0)
    return ((this->x==other.x)&&(this->y==other.y)&&(this->z==other.z)&&(this->w==other.w));
#else
    return ((abs(this->x-other.x)<=COMPARE_ACCURACY)&&(abs(this->y-other.y)<=COMPARE_ACCURACY)&&(abs(this->z-other.z)<=COMPARE_ACCURACY)&&(abs(this->w-other.w)<=COMPARE_ACCURACY));
#endif
}

Quaternion Quaternion::conjugate()
{
    return Quaternion(-x, -y, -z, w);
}

float Quaternion::norm()
{
    return sqrt(x*x + y*y + z*z + w*w);
}

Quaternion Quaternion::unit()
{
    return (*this)/norm();
}

float Quaternion::distance(Quaternion other)
{
    return ((*this) - other).norm();
}

float Quaternion::dot(Quaternion other)
{
    return (x*other.x + y*other.y + z*other.z + w*other.w);
}

Quaternion Quaternion::slerp(Quaternion endpt, float t)
{
    Quaternion qret;
    float cosHalfTheta = dot(endpt);

    if(abs(cosHalfTheta) >= 1.0f)
    {
        qret.x = x;
        qret.y = y;
        qret.z = z;
        qret.w = w;
        return qret;
    }

    float halfTheta = acos(cosHalfTheta);
    float sinHalfTheta = sqrt(1.0f - cosHalfTheta*cosHalfTheta);

    if(abs(sinHalfTheta) < 0.001f)
    {
        qret.x = (x*0.5f + endpt.x*0.5f);
        qret.y = (y*0.5f + endpt.y*0.5f);
        qret.z = (z*0.5f + endpt.z*0.5f);
        qret.w = (w*0.5f + endpt.w*0.5f);
        return qret;
    }

    float ratA = sin((1 - t) * halfTheta) / sinHalfTheta;
    float ratB = sin(t * halfTheta) / sinHalfTheta;

    qret.x = (x*ratA + endpt.x*ratB);
    qret.y = (y*ratA + endpt.y*ratB);
    qret.z = (z*ratA + endpt.z*ratB);
    qret.w = (w*ratA + endpt.w*ratB);
    return qret;
}

const Quaternion Quaternion::zero = Quaternion(0,0,0,0);
const Quaternion Quaternion::identity = Quaternion(0,0,0,1);
const Quaternion Quaternion::i = Quaternion(1,0,0,0);
const Quaternion Quaternion::j = Quaternion(0,1,0,0);
const Quaternion Quaternion::k = Quaternion(0,0,1,0);
