/*
* Copyright (c) 2021 Karol Janic
*/

#ifndef MATH_H
#define MATH_H

// include basic C++ functions
#include <algorithm> 
#include <cmath> 
#include <cassert> 

// math constants define
const float PI = 3.141592741f;
const float EPSILON = 0.0001f;


// Vector2D math class
class Vector2D
{
public:
    union
    {
        float m[1][1];
        float v[2];

        struct
        {
            float x;
            float y;
        };
    };

    // constructor v1
    Vector2D()
    {
    }

    // constructor v2
    Vector2D(float _x, float _y)
    {
        x = _x;
        y = _y;
    }

    // opposite operator
    Vector2D operator-() const
    {
        return Vector2D(-x, -y);
    }

    // scalar multiply operator
    Vector2D operator*(float s) const
    {
        return Vector2D(x * s, y * s);
    }

    // scalar division operator
    Vector2D operator/(float s) const
    {
        return Vector2D(x / s, y / s);
    }

    // shortened scalar multiply operator 
    void operator*=(float s)
    {
        x *= s;
        y *= s;
    }

    // multiplication operator
    Vector2D operator+(const Vector2D& vector2) const
    {
        return Vector2D(x + vector2.x, y + vector2.y);
    }

    // scalar multiplication operator
    Vector2D operator+(float s) const 
    {
        return Vector2D(x + s, y + s);
    }

    // shortened addition operator
    void operator+=(const Vector2D& vector2)
    {
        x += vector2.x;
        y += vector2.y;
    }

    // substraction operator
    Vector2D operator-(const Vector2D& vector2) const
    {
        return Vector2D(x - vector2.x, y - vector2.y);
    }

    // shortened substraction operator
    void operator-=(const Vector2D& rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
    }

    // returns (vector length)^2
    float LengthPower2() const
    {
        return x * x + y * y;
    }

    // returns vector length
    float Length() const
    {
        return std::sqrt(x * x + y * y);
    }

    // vector rotate operation by angle
    void Rotate(float angle)
    {
        float c = std::cos(angle);
        float s = std::sin(angle);

        float xp = x * c - y * s;
        float yp = x * s + y * c;

        x = xp;
        y = yp;
    }

    // vector normalisation
    void Normalize()
    {
        float len = Length();

        if (len > EPSILON)
        {
            float invLen = 1.0f / len;
            x *= invLen;
            y *= invLen;
        }
    }
};

// reversed scalar multiplication operator 
inline Vector2D operator*(float s, const Vector2D& vector)
{
    return Vector2D(s * vector.x, s * vector.y);
}


// Matrix2X2 math class
class Matrix2X2
{
public:
    union
    {
        struct
        {
            float m00, m01;
            float m10, m11;
        };

        float m[2][2];
        float v[4];
    };

    // constructor v1
    Matrix2X2()
    {
    }

    // constructor v2
    Matrix2X2(float a, float b, float c, float d)
    {
        m00 = a;
        m01 = b;
        m10 = c;
        m11 = d;
    }

    // constructor v3 - rotation matrix 
    Matrix2X2(float radians)
    {
        float c = std::cos(radians);
        float s = std::sin(radians);

        m00 = c; m01 = -s;
        m10 = s; m11 = c;
    }

    // return Matrix2x2 with absolute values
    Matrix2X2 Abs() const
    {
        return Matrix2X2(std::abs(m00), std::abs(m01), std::abs(m10), std::abs(m11));
    }

    // returns the transposed matrix
    Matrix2X2 Transpose() const
    {
        return Matrix2X2(m00, m10, m01, m11);
    }

    // multiplication operation: matrix2X2 * vector2D
    const Vector2D operator*(const Vector2D& vector) const
    {
        return Vector2D(m00 * vector.x + m01 * vector.y, m10 * vector.x + m11 * vector.y);
    }

    // multiplication operation: matrix2X2 * matrix2X2
    const Matrix2X2 operator*(const Matrix2X2& matrix2) const
    {
        return Matrix2X2(
            m[0][0] * matrix2.m[0][0] + m[0][1] * matrix2.m[1][0],
            m[0][0] * matrix2.m[0][1] + m[0][1] * matrix2.m[1][1],
            m[1][0] * matrix2.m[0][0] + m[1][1] * matrix2.m[1][0],
            m[1][0] * matrix2.m[0][1] + m[1][1] * matrix2.m[1][1] );
    }
};

// return vector1 dot vector2
inline float Dot(const Vector2D& vector1, const Vector2D& vector2)
{
    return vector1.x * vector2.x + vector1.y * vector2.y;
}

// returns vector cross a
inline Vector2D Cross(const Vector2D& vector, float a)
{
    return Vector2D(a * vector.y, -a * vector.x);
}

// returns a cross vector
inline Vector2D Cross(float a, const Vector2D& vector)
{
    return Vector2D(-a * vector.y, a * vector.x);
}

// returns vector1 cross vector2
inline float Cross(const Vector2D& vector1, const Vector2D& vector2)
{
    return vector1.x * vector2.y - vector1.y * vector2.x;
}

// returns wheter numbers are equal with EPSILON tolerance
inline bool Equal(float number1, float number2)
{
    return std::abs(number1 - number2) <= EPSILON;
}

// returns number clamp
inline float Clamp(float min, float max, float number)
{
    if (number < min) return min;
    if (number > max) return max;
    return number;
}

// return rounded number as int
inline int Round(float number)
{
    return (int)(number + 0.5f);
}

// returns random number between <_min, _max>
inline float Random(float _min, float _max)
{
    float number = (float)rand();
    number /= RAND_MAX;
    number = (_max - _min) * number + _min;
    return number;
}

// return wheter bias is grater then number
inline bool BiasGreaterThan(float a, float b)
{
    const float k_biasRelative = 0.95f;
    const float k_biasAbsolute = 0.01f;
    return a >= b * k_biasRelative + a * k_biasAbsolute;
}

#endif // MATH_H