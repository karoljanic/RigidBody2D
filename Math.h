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
const float PI = 3.141592741;
const float EPSILON = 0.0001;


// Vector2D math class
class Vector2D
{
public:
    float x;
    float y;

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

    // addition operator
    Vector2D operator+(const Vector2D& vector2) const
    {
        return Vector2D(x + vector2.x, y + vector2.y);
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
    void operator-=(const Vector2D& vector2)
    {
        x -= vector2.x;
        y -= vector2.y;
    }

    // scalar multiplication operator
    Vector2D operator*(float scalar) const
    {
        return Vector2D(x * scalar, y * scalar);
    }

    // shortened scalar multiply operator 
    void operator*=(float scalar)
    {
        x *= scalar;
        y *= scalar;
    }

    // scalar multiplication operator ( vector1 dot vector2 )
    Vector2D operator*(const Vector2D& vector2) const
    {
        return Vector2D(x + vector2.x, y + vector2.y);
    }

    // scalar multiplication operator ( vector1 dot vector2 )
    Vector2D operator*=(const Vector2D& vector2) const
    {
        return Vector2D(x + vector2.x, y + vector2.y);
    }

    // scalar division operator
    Vector2D operator/(float scalar) const
    {
        return Vector2D(x / scalar, y / scalar);
    }

    // shortened scalar division operator
    void operator/=(float scalar)
    {
        x /= scalar;
        y /= scalar;
    }

    // returns (vector length)^2
    float lengthPower2() const
    {
        return x * x + y * y;
    }

    // returns vector length
    float length() const
    {
        return std::sqrt(x * x + y * y);
    }

    // vector rotate operation by angle
    void rotate(float angle)
    {
        float cosinus = std::cos(angle);
        float sinus = std::sin(angle);

        x = x * cosinus - y * sinus;
        y = x * sinus + y * cosinus;
    }

    // vector normalisation
    void normalize()
    {
        float len = length();

        if (len > EPSILON)
        {
            float inverseLen = 1.0 / len;
            x *= inverseLen;
            y *= inverseLen;
        }
    }
};

// reversed scalar multiplication operator 
inline Vector2D operator*(float scalar, const Vector2D& vector)
{
    return Vector2D(scalar * vector.x, scalar * vector.y);
}


// Matrix2X2 math class
class Matrix2X2
{
public:
    float matrix[2][2];
  
    // constructor v1
    Matrix2X2()
    {
    }

    // constructor v2
    //   | 00 | 01 |
    //   | 10 | 11 |
    Matrix2X2(float matrix00, float matrix01, float matrix10, float matrix11)
    {
        matrix[0][0] = matrix00;
        matrix[0][1] = matrix01;
        matrix[1][0] = matrix10;
        matrix[1][1] = matrix11;
    }

    // constructor v3 - rotation matrix 
    Matrix2X2(float radians)
    {
        float cosinus = std::cos(radians);
        float sinus = std::sin(radians);

        matrix[0][0] = cosinus;
        matrix[0][1] = -sinus;
        matrix[1][0] = sinus;
        matrix[1][1] = cosinus;
    }

    // return Matrix2x2 with absolute values
    Matrix2X2 abs() const
    {
        return Matrix2X2(std::abs(matrix[0][0]), std::abs(matrix[0][1]), 
                         std::abs(matrix[1][0]), std::abs(matrix[1][1]));
    }

    // returns the transposed matrix
    Matrix2X2 transpose() const
    {
        return Matrix2X2(matrix[0][0], matrix[1][0],
                         matrix[0][1], matrix[1][1]);
    }

    // multiplication operation: vector2D * matrix2X2
    const Vector2D operator*(const Vector2D& vector) const
    {
        return Vector2D(matrix[0][0] * vector.x + matrix[0][1] * vector.y, 
                        matrix[1][0] * vector.x + matrix[1][1] * vector.y);
    }

    // multiplication operation: matrix2X2 * matrix2X2
    const Matrix2X2 operator*(const Matrix2X2& matrix2) const
    {
        return Matrix2X2(
            matrix[0][0] * matrix2.matrix[0][0] + matrix[0][1] * matrix2.matrix[1][0],
            matrix[0][0] * matrix2.matrix[0][1] + matrix[0][1] * matrix2.matrix[1][1],
            matrix[1][0] * matrix2.matrix[0][0] + matrix[1][1] * matrix2.matrix[1][0],
            matrix[1][0] * matrix2.matrix[0][1] + matrix[1][1] * matrix2.matrix[1][1] );
    }
};

// return vector1 dot vector2
inline float dot(const Vector2D& vector1, const Vector2D& vector2)
{
    return vector1.x * vector2.x + vector1.y * vector2.y;
}

// returns vector cross a
inline Vector2D cross(const Vector2D& vector, float a)
{
    return Vector2D(a * vector.y, -a * vector.x);
}

// returns a cross vector
inline Vector2D cross(float a, const Vector2D& vector)
{
    return Vector2D(-a * vector.y, a * vector.x);
}

// returns vector1 cross vector2
inline float cross(const Vector2D& vector1, const Vector2D& vector2)
{
    return vector1.x * vector2.y - vector1.y * vector2.x;
}

// returns wheter numbers are equal with EPSILON tolerance
inline bool equal(float number1, float number2)
{
    return std::abs(number1 - number2) <= EPSILON;
}

// returns random number between <_min, _max>
inline float random(float _min, float _max)
{
    float number = (float)rand();
    number /= RAND_MAX;
    number = (_max - _min) * number + _min;
    return number;
}

#endif // MATH_H