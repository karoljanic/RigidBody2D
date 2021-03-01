/*
* Copyright (c) 2021 Karol Janic
*/

#ifndef SHAPE_H
#define SHAPE_H

#include "RigidBody.h"
#include "IncludesManager.h"


// virtual class Shape is a base for every geometric shape
class Shape
{
public:
    // every shape has own indentyficator
    enum ID
    {
        CircleID,   // 0
        PolygonID,  // 1
        CountID,    // 2
    };

    RigidBody* body;

    float radius;   // only for circle
    Matrix2X2 orient;    // only for polygon

    // constructor
    Shape() {}

    // virtual method to create the indetic shape
    virtual Shape* Copy() const = 0;

    // virtual method to calculate mass and moment of inertial using body density
    // density - density of body which mass and inertial moment is calculating
    virtual void Calculate(float density) = 0;

    // virtual method to set shape orientation
    // radians - rotation angle value to set
    virtual void SetOrientation(float radians) = 0;

    // virtual method to draw shape
    virtual void Draw() const = 0;

    // virtual method to get shape indentyficator
    virtual int GetType() const = 0; 
};

#endif // SHAPE_H