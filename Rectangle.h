/*
* Copyright (c) 2021 Karol Janic
*/

#ifndef RECTANGLE_H
#define RECTANGLE_H


// Rect class - unique poly ( shape ) = rectangle
class Rect : public Poly 
{
public:
    // constructor
    Rect(float width, float height)
    {
        vrtxCount = 4;
        vrtcsArray[0].x = -width;
        vrtcsArray[0].y = -height;
        vrtcsArray[1].x = width;
        vrtcsArray[1].y = -height;
        vrtcsArray[2].x = width;
        vrtcsArray[2].y = height;
        vrtcsArray[3].x = -width;
        vrtcsArray[3].y = height;
        nrmls[0].x = 0.0f;
        nrmls[0].y = -1.0f;
        nrmls[1].x = 1.0f;
        nrmls[1].y = 0.0f;
        nrmls[2].x = 0.0f;
        nrmls[2].y = 1.0f;
        nrmls[3].x = -1.0f;
        nrmls[3].y = 0.0f;
    }
};

#endif // RECTANGLE_H