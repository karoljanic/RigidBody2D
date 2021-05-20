/*
* Copyright (c) 2021 Karol Janic
*/

#ifndef RECTANGLE_H
#define RECTANGLE_H


// Rect class - unique poly shape = rectangle
class Rect : public Poly 
{
public:
    // constructor
    // width - length of the first side 
    // height - length of the second side 
    Rect(float width, float height)
    {
        verticesCount = 4;
        verticesArray[0].x = -width;
        verticesArray[0].y = -height;
        verticesArray[1].x = width;
        verticesArray[1].y = -height;
        verticesArray[2].x = width;
        verticesArray[2].y = height;
        verticesArray[3].x = -width;
        verticesArray[3].y = height;
        normalVectors[0].x = 0.0;
        normalVectors[0].y = -1.0;
        normalVectors[1].x = 1.0;
        normalVectors[1].y = 0.0;
        normalVectors[2].x = 0.0;
        normalVectors[2].y = 1.0;
        normalVectors[3].x = -1.0;
        normalVectors[3].y = 0.0;
    }
};

#endif // RECTANGLE_H