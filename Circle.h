/*
* Copyright (c) 2021 Karol Janic
*/

#ifndef CIRCLE_H
#define CIRCLE_H

// number of points using to render circle
#define circlePoints 25


// Circle class - unique shape
class Circle : public Shape
{
public:
    float radius;

    // costructor
    // _radius - length of radius of creating circle 
    Circle(float _radius)
    {
        radius = _radius;
    }

    Shape* Copy() const
    {
        return new Circle(radius);
    }

    void Calculate(float density)
    {
        body->mass = PI * radius * radius * density;
        body->inverseMass = 1.0 / body->mass;
        body->inertialMoment = 0.5 * body->mass * radius * radius;
        body->inverseInertialMoment = 1.0 / body->inertialMoment;
    }

    void SetOrientation(float radians)
    {
        // rotate of circle isn't relevant
    }

    void Draw() const
    {
        glColor3f(body->bodyColor.red, body->bodyColor.green, body->bodyColor.blue);
        glBegin(GL_POLYGON);
        float theta = body->orientation;
        float angle = PI * 2.0 / (float)circlePoints;
        Vector2D point;
        for (int i = 0; i < circlePoints; i++)
        {
            theta += angle;
            point.x = std::cos(theta);
            point.y = std::sin(theta);
            point *= radius;
            point += body->position;
            glVertex2f(point.x, point.y);
        }
        glEnd();
    }

    int GetType() const
    {
        return CircleID;
    }
};

#endif // CIRCLE_H