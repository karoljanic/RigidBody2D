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
    Circle(float r)
    {
        radius = r;
    }

    Shape* Copy() const
    {
        return new Circle(radius);
    }

    void Calculate(float density)
    {
        body->mass = PI * radius * radius * density;
        body->inverseMass = 1.0f / body->mass;
        body->inertialMoment = 0.5f * body->mass * radius * radius;
        body->inverseInertialMoment = 1.0f / body->inertialMoment;
    }

    void SetOrientation(float radians)
    {
        // rotate of circle isn't relevant
    }

    void Draw() const
    {
        glColor3f(body->bodyColor.r, body->bodyColor.g, body->bodyColor.b);
        glBegin(GL_LINE_LOOP);
        float theta = body->orientation;
        float angle = PI * 2.0f / (float)circlePoints;
        for (int i = 0; i < circlePoints; i++)
        {
            theta += angle;
            Vector2D p(std::cos(theta), std::sin(theta));
            p *= radius;
            p += body->position;
            glVertex2f(p.x, p.y);
        }
        glEnd();
    }

    int GetType() const
    {
        return CircleID;
    }
};

#endif // CIRCLE_H