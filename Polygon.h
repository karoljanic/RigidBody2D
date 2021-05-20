/*
* Copyright (c) 2021 Karol Janic
*/

#ifndef POLYGON_H
#define POLYGON_H

// max number of vertices in polygon model
#define MaxPolyVertexCount 128


// Poly class - unique shape 
class Poly : public Shape
{
public:
    Matrix2X2 orientation;
    int verticesCount;
    Vector2D verticesArray[MaxPolyVertexCount];
    Vector2D normalVectors[MaxPolyVertexCount];

    // virtual constructor
    Poly() {}

    // costructor
    // _vertices - poiter to vector 2d array with subsequent vertices 
    // _count - number of vertices; must be between 3 and MaxPolyVertexCount
    Poly(Vector2D* _vertices, int _count)
    {
        // finding tha most right point of polygon
        int theMostRight = 0;
        float currentValue = _vertices[0].x;
        for (int i = 1; i < _count; i++)
        {
            float x = _vertices[i].x;
            if (x > currentValue)
            {
                currentValue = x;
                theMostRight = i;
            }
            else if (x == currentValue && (_vertices[i].y < _vertices[theMostRight].y))
            {
                theMostRight = i;
            }
        }

        int vrtx[MaxPolyVertexCount];
        int vrtxIndex = theMostRight;
        int k = 0;

        // sorting vertices by angle (counterclockwise)
        while (true)
        {
            vrtx[k] = vrtxIndex;
            int nextVrtxIndex = 0;
            for (int i = 1; i < _count; i++)
            {
                if (nextVrtxIndex == vrtxIndex)
                {
                    nextVrtxIndex = i;
                    continue;
                }

                Vector2D point1 = _vertices[nextVrtxIndex] - _vertices[vrtx[k]];
                Vector2D point2 = _vertices[i] - _vertices[vrtx[k]];
                float res = cross(point1, point2);
                if (res < 0.0f)
                    nextVrtxIndex = i;

                if (res == 0.0f && point2.lengthPower2() > point1.lengthPower2())
                    nextVrtxIndex = i;
            }

            k++;
            vrtxIndex = nextVrtxIndex;

            if (nextVrtxIndex == theMostRight)
            {
                verticesCount = k;
                break;
            }
        }

        for (int i = 0; i < verticesCount; i++)
            verticesArray[i] = _vertices[vrtx[i]];

        // calculate face normal vectors
        int j;
        for (int i = 0; i < verticesCount; i++)
        {   
            j = (i + 1)% verticesCount;
  
            // vector AB = [B_x - A_x, B_y - A_y]
            Vector2D face = verticesArray[j] - verticesArray[i];

            // if the side length is close to zero, the result of the normal vector is unphysical 
            assert(face.lengthPower2() > EPSILON * EPSILON);
            
            // vector A1x + B1y + C1 is perpendicular to A2x + B2x + C2 <=> A1 * A2 + B1 * B2 == 0, so
            normalVectors[i] = Vector2D(face.y, -face.x);
            // the normal vector must be normalised 
            normalVectors[i].normalize();
        }
    }

    Shape* Copy() const
    {
        Vector2D* vertices = new Vector2D[verticesCount];
        for (int i = 0; i < verticesCount; i++)
            vertices[i] = verticesArray[i];
            
        Poly* poly = new Poly();
        poly->orientation = orientation;
        for (int i = 0; i < verticesCount; i++)
        {
            poly->verticesArray[i] = verticesArray[i];
            poly->normalVectors[i] = normalVectors[i];
        }
        poly->verticesCount = verticesCount;
        return poly;
    }

    void Calculate(float density)
    {
        float area = 0.0;
        float inertialMoment = 0.0;

        // area = 1/2 * | (x1*y2 - y1*x2) + (x2*y3 - y2*x3) + ... + (xn*y1 - yn*x1) |
        for (int i = 0; i < verticesCount; i++)
        {
            area += verticesArray[i].x * verticesArray[i + 1].y;
            area -= verticesArray[i].y * verticesArray[i + 1].x;
        }
        area += verticesArray[verticesCount - 1].x * verticesArray[0].y;
        area -= verticesArray[verticesCount - 1].y * verticesArray[0].x;
        area = 0.5 * abs(area);

        body->mass = density * area;
        if (body->mass == 0)
            body->inverseMass = 0;
        else
            body->inverseMass = 1.0 / body->mass;


        // moment of inertia = 1/12 * sum(from k=0 to k = n-1)[( xk*y(k+1) - x(k+1)*yk )( (x(k+1))^2 + x(k+1)*xk + (xk)^2 + y(k+1))^2 + y(k+1)*yk + (yk)^2 )], if k == n then k = 0
        for (int i = 0; i < verticesCount - 1; i++)
        {
            inertialMoment += (verticesArray[i].x * verticesArray[i + 1].y - verticesArray[i + 1].x * verticesArray[i].y) *
                              (verticesArray[i + 1].x * verticesArray[i + 1].x + verticesArray[i].x * verticesArray[i + 1].x + verticesArray[i].x * verticesArray[i].x +
                               verticesArray[i + 1].y * verticesArray[i + 1].y + verticesArray[i].y * verticesArray[i + 1].y + verticesArray[i].y * verticesArray[i].y);
        }
        inertialMoment += (verticesArray[verticesCount - 1].x * verticesArray[0].y - verticesArray[0].x * verticesArray[verticesCount - 1].y) *
                          (verticesArray[0].x * verticesArray[0].x + verticesArray[verticesCount - 1].x * verticesArray[0].x + verticesArray[verticesCount - 1].x * verticesArray[verticesCount - 1].x +
                           verticesArray[0].y * verticesArray[0].y + verticesArray[verticesCount - 1].y * verticesArray[0].y + verticesArray[verticesCount - 1].y * verticesArray[verticesCount - 1].y);

        inertialMoment *= 0.0833;

        body->inertialMoment = density * inertialMoment;
        if (body->inertialMoment == 0)
            body->inverseInertialMoment = 0;
        else
            body->inverseInertialMoment = 1.0 / body->inertialMoment;
    }

    void SetOrientation(float radians)
    {
        orientation = Matrix2X2(radians);
    }

    void Draw() const
    {
        glColor3f(body->bodyColor.red, body->bodyColor.green, body->bodyColor.blue);
        glBegin(GL_POLYGON);
        for (int i = 0; i < verticesCount; i++)
        {
            Vector2D v = body->position + orientation * verticesArray[i];
            glVertex2f(v.x, v.y);
        }
        glEnd();
    }

    int GetType() const
    {
        return PolygonID;
    }

    // returns extreme point in a polygon
    Vector2D GetExtreme(const Vector2D& direction)
    {
        float currentValue = -FLT_MAX;
        Vector2D currentVertex;
        Vector2D bestVertex;
        float scalar;
        for (int i = 0; i < verticesCount; i++)
        {
            currentVertex  = verticesArray[i];
            scalar = dot(currentVertex, direction);

            if (scalar > currentValue)
            {
                bestVertex = currentVertex;
                currentValue = scalar;
            }
        }

        return bestVertex;
    }
};

#endif // POLYGON_H