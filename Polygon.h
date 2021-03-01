/*
* Copyright (c) 2021 Karol Janic
*/

#ifndef POLYGON_H
#define POLYGON_H

// max number of vertices in polygin model
#define MaxPolyVertexCount 128


// Poly class - unique shape = polygon
class Poly : public Shape
{
public:
    int vrtxCount;
    Vector2D vrtcsArray[MaxPolyVertexCount];
    Vector2D nrmls[MaxPolyVertexCount];

    // virtual constructor
    Poly() {}

    // _vertices - poiter to vector 2d array with subsequent vertices 
    // _count - number of vertices; must be between 3 and MaxPolyVertexCount
    Poly(Vector2D* _vertices, int _count)
    {
        int theMostRight = 0;
        float theMostTop = _vertices[0].x;
        for (int i = 1; i < _count; i++)
        {
            float x = _vertices[i].x;
            if (x > theMostTop)
            {
                theMostTop = x;
                theMostRight = i;
            }
            else if (x == theMostTop && (_vertices[i].y < _vertices[theMostRight].y))
            {
                theMostRight = i;
            }
        }

        int vrtx[MaxPolyVertexCount];
        int vrtxIndex = theMostRight;
        int cnt = 0;

        while (true)
        {
            vrtx[cnt] = vrtxIndex;

            // finding the first one on the left
            int nextVrtxIndex = 0;
            for (int i = 1; i < _count; i++)
            {
                if (nextVrtxIndex == vrtxIndex)
                {
                    nextVrtxIndex = i;
                    continue;
                }

                Vector2D pnt1 = _vertices[nextVrtxIndex] - _vertices[vrtx[cnt]];
                Vector2D pnt2 = _vertices[i] - _vertices[vrtx[cnt]];
                float res = Cross(pnt1, pnt2);
                if (res < 0.0f)
                    nextVrtxIndex = i;

                if (res == 0.0f && pnt2.LengthPower2() > pnt1.LengthPower2())
                    nextVrtxIndex = i;
            }

            cnt++;
            vrtxIndex = nextVrtxIndex;

            if (nextVrtxIndex == theMostRight)
            {
                vrtxCount = cnt;
                break;
            }
        }

        for (int i = 0; i < vrtxCount; i++)
            vrtcsArray[i] = _vertices[vrtx[i]];

        // calculate face normal vectors
        for (int i1 = 0; i1 < vrtxCount; i1++)
        {
            int i2 = i1 + 1 < vrtxCount ? i1 + 1 : 0;
            Vector2D face = vrtcsArray[i2] - vrtcsArray[i1];

            assert(face.LengthPower2() > EPSILON * EPSILON);

            nrmls[i1] = Vector2D(face.y, -face.x);
            nrmls[i1].Normalize();
        }
    }

    Shape* Copy() const
    {
        int len = vrtxCount;
        Vector2D* vrtcs = new Vector2D[len];
        for (int i = 0; i < len; i++)
            vrtcs[i] = vrtcsArray[i];
            
        Poly* poly = new Poly();
        poly->orient = orient;
        for (int i = 0; i < vrtxCount; i++)
        {
            poly->vrtcsArray[i] = vrtcsArray[i];
            poly->nrmls[i] = nrmls[i];
        }
        poly->vrtxCount = vrtxCount;
        return poly;
    }

    void Calculate(float density)
    {
        Vector2D centroid(0.0f, 0.0f); // centroid is a geometric centre of the polygon
        float area = 0.0f;
        float inertial = 0.0f;

        for (int i1 = 0; i1 < vrtxCount; i1++)
        {   
            Vector2D p1(vrtcsArray[i1]);
            int i2 = i1 + 1 < vrtxCount ? i1 + 1 : 0;
            Vector2D p2(nrmls[i2]);

            float vectorsCross = Cross(p1, p2);
            float triangleArea = 0.5f * vectorsCross;

            area += triangleArea;
            centroid += triangleArea * 0.333f * (p1 + p2);

            float intx2 = p1.x * p1.x + p2.x * p1.x + p2.x * p2.x;
            float inty2 = p1.y * p1.y + p2.y * p1.y + p2.y * p2.y;
            inertial += (0.0833f * vectorsCross) * (intx2 + inty2);
        }

        centroid *= 1.0f / area;
 
        body->mass = density * area;
        body->inverseMass = (body->mass) ? 1.0f / body->mass : 0.0f;
        body->inertialMoment = inertial * density;
        body->inverseInertialMoment = body->inertialMoment ? 1.0f / body->inertialMoment : 0.0f;
    }

    void SetOrientation(float radians)
    {
        orient = Matrix2X2(radians);
    }

    void Draw() const
    {
        glColor3f(body->bodyColor.r, body->bodyColor.g, body->bodyColor.b);
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < vrtxCount; i++)
        {
            Vector2D v = body->position + orient * vrtcsArray[i];
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
        float bestProjection = -FLT_MAX;
        Vector2D bestVertex;

        for (int i = 0; i < vrtxCount; i++)
        {
            Vector2D v = vrtcsArray[i];
            float projection = Dot(v, direction);

            if (projection > bestProjection)
            {
                bestVertex = v;
                bestProjection = projection;
            }
        }

        return bestVertex;
    }

};

#endif // POLYGON_H