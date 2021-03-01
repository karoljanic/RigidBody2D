/*
* Copyright (c) 2021 Karol Janic
*/

#include "IncludesManager.h"

void CircleToCircle(ContactPoint* point, RigidBody* bodyA, RigidBody* bodyB)
{
    Circle* A = (Circle*)(bodyA->shape);
    Circle* B = (Circle*)(bodyB->shape);

    Vector2D normal = bodyB->position - bodyA->position;
    float distancePower2 = normal.LengthPower2();
    float radius = A->radius + B->radius;

    if (distancePower2 >= radius * radius)
    {
        point->contact_count = 0;
        return;
    }

    float distance = std::sqrt(distancePower2);
    point->contact_count = 1;

    if (distance == 0.0f)
    {
        point->penetration = A->radius;
        point->normal = Vector2D(1, 0);
        point->contacts[0] = bodyA->position;
    }
    else
    {
        point->penetration = radius - distance;
        point->normal = normal / distance; 
        point->contacts[0] = point->normal * A->radius + bodyA->position;
    }
}

void CircleToPolygon(ContactPoint* point, RigidBody* bodyA, RigidBody* bodyB)
{
    Circle* A = (Circle*)(bodyA->shape);
    Poly* B = (Poly*)(bodyB->shape);

    point->contact_count = 0;

    Vector2D center = bodyA->position;
    center = B->orient.Transpose() * (center - bodyB->position);

    float separation = -FLT_MAX;
    int faceNormal = 0;
    for (int i = 0; i < B->vrtxCount; i++)
    {
        float s = Dot(B->nrmls[i], center - B->vrtcsArray[i]);
        if (s > A->radius)
            return;

        if (s > separation)
        {
            separation = s;
            faceNormal = i;
        }
    }

    Vector2D vector1 = B->vrtcsArray[faceNormal];
    int j = faceNormal + 1 < B->vrtxCount ? faceNormal + 1 : 0;
    Vector2D vector2 = B->vrtcsArray[j];

    if (separation < EPSILON)
    {
        point->contact_count = 1;
        point->normal = -(B->orient * B->nrmls[faceNormal]);
        point->contacts[0] = point->normal * A->radius + bodyA->position;
        point->penetration = A->radius;
        return;
    }

    float dot1 = Dot(center - vector1, vector2 - vector1);
    float dot2 = Dot(center - vector2, vector1 - vector2);
    point->penetration = A->radius - separation;

    if (dot1 <= 0.0f)
    {
        Vector2D vctr1 = center - vector1;
        if (Dot(vctr1, vctr1) > A->radius * A->radius)
            return;

        point->contact_count = 1;
        Vector2D n = vector1 - center;
        n = B->orient * n;
        n.Normalize();
        point->normal = n;
        vector1 = B->orient * vector1 + bodyB->position;
        point->contacts[0] = vector1;
    }
    else if (dot2 <= 0.0f)
    {
        Vector2D vctr2 = center - vector2;
        if (Dot(vctr2, vctr2) > A->radius * A->radius)
            return;

        point->contact_count = 1;
        Vector2D vector3 = vector2 - center;
        vector2 = B->orient * vector2 + bodyB->position;
        point->contacts[0] = vector2;
        vector3 = B->orient * vector3;
        vector3.Normalize();
        point->normal = vector3;
    }
    else
    {
        Vector2D n = B->nrmls[faceNormal];
        if (Dot(center - vector1, n) > A->radius)
            return;

        n = B->orient * n;
        point->normal = -n;
        point->contacts[0] = point->normal * A->radius + bodyA->position;
        point->contact_count = 1;
    }
}

void PolygonToCircle(ContactPoint* point, RigidBody* bodyA, RigidBody* bodyB)
{
    CircleToPolygon(point, bodyB, bodyA);
    point->normal = -point->normal;
}

float FindAxisLeastPenetration(int* faceIndex, Poly* polyA, Poly* polyB)
{
    float bestDistance = -FLT_MAX;
    int bestIndex=0;

    for (int i = 0; i < polyA->vrtxCount; i++)
    {
        Vector2D n = polyA->nrmls[i];
        Vector2D nw = polyA->orient * n;

        Matrix2X2 buT = polyB->orient.Transpose();
        n = buT * nw;

        Vector2D s = polyB->GetExtreme(-n);

        Vector2D v = polyA->vrtcsArray[i];
        v = polyA->orient * v + polyA->body->position;
        v -= polyB->body->position;
        v = buT * v;

        float d = Dot(n, s - v);

        if (d > bestDistance)
        {
            bestDistance = d;
            bestIndex = i;
        }
    }

    *faceIndex = bestIndex;
    return bestDistance;
}

void FindIncidentFace(Vector2D* vector, Poly* RefPoly, Poly* IncPoly, int referenceIndex)
{
    Vector2D referenceNormal = RefPoly->nrmls[referenceIndex];

    referenceNormal = RefPoly->orient * referenceNormal; 
    referenceNormal = IncPoly->orient.Transpose() * referenceNormal; 

    int incidentFace = 0;
    float minDot = FLT_MAX;
    for (int i = 0; i < IncPoly->vrtxCount; i++)
    {
        float dot = Dot(referenceNormal, IncPoly->nrmls[i]);
        if (dot < minDot)
        {
            minDot = dot;
            incidentFace = i;
        }
    }

    vector[0] = IncPoly->orient * IncPoly->vrtcsArray[incidentFace] + IncPoly->body->position;
    incidentFace = incidentFace + 1 >= (int)IncPoly->vrtxCount ? 0 : incidentFace + 1;
    vector[1] = IncPoly->orient * IncPoly->vrtcsArray[incidentFace] + IncPoly->body->position;
}

int Clip(Vector2D normalVector, float c, Vector2D* face)
{
    int sp = 0;
    Vector2D out[2] = {
      face[0],
      face[1]
    };

    float d1 = Dot(normalVector, face[0]) - c;
    float d2 = Dot(normalVector, face[1]) - c;

    if (d1 <= 0.0f) out[sp++] = face[0];
    if (d2 <= 0.0f) out[sp++] = face[1];

    if (d1 * d2 < 0.0f)
    {
        float alpha = d1 / (d1 - d2);
        out[sp] = face[0] + alpha * (face[1] - face[0]);
        sp++;
    }

    face[0] = out[0];
    face[1] = out[1];

    assert(sp != 3);
    return sp;
}

void PolygonToPolygon(ContactPoint* point, RigidBody* bodyA, RigidBody* bodyB)
{
    Poly* A = (Poly*)(bodyA->shape);
    Poly* B = (Poly*)(bodyB->shape);
    point->contact_count = 0;

    int faceA;
    float penetrationA = FindAxisLeastPenetration(&faceA, A, B);
    if (penetrationA >= 0.0f)
        return;

    int faceB;
    float penetrationB = FindAxisLeastPenetration(&faceB, B, A);
    if (penetrationB >= 0.0f)
        return;

    int referenceIndex;
    bool flip; 
    Poly* RefPoly; 
    Poly* IncPoly; 

    if (BiasGreaterThan(penetrationA, penetrationB))
    {
        RefPoly = A;
        IncPoly = B;
        referenceIndex = faceA;
        flip = false;
    }
    else
    {
        RefPoly = B;
        IncPoly = A;
        referenceIndex = faceB;
        flip = true;
    }

    Vector2D incidentFace[2];
    FindIncidentFace(incidentFace, RefPoly, IncPoly, referenceIndex);

    Vector2D v1 = RefPoly->vrtcsArray[referenceIndex];
    referenceIndex = referenceIndex + 1 == RefPoly->vrtxCount ? 0 : referenceIndex + 1;
    Vector2D v2 = RefPoly->vrtcsArray[referenceIndex];

    v1 = RefPoly->orient * v1 + RefPoly->body->position;
    v2 = RefPoly->orient * v2 + RefPoly->body->position;

    Vector2D sidePlaneNormal = (v2 - v1);
    sidePlaneNormal.Normalize();

    Vector2D refFaceNormal(sidePlaneNormal.y, -sidePlaneNormal.x);

    float refC = Dot(refFaceNormal, v1);
    float negSide = -Dot(sidePlaneNormal, v1);
    float posSide = Dot(sidePlaneNormal, v2);

    if (Clip(-sidePlaneNormal, negSide, incidentFace) < 2)
        return; 

    if (Clip(sidePlaneNormal, posSide, incidentFace) < 2)
        return; 

    point->normal = flip ? -refFaceNormal : refFaceNormal;

    int cp = 0; 
    float separation = Dot(refFaceNormal, incidentFace[0]) - refC;
    if (separation <= 0.0f)
    {
        point->contacts[cp] = incidentFace[0];
        point->penetration = -separation;
        cp++;
    }
    else
    {
        point->penetration = 0;
    }

    separation = Dot(refFaceNormal, incidentFace[1]) - refC;
    if (separation <= 0.0f)
    {
        point->contacts[cp] = incidentFace[1];

        point->penetration += -separation;
        cp++;

        point->penetration /= (float)cp;
    }

    point->contact_count = cp;
}