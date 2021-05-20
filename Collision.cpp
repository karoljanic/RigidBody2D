/*
* Copyright (c) 2021 Karol Janic
*/

#include "IncludesManager.h"

const float K_BIAS_RELATIVE = 0.95f;
const float K_BIAS_ABSOLUTE = 0.01f;

void CircleToCircle(ContactPoint* point, RigidBody* bodyA, RigidBody* bodyB)
{
    Circle* A = (Circle*)(bodyA->shape);
    Circle* B = (Circle*)(bodyB->shape);

    Vector2D normal = bodyB->position - bodyA->position;
    float distancePower2 = normal.lengthPower2();
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
    center = B->orientation.transpose() * (center - bodyB->position);

    float separation = -FLT_MAX;
    int faceNormal = 0;
    for (int i = 0; i < B->verticesCount; i++)
    {
        float s = dot(B->normalVectors[i], center - B->verticesArray[i]);
        if (s > A->radius)
            return;

        if (s > separation)
        {
            separation = s;
            faceNormal = i;
        }
    }

    Vector2D vector1 = B->verticesArray[faceNormal];
    int j = faceNormal + 1 < B->verticesCount ? faceNormal + 1 : 0;
    Vector2D vector2 = B->verticesArray[j];

    if (separation < EPSILON)
    {
        point->contact_count = 1;
        point->normal = -(B->orientation * B->normalVectors[faceNormal]);
        point->contacts[0] = point->normal * A->radius + bodyA->position;
        point->penetration = A->radius;
        return;
    }

    float dot1 = dot(center - vector1, vector2 - vector1);
    float dot2 = dot(center - vector2, vector1 - vector2);
    point->penetration = A->radius - separation;

    if (dot1 <= 0.0f)
    {
        Vector2D vctr1 = center - vector1;
        if (dot(vctr1, vctr1) > A->radius * A->radius)
            return;

        point->contact_count = 1;
        Vector2D n = vector1 - center;
        n = B->orientation * n;
        n.normalize();
        point->normal = n;
        vector1 = B->orientation * vector1 + bodyB->position;
        point->contacts[0] = vector1;
    }
    else if (dot2 <= 0.0f)
    {
        Vector2D vctr2 = center - vector2;
        if (dot(vctr2, vctr2) > A->radius * A->radius)
            return;

        point->contact_count = 1;
        Vector2D vector3 = vector2 - center;
        vector2 = B->orientation * vector2 + bodyB->position;
        point->contacts[0] = vector2;
        vector3 = B->orientation * vector3;
        vector3.normalize();
        point->normal = vector3;
    }
    else
    {
        Vector2D n = B->normalVectors[faceNormal];
        if (dot(center - vector1, n) > A->radius)
            return;

        n = B->orientation * n;
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

    for (int i = 0; i < polyA->verticesCount; i++)
    {
        Vector2D n = polyA->normalVectors[i];
        Vector2D nw = polyA->orientation * n;

        Matrix2X2 buT = polyB->orientation.transpose();
        n = buT * nw;

        Vector2D s = polyB->GetExtreme(-n);

        Vector2D v = polyA->verticesArray[i];
        v = polyA->orientation * v + polyA->body->position;
        v -= polyB->body->position;
        v = buT * v;

        float d = dot(n, s - v);

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
    Vector2D referenceNormal = RefPoly->normalVectors[referenceIndex];

    referenceNormal = RefPoly->orientation * referenceNormal;
    referenceNormal = IncPoly->orientation.transpose() * referenceNormal;

    int incidentFace = 0;
    float minDot = FLT_MAX;
    for (int i = 0; i < IncPoly->verticesCount; i++)
    {
        float dot1 = dot(referenceNormal, IncPoly->normalVectors[i]);
        if (dot1 < minDot)
        {
            minDot = dot1;
            incidentFace = i;
        }
    }

    vector[0] = IncPoly->orientation * IncPoly->verticesArray[incidentFace] + IncPoly->body->position;
    incidentFace = incidentFace + 1 >= (int)IncPoly->verticesCount ? 0 : incidentFace + 1;
    vector[1] = IncPoly->orientation * IncPoly->verticesArray[incidentFace] + IncPoly->body->position;
}

int Clip(Vector2D normalVector, float c, Vector2D* face)
{
    int sp = 0;
    Vector2D out[2] = {
      face[0],
      face[1]
    };

    float d1 = dot(normalVector, face[0]) - c;
    float d2 = dot(normalVector, face[1]) - c;

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

    const float k_biasRelative = 0.95f;
    const float k_biasAbsolute = 0.01f;
    //return a >= b * k_biasRelative + a * k_biasAbsolute;
    //if (biasGreaterThan(penetrationA, penetrationB))
    if (penetrationA >= penetrationB * K_BIAS_RELATIVE + penetrationA * K_BIAS_ABSOLUTE)
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

    Vector2D v1 = RefPoly->verticesArray[referenceIndex];
    referenceIndex = referenceIndex + 1 == RefPoly->verticesCount ? 0 : referenceIndex + 1;
    Vector2D v2 = RefPoly->verticesArray[referenceIndex];

    v1 = RefPoly->orientation * v1 + RefPoly->body->position;
    v2 = RefPoly->orientation * v2 + RefPoly->body->position;

    Vector2D sidePlaneNormal = (v2 - v1);
    sidePlaneNormal.normalize();

    Vector2D refFaceNormal(sidePlaneNormal.y, -sidePlaneNormal.x);

    float refC = dot(refFaceNormal, v1);
    float negSide = -dot(sidePlaneNormal, v1);
    float posSide = dot(sidePlaneNormal, v2);

    if (Clip(-sidePlaneNormal, negSide, incidentFace) < 2)
        return; 

    if (Clip(sidePlaneNormal, posSide, incidentFace) < 2)
        return; 

    point->normal = flip ? -refFaceNormal : refFaceNormal;

    int cp = 0; 
    float separation = dot(refFaceNormal, incidentFace[0]) - refC;
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

    separation = dot(refFaceNormal, incidentFace[1]) - refC;
    if (separation <= 0.0f)
    {
        point->contacts[cp] = incidentFace[1];

        point->penetration += -separation;
        cp++;

        point->penetration /= (float)cp;
    }

    point->contact_count = cp;
}