/*
* Copyright (c) 2021 Karol Janic
*/

#ifndef CONTACTPOINT_H
#define CONTACTPOINT_H

#include <math.h>
#include "Constans.h"

class RigidBody;


// ContactPoint class
class ContactPoint
{
public:
    RigidBody* bodyA;
    RigidBody* bodyB;

    float penetration;   
    Vector2D normal;
    Vector2D contacts[2];
    int contact_count; 
    float resultantRestitution;              
    float resultantKineticFriction;            
    float resultantStaticFriction;   

    float penetrationAllowance = 0.05f;
    float penetrationPercent = 0.4f;

    // constructor 
    // _bodyA and _bodyB - bodies between which collision occurs 
    ContactPoint(RigidBody* _bodyA, RigidBody* _bodyB)
    {
        bodyA = _bodyA;
        bodyB = _bodyB;

        resultantRestitution = bodyA->restitution * bodyB->restitution;
        resultantStaticFriction = std::sqrt(bodyA->staticFriction * bodyB->staticFriction);
        resultantKineticFriction = std::sqrt(bodyA->kinetcFriction * bodyB->kinetcFriction);

        for (int i = 0; i < contact_count; i++)
        {
            Vector2D ra = contacts[i] - bodyA->position;
            Vector2D rb = contacts[i] - bodyB->position;

            Vector2D rv = bodyB->velocity + Cross(bodyB->angularVelocity, rb) - bodyA->velocity - Cross(bodyA->angularVelocity, ra);

            if (rv.LengthPower2() < (dt * gravity).LengthPower2() + EPSILON)
                resultantRestitution = 0.0f;
        }
    }

    // solves collision
    void Solve()
    {
        if (bodyA->shape->GetType() == bodyB->shape->GetType())
        {
            if (bodyA->shape->GetType() == 0)
                CircleToCircle(this, bodyA, bodyB);
            else
                PolygonToPolygon(this, bodyA, bodyB);
        }
        else
        {
            if (bodyA->shape->GetType() == 0)
                CircleToPolygon(this, bodyA, bodyB);
            else
                PolygonToCircle(this, bodyA, bodyB);
        }
    }

    // applies impulses
    void ApplyImpuls()
    {
        if (Equal(bodyA->inverseMass + bodyB->inverseMass, 0))
        {
            bodyA->velocity.x = 0;
            bodyA->velocity.y = 0;
            bodyB->velocity.x = 0;
            bodyB->velocity.y = 0;
            return;
        }

        for (int i = 0; i < contact_count; i++)
        {
            Vector2D ra = contacts[i] - bodyA->position;
            Vector2D rb = contacts[i] - bodyB->position;

            Vector2D rv = bodyB->velocity + Cross(bodyB->angularVelocity, rb) - bodyA->velocity - Cross(bodyA->angularVelocity, ra);

            float contactVel = Dot(rv, normal);

            if (contactVel > 0)
                return;

            float raCrossN = Cross(ra, normal);
            float rbCrossN = Cross(rb, normal);
            float invMassSum = bodyA->inverseMass + bodyB->inverseMass + raCrossN * raCrossN * bodyA->inverseInertialMoment + rbCrossN * rbCrossN * bodyB->inverseInertialMoment;

            float j = -(1.0f + resultantRestitution) * contactVel;
            j /= invMassSum;
            j /= (float)contact_count;

            Vector2D impulse = normal * j;
            bodyA->ApplyImpulse(-impulse, ra);
            bodyB->ApplyImpulse(impulse, rb);

            rv = bodyB->velocity + Cross(bodyB->angularVelocity, rb) - bodyA->velocity - Cross(bodyA->angularVelocity, ra);

            Vector2D t = rv - (normal * Dot(rv, normal));
            t.Normalize();

            float jt = -Dot(rv, t);
            jt /= invMassSum;
            jt /= (float)contact_count;

            if (Equal(jt, 0.0f))
                return;

            Vector2D tangentImpulse;
            if (std::abs(jt) < j * resultantStaticFriction)
                tangentImpulse = t * jt;
            else
                tangentImpulse = t * -j * resultantKineticFriction;

            bodyA->ApplyImpulse(-tangentImpulse, ra);
            bodyB->ApplyImpulse(tangentImpulse, rb);
        }
    }

    // corrects bodies position
    void CorrectPosition()
    {
        if (penetration > penetrationAllowance)
        {
            Vector2D correction = ((penetration - penetrationAllowance) / (bodyA->inverseMass + bodyB->inverseMass)) * normal * penetrationPercent;
            bodyA->position -= correction * bodyA->inverseMass;
            bodyB->position += correction * bodyB->inverseMass;
        }
    }

};

#endif // CONTACTPOINT_H