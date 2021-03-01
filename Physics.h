/*
* Copyright (c) 2021 Karol Janic
*/

#ifndef PHYSICS_H
#define PHYSICS_H

#include "IncludesManager.h"

// integrates forces
void IntegrateForce(RigidBody* body, float dt)
{
    if (body->inverseMass == 0.0f)
        return;

    body->velocity += (body->force * body->inverseMass + gravity) * (dt / 2.0f);
    body->angularVelocity += body->torque * body->inverseInertialMoment * (dt / 2.0f);
}

// integrates velocities
void IntegrateVelocity(RigidBody* body, float dt)
{
    if (body->inverseMass == 0.0f)
        return;

    body->position += body->velocity * dt;
    body->orientation += body->angularVelocity * dt;
    body->SetOrientation(body->orientation);
    IntegrateForce(body, dt);
}

#endif // PHYSICS_H
