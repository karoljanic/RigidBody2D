/*
* Copyright (c) 2021 Karol Janic
*/

#include "IncludesManager.h"

// if we don't want define mass and moment of inertial - automatic calculations
#define CalculateMassAndInertialMoment 
// if we want define mass and moment of inertial
// #define NoCalculateMassAndInertialMoment 


RigidBody::RigidBody(Shape* _shape, float x, float y, float _orientation, float _mass, float _inertialMoment, float _density = 1)
{
    shape = _shape->Copy();
    shape->body = this;

    position.x = x;
    position.y = y;

#if defined(CalculateMassAndInertialMoment)
    shape->Calculate(_density);
#elif defined(NoCalculateMassAndInertialMoment)
    mass = _mass;
    inverseMass = 1.0f / mass;
    inertialMoment = _inertialMoment;
    inverseInertialMoment = 1.0f / inertialMoment;
#endif
    
    velocity.x = 0;
    velocity.y = 0;
    angularVelocity = 0.0f;

    torque = 0.0f;
    orientation = _orientation;
    force.x = 0;
    force.y = 0;
    staticFriction = 0.5f;
    kinetcFriction = 0.3f;
    restitution = 0.3f;
}

void RigidBody::ApplyForce(const Vector2D& _force)
{
    force += _force;
}

void RigidBody::ApplyImpulse(const Vector2D& impulse, const Vector2D& contactVector)
{
    velocity += impulse * inverseMass;
    angularVelocity += Cross(contactVector, impulse) * inverseInertialMoment;
}

void RigidBody::SetVelocity(const Vector2D& linearVelocity, float _angularVelocity)
{
    velocity.x = linearVelocity.x;
    velocity.y = linearVelocity.y;
    angularVelocity = _angularVelocity;
}

void RigidBody::SetTorque(float _torque)
{
    torque = _torque;
}

void RigidBody::SetStatic()
{
    mass = 0.0f;
    inertialMoment = 0.0f;
    inverseMass = 0.0f;
    inverseInertialMoment = 0.0f;
}

void RigidBody::SetFrictions(float _staticFriction, float _kineticFriction, float _restitution)
{
    staticFriction = _staticFriction;
    kinetcFriction = _kineticFriction;
    restitution = _restitution;
}

void RigidBody::SetOrientation(float _orientation)
{
    orientation = _orientation;
    shape->SetOrientation(_orientation);
}

void RigidBody::SetColor(float r, float g, float b)
{
    bodyColor.r = r;
    bodyColor.g = g;
    bodyColor.b = b;
}
