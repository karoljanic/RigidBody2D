/*
* Copyright (c) 2021 Karol Janic
*/

#include "IncludesManager.h"

// if we don't want to define mass and moment of inertial - automatic calculations
#define CalculateMassAndInertialMoment 
// if we want to define mass and moment of inertial
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
    inverseMass = 1.0 / mass;
    inertialMoment = _inertialMoment;
    inverseInertialMoment = 1.0 / inertialMoment;
#endif
    
    velocity.x = 0;
    velocity.y = 0;
    angularVelocity = 0.0;

    torque = 0.0;
    orientation = _orientation;
    force.x = 0;
    force.y = 0;
    staticFriction = 0.4;
    kinetcFriction = 0.3;
    restitution = 1.0;
}

void RigidBody::ApplyForce(const Vector2D& _force)
{
    force += _force;
}

void RigidBody::ApplyImpulse(const Vector2D& impulse, const Vector2D& contactVector)
{
    // impulse / mass = (force * delta time) / mass = (force / mass) * delta time = acceleration * delta time = delta velocity
    velocity += impulse * inverseMass; 

    // (contact vector x impulse) / inertial moment = (torque * delta time) /  inertial moment = (torque / inertial moment) * delta time = angular acceleration * delta time = delta angular velocity
    angularVelocity += cross(contactVector, impulse) * inverseInertialMoment;
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
    // if mass and moment of inertia are zero, then forces and torques are also zero, so nothing acts on the body 
    mass = 0.0;
    inertialMoment = 0.0;
    inverseMass = 0.0;
    inverseInertialMoment = 0.0;
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
    bodyColor.red = r;
    bodyColor.green = g;
    bodyColor.blue = b;
}
