/*
* Copyright (c) 2021 Karol Janic
*/

#ifndef COLLISION_H
#define COLLISION_H

#include "Shape.h"

class RigidBody;
class ContactPoint;

// solves circle - circle collision
void CircleToCircle(ContactPoint* point, RigidBody* bodyA, RigidBody* bodyB);

// solves circle - polygon collision
void CircleToPolygon(ContactPoint* point, RigidBody* bodyA, RigidBody* bodyB);

// solves polygon - circle collision
void PolygonToCircle(ContactPoint* point, RigidBody* bodyA, RigidBody* bodyB);

// solves polygon - polygon collision
void PolygonToPolygon(ContactPoint* point, RigidBody* bodyA, RigidBody* bodyB);

#endif // COLLISION_H