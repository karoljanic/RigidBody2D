/*
* Copyright (c) 2021 Karol Janic
*/

#ifndef WORLD_H
#define WORLD_H

#include "Math.h"

// World class
class World
{
public:
    float dt;
    unsigned int iterations;
    std::vector<RigidBody*> bodies;
    std::vector<ContactPoint> contacts;

    // constructor
    // _dt - constant which is use in integration
    // _iterations - number of iterations to animate
    World(float _dt, unsigned int _iterations);

    // adds a new RigidBody
    // _shape - poiter to shape, creating body will be have this shape
    // ( _x, _y ) - pointer to center body position
    RigidBody* Add(Shape* _shape, int _x, int _y);

    // carries out one frame of simulation 
    void Step();

    // draws a current world
    void Render();

};

#endif // WORLD_H