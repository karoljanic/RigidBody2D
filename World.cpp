/*
* Copyright (c) 2021 Karol Janic
*/

#include "IncludesManager.h"
#include "Physics.h"

World::World(float _dt, unsigned int _iterations)
{
    dt = _dt;
    iterations = _iterations;
}

RigidBody* World::Add(Shape* shape, int x, int y)
{
    assert(shape);
    RigidBody* b = new RigidBody(shape, x, y, 0, 73, 34, 1);
    bodies.push_back(b);
    return b;
}

void World::Step()
{
    contacts.clear();
    for (int i = 0; i < bodies.size(); i++)
    {
        RigidBody* A = bodies[i];

        for (int j = i + 1; j < bodies.size(); j++)
        {
            RigidBody* B = bodies[j];
            if (A->inverseMass == 0 && B->inverseMass == 0)
                continue;
            ContactPoint m(A, B);
            m.Solve();
            if (m.contact_count)
                contacts.emplace_back(m);
        }
    }

    for (int i = 0; i < bodies.size(); i++)
        IntegrateForce(bodies[i], dt);

    for (int j = 0; j < iterations; j++)
    {
        for (int i = 0; i < contacts.size(); i++)
            contacts[i].ApplyImpuls();
    }
        
    for (int i = 0; i < bodies.size(); i++)
        IntegrateVelocity(bodies[i], dt);

    for (int i = 0; i < contacts.size(); i++)
        contacts[i].CorrectPosition();

    for (int i = 0; i < bodies.size(); i++)
    {
        RigidBody* b = bodies[i];
        b->force.x = 0;
        b->force.y = 0;
        b->torque = 0;
    }
}

void World::Render()
{
    for (int i = 0; i < bodies.size(); i++)
    {
        RigidBody* b = bodies[i];
        b->shape->Draw();
    }

}
