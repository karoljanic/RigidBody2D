/*
* Copyright (c) 2021 Karol Janic
*/

#ifndef FANCYWORLD_H
#define FANCYWORLD_H

#include "IncludesManager.h"

World scene(1.0f/60.0f, 10);
Timer clock;

// mouse moves definition
void mouse(int button, int state, int x, int y)
{
    x /= 10.0f;
    y /= 10.0f;

    if (state == GLUT_DOWN)
        switch (button)
        {
        case GLUT_LEFT_BUTTON:
        {
            int count = (int)random(5, MaxPolyVertexCount);
            Vector2D* vertices = new Vector2D[count];
            for (int i = 0; i < count; i++)
            {
                vertices[i].x = random(-7.0, 7.0);
                vertices[i].y = random(-7.0, 7.0);
            }

            Poly poly(vertices, count);
            RigidBody* body = scene.Add(&poly, x, y);
            body->SetOrientation(random(-PI, PI));
            body->restitution = 0.4f;
            body->kinetcFriction = 0.2f;
            body->staticFriction = 0.4f;
            body->SetColor(random(0, 1), random(0, 1), random(0, 1));
            delete[] vertices;
        }
        break;
        case GLUT_RIGHT_BUTTON:
        {
            Circle circ (random(1.0f, 3.0f));
            RigidBody* body = scene.Add(&circ, x, y);
            body->SetColor(random(0, 1), random(0, 1), random(0, 1));
        }
        break;
        }
}

// keyboard moves definition
void keyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
    case 27:
        exit(0);
        break;
    }
}

void loop()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    static double accumulator = 0;
    accumulator += clock.Time();

    clock.Start();

    //accumulator = clamp(0.0f, 0.1f, accumulator);

    if (accumulator < 0.0) 
        accumulator = 0.0;
    if (accumulator > 0.1) 
        accumulator = 0.1;

    while (accumulator >= dt)
    {
        scene.Step();
        accumulator -= dt;
    }

    clock.Stop();
    scene.Render();
    glutSwapBuffers();
}


// Fancy World - an empty stage on which we can freely create polygons and circles
void InitFancyWorld(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Rigid Body 2D Simulator: Fancy World");
    glutDisplayFunc(loop);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutIdleFunc(loop);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, 80, 60, 0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    RigidBody* body; 

    Rect rect(50.0f, 1.0f);
    body = scene.Add(&rect, 40, 55);
    body->SetStatic();
    body->SetOrientation(0);

    srand(1);
    glutMainLoop();
}

#endif //FANCYWORLD_H
