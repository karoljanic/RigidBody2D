/*
* Copyright (c) 2021 Karol Janic
*/

#ifndef TICTAC_H
#define TICTAC_H

#include "IncludesManager.h"

World scene(1.0f / 60.0f, 10);
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
            Vector2D* vertices = new Vector2D[30];
            vertices[0] = Vector2D(0, 3);
            vectices[1] = 
            Poly poly(vertices, 30);
            RigidBody* body = scene.Add(&poly, x, y);
            body->SetOrientation(Random(-PI, PI));
            body->restitution = 0.2f;
            body->kinetcFriction = 0.2f;
            body->staticFriction = 0.4f;
            body->SetColor(Random(0, 1), Random(0, 1), Random(0, 1));
            delete[] vertices;
        }
        break;
        case GLUT_RIGHT_BUTTON:
        {
            Circle circ(Random(1.0f, 3.0f));
            RigidBody* body = scene.Add(&circ, x, y);
            body->SetColor(Random(0, 1), Random(0, 1), Random(0, 1));
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

    accumulator = Clamp(0.0f, 0.1f, accumulator);
    while (accumulator >= dt)
    {
        scene.Step();
        accumulator -= dt;
    }

    clock.Stop();
    scene.Render();
    glutSwapBuffers();
}

void StartTicTacRebound(int argc, char** argv)
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

#endif TICTAC_H