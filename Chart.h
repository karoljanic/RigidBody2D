/*
* Copyright (c) 2021 Karol Janic
*/
/*
#ifndef CHART_H
#define CHART_H

//#include "IncludesManager.h"

// mouse moves definition
void mouse(int button, int state, int x, int y)
{
    if (state == GLUT_DOWN)
        switch (button)
        {
        case GLUT_LEFT_BUTTON:
        {
         
        }
        break;
        case GLUT_RIGHT_BUTTON:
        {
            
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

    glutSwapBuffers();
}


// Fancy World - an empty stage on which we can freely create polygons and circles
void test(int argc, char** argv)
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

 
    glutMainLoop();
}

#endif //CHART_H
*/