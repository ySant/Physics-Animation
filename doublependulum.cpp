/*
 *  Double Pendulum Animation
 *  --
 *  g++ doublependulum.cpp obj.cpp
 *          -o doublependulum.o
 *          -L/usr/lib/x86_64-linux-gnu/ -lGL -lglut
 *  -----
 *  Yudi Santoso, 2017
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include <GL/glut.h>
#include "obj.h"

using namespace obj;

// Pendulum variables
static GLfloat theta1;
static GLfloat theta1dot;
static GLfloat theta1dotdot;
static GLfloat theta2;
static GLfloat theta2dot;
static GLfloat theta2dotdot;
static GLfloat x1[3];    // position of mass 1
static GLfloat x2[3];    // position of mass 2

// The pendulum parameters:
static const  GLfloat l1 = 0.4f;    // upper length
static const  GLfloat l2 = 0.4f;    // lower length
static const  GLfloat m1 = 1.0f;    // ball 1 mass
static const  GLfloat m2 = 2.0f;    // ball 2 mass
static const  GLfloat r1 = 0.05f;   // ball 1 radius
static const  GLfloat r2 = 0.05f;   // ball 2 radius
static const  GLfloat g = 0.1f;    // gravity
static const GLfloat da = 0.0005f;    // air damping
static const GLfloat xO[3] = {0.0f, 0.4f, 0.0f};      // the fixed end coordinate

// Animation parameters:
static struct timeval prev_time;   // time

static const int sPF = 10;  // steps per frame
static float dt = 0.0;      // time interval - to be set later


void initialState()  // set initial positions and velocities
{
// Initial angles: (in radian)
  theta1 = 1.0;       
  theta2 = 1.6;      
// Initial angular velocities: (in radian/sec)
  theta1dot = 0.2;    
  theta2dot = 0.2;    
}

void updateState_explicitEuler()
{
// Explicit Euler
  theta2dotdot = 1.0/(1.0-m2/(m1+m2)*cos(theta1-theta2)*cos(theta1-theta2))*(m2/(m1+m2)*theta2dot*theta2dot*sin(theta1-theta2)*cos(theta1-theta2) + g/l2*sin(theta1)*cos(theta1-theta2) + l1/l2*theta1dot*theta1dot*sin(theta1-theta2) - g/l2*sin(theta2));
  theta1dotdot = -m2/(m1+m2)*l2/l1*(theta2dotdot*cos(theta1-theta2) + theta2dot*theta2dot*sin(theta1-theta2)) - g/l1*sin(theta1);

  theta1 = theta1 + theta1dot*dt;
  theta1dot = theta1dot + theta1dotdot*dt;
  theta2 = theta2 + theta2dot*dt;
  theta2dot = theta2dot + theta2dotdot*dt;
}




void initGL() {
  glClearColor(0.0f, 0.0f, 0.1f, 1.0f);
}


void getPosition()
{
  x1[0] = xO[0] + l1*sin(theta1);
  x1[1] = xO[1] - l1*cos(theta1);
  x2[0] = xO[0] + l1*sin(theta1) + l2*sin(theta2);
  x2[1] = xO[1] - l1*cos(theta1) - l2*cos(theta2);

  //printf("theta1, x1: %2.2f, %2.2f \n", theta1, x1[0]);
  //printf("theta2, x2: %2.2f, %2.2f \n", theta2, x2[0]);
}


void display()
{
  getPosition();
  glClear(GL_COLOR_BUFFER_BIT);
  drawShaft(xO[0], xO[1], x1[0], x1[1]);
  drawBall(x1[0],x1[1],r1);
  drawShaft(x1[0], x1[1], x2[0], x2[1]);
  drawBall(x2[0],x2[1],r2);
  glFlush();
}

void AnimateScene()
{
// Time elapsed
  struct timeval current_time;
  gettimeofday(&current_time, NULL);
  dt = (float)(current_time.tv_sec  - prev_time.tv_sec) +
  1.0e-6*(current_time.tv_usec - prev_time.tv_usec);

// Animate the motion of the cloth
  for (int s=0; s<sPF; s++)
  {
// Choose the integrator:
    updateState_explicitEuler();
//    updateState_RK2();
//    updateState_RK4();

  }

// Save time_now for next time
  prev_time = current_time;
// Force redraw
  glutPostRedisplay();
}

int main()
{
  int argc = 1;
  int t=0;
  char *argv[1] = {(char*)"None"};
  glutInit(&argc, argv);
  glutInitWindowSize(600, 600);
  glutInitWindowPosition(50, 50);
  glutCreateWindow("Double Pendulum");


  glutDisplayFunc(display);
  glutIdleFunc (AnimateScene);
  initGL();
  initialState();
  gettimeofday (&prev_time, NULL);
  glutMainLoop();

  return 0;
}
