/* 
 *  Pendulum Simulation
 *  --
 *  g++ pendulum.cpp obj.cpp -o pendulum.o 
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
static GLfloat theta; 
static GLfloat thetadot; 
static GLfloat thetadotdot; 
static GLfloat x[3];    // position

// The pendulum parameters:
static const  GLfloat l = 0.8f;    // length 
static const  GLfloat m = 1.0f;    // ball mass 
static const  GLfloat r = 0.05f;      // radius 
static const  GLfloat g = 0.1f;    // gravity
static const GLfloat xO[3] = {0.0f, 0.8f, 0.0f};          // the fixed end coordinate

// Animation parameters:
static struct timeval prev_time;   // time 

static const int sPF = 10;  // steps per frame
static float dt = 0.0;      // time interval - to be set later


void initialState()  // set initial positions and velocities 
{
// Initial angle: 
  theta = 0.5;       // in radian
// Initial angular velocity: 
  thetadot = 0.2;    // in radian/sec

}

void updateState_explicitEuler()
{
// Explicit Euler
  thetadotdot = -g/l*sin(theta);

  theta = theta + thetadot*dt;
  thetadot = thetadot + thetadotdot*dt;

}

void initGL() {
  glClearColor(0.0f, 0.0f, 0.1f, 1.0f);
}


void getPosition()
{
  x[0] = xO[0] + l*sin(theta);
  x[1] = xO[1] - l*cos(theta);
  //printf("theta, x1: %2.2f, %2.2f \n", theta, x[1]);
}


void display()
{
  getPosition();
  glClear(GL_COLOR_BUFFER_BIT);
//  drawPoints();
  drawShaft(xO[0], xO[1],x[0], x[1]);
  drawBall(x[0],x[1],r);
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
  glutCreateWindow("Pendulum");


  glutDisplayFunc(display);
  glutIdleFunc (AnimateScene);
  initGL();
  initialState();
  gettimeofday (&prev_time, NULL);
  glutMainLoop();

  return 0;
}

