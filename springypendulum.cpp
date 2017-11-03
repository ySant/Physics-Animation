/*
 *  Springy Pendulum Simulation
 *  --
 *  g++ springypendulum.cpp obj.cpp
 *          -o springypendulum.o
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
static GLfloat l;
static GLfloat ldot;
static GLfloat ldotdot;
static GLfloat x[3];    // position

// The pendulum parameters:
static const  GLfloat l0 = 0.8f;    // rest length
static const GLfloat k = 0.5f;     // spring constant
static const  GLfloat m = 1.0f;    // ball mass
static const  GLfloat r = 0.05f;   // ball radius
static const  GLfloat g = 0.1f;    // gravity
static const GLfloat da = 0.0005f;    // air damping
static const GLfloat ds = 0.01f;    // spring damping
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
// Initial length:
  l = 1.0f;
// Initial radial velocity:
  ldot = 0.1f;
}

void updateState_explicitEuler()
{
// Explicit Euler
  thetadotdot = -2.0*ldot*thetadot/l -g/l*sin(theta) - da/m*thetadot;
  ldotdot = l*thetadot*thetadot - g*(1.0-cos(theta)) - k/m*(l-l0) - ds/m*ldot;

  theta = theta + thetadot*dt;
  thetadot = thetadot + thetadotdot*dt;
  l = l + ldot*dt;
  ldot = ldot + ldotdot*dt;
}

// Runge-Kutta
void updateState_RK2()
{
// K1:
  GLfloat K1theta;
  GLfloat K1l;
  GLfloat K1vtheta;
  GLfloat K1vl;   
  GLfloat K1atheta;  
  GLfloat K1al;
  GLfloat K2atheta;  
  GLfloat K2al;

//  updateForce();
  K1atheta = -2.0*ldot*thetadot/l -g/l*sin(theta) - da/m*thetadot;  
  K1al = l*thetadot*thetadot - g*(1.0-cos(theta)) - k/m*(l-l0) - ds/m*ldot;
  K1theta = theta;
  K1l = l;
  K1vtheta = thetadot;
  K1vl = ldot;   
  
// K2:
  theta = theta + 0.5*dt*K1vtheta;
  thetadot = thetadot + 0.5*dt*K1atheta;  // K2v
  l = l + 0.5*dt*K1vl;
  ldot = ldot + 0.5*dt*K1al;  // K2v
//  updateForce();   // F at t+dt/2,  K2a = F(t+dt/2)/m
  K2atheta = -2.0*ldot*thetadot/l -g/l*sin(theta) - da/m*thetadot;  
  K2al = l*thetadot*thetadot - g*(1.0-cos(theta)) - k/m*(l-l0) - ds/m*ldot;
  
// Update state:
  theta = K1theta + dt*thetadot;
  thetadot = K1vtheta + dt*K2atheta;  // K2v
  l = K1l + dt*ldot;
  ldot = K1vl + dt*K2al;  // K2v
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
  drawSpring(xO[0], xO[1], x[0], x[1], l, theta, 10);
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
//    updateState_explicitEuler();
    updateState_RK2();
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
  glutCreateWindow("Springy Pendulum");


  glutDisplayFunc(display);
  glutIdleFunc (AnimateScene);
  initGL();
  initialState();
  gettimeofday (&prev_time, NULL);
  glutMainLoop();

  return 0;
}
