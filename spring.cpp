/*
 *  Spring Simulation
 *  --
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

// Spring variables :
static GLfloat x[3];
static GLfloat v[3];
static GLfloat F[3];

// Spring parameters:
static const  GLfloat k = 0.5f;      // spring constant
static const  GLfloat l0 = 0.8f;     // rest length
static const  GLfloat m = 1.0f;      // ball mass
static const  GLfloat r = 0.05f;     // ball radius
static const  GLfloat d = 0.05f;     // damping constant
static const GLfloat xO[3] = {0.0f, 0.8f, 0.0f};          // the fixed end coordinate

// Gravity:
static const GLfloat grav = -0.01f;

// Simulation parameters+variables:
static const int sPF = 2;  // steps per frame
static float dt = 0.0;      // time interval - to be set later
static struct timeval prev_time;   // time

void initialState()  // set initial positions and velocities
{
// Initial position:
  x[0] = 0.0f;
  x[1] = -0.2f;
  x[2] = 0.0f;

// Initial velocity: - randomize within a range
  v[0] = 0.0;
  v[1] = 0.0;
  v[2] = 0.0;
}

void updateForce()
{
  double dlt = 0;   // current length square
  double dvx = 0;   // v.x
  for (int k=0; k<3; k++){
    dlt += pow((x[k]-xO[k]), 2.0);
    dvx += (v[k]-0.0f)*(x[k]-xO[k]);
  }

// The forces:
// The spring force:
  GLfloat f_s[3];
  GLfloat lt = (GLfloat)(sqrt(dlt));
  for (int k=0; k<3; k++){
    f_s[k] = -k*(x[k]-xO[k])/lt*(lt-l0);
  }

// The damping force:
  GLfloat f_d[3];
  for (int k=0; k<3; k++){
    f_d[k] = -d*(x[k]-xO[k])/dlt*dvx;
  }

// The gravity
  GLfloat f_g[3];
  f_g[0] = 0.0f;
  f_g[1] = grav;
  f_g[2] = 0.0f;

// The total force:
  for (int k=0; k<3; k++){
    F[k] = f_s[k] + f_d[k] + f_g[k];
  }

}

void updateState_explicitEuler()
{
// Explicit Euler
  updateForce();
  for (int k=0; k<3; k++){
    x[k] = x[k] + v[k]*dt;
    v[k] = v[k] + F[k]/m*dt;
  }
}

// Runge-Kutta
void updateState_RK2()
{
// K1:
  GLfloat K1x[3];   // xt
  GLfloat K1v[3];   // vt
  GLfloat K1a[3];   // at
  updateForce();
  for (int k=0; k<3; k++){
    K1x[k] = x[k];
    K1v[k] = v[k];
    K1a[k] = F[k]/m;
  }
// K2:
  for (int k=0; k<3; k++){
    x[k] = x[k] + 0.5*dt*K1v[k];
    v[k] = v[k] + 0.5*dt*K1a[k];  // K2v
  }
  updateForce();   // F at t+dt/2,  K2a = F(t+dt/2)/m

// Update state:
  for (int k=0; k<3; k++){
    x[k] = K1x[k] + dt*v[k];
    v[k] = K1v[k] + dt*F[k]/m;
  }

}

void updateState_RK4()
{
// K1:
  GLfloat K1x[3];   // xt
  GLfloat K1v[3];   // vt
  GLfloat K1a[3];   // at
  updateForce();            // Ft
  for (int k=0; k<3; k++){
    K1x[k] = x[k];
    K1v[k] = v[k];
    K1a[k] = F[k]/m;
  }
// K2:
  GLfloat K2x[3];
  GLfloat K2v[3];
  GLfloat K2a[3];
  for (int k=0; k<3; k++){
    K2x[k] = x[k] + 0.5*dt*K1v[k];
    K2v[k] = v[k] + 0.5*dt*K1a[k];
    x[k] = K2x[k];   // to compute F
    v[k] = K2v[k];   // to compute F
  }
  updateForce();   // F at t+dt/2,  K2a = F(S2, t+dt/2)/m
  for (int k=0; k<3; k++){
    K2a[k] = F[k]/m;
  }
// K3:
  GLfloat K3x[3];
  GLfloat K3v[3];
  GLfloat K3a[3];
  for (int k=0; k<3; k++){
    K3x[k] = K1x[k] + 0.5*dt*K2v[k];
    K3v[k] = K1v[k] + 0.5*dt*K2a[k];
    x[k] = K3x[k];   // to compute F
    v[k] = K3v[k];   // to compute F
  }
  updateForce();   // F at t+dt/2,  K3a = F(S3, t+dt/2)/m
  for (int k=0; k<3; k++){
    K3a[k] = F[k]/m;
  }
// K4:
  GLfloat K4x[3];
  GLfloat K4v[3];
  GLfloat K4a[3];
  for (int k=0; k<3; k++){
    K4x[k] = K1x[k] + dt*K3v[k];
    K4v[k] = K1v[k] + dt*K3a[k];
    x[k] = K4x[k];   // to compute F
    v[k] = K4v[k];   // to compute F
  }
  updateForce();   // F at t+dt,  K3a = F(S4, t+dt)/m
  for (int k=0; k<3; k++){
    K4a[k] = F[k]/m;
  }
// Update state:
  for (int k=0; k<3; k++){
    x[k] = K1x[k] + dt/6*(K1v[k] +2.0*K2v[k] +2.0*K3v[k] +K4v[k]);
    v[k] = K1v[k] + dt/6*(K1a[k] +2.0*K2a[k] +2.0*K3a[k] +K4a[k]);
  }

}

void initGL() {
  glClearColor(0.0f, 0.0f, 0.1f, 1.0f);
}

void display()
{
  float length = pow((x[0]-xO[0])*(x[0]-xO[0])+(x[1]-xO[1])*(x[1]-xO[1]),0.5);
  float theta = atan2(x[0]-xO[0],xO[1]-x[1]);
  glClear(GL_COLOR_BUFFER_BIT);
  drawSpring(xO[0], xO[1], x[0], x[1], length, theta, 10);
  drawBall(x[0], x[1], r);
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
  glutCreateWindow("Spring Simulation");


  glutDisplayFunc(display);
  glutIdleFunc (AnimateScene);
  initGL();
  initialState();
  gettimeofday (&prev_time, NULL);
  glutMainLoop();

  return 0;
}
