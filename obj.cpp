// obj.cpp
#include <GL/glut.h>
#include <math.h>
#include "obj.h"

void obj::drawBall(float x, float y, float r)
{
// as a circle (ball)
  for (int i=0; i <360; i++)
  {
    glBegin(GL_TRIANGLES);
    glColor3f(0.1f, 1.0f, 0.0f);
    glVertex2f(x, y);
    glVertex2f(cos(i*3.14159/180.0)*r + x, sin(i*3.14159/180.0)*r + y);
    glVertex2f(cos((i+1)*3.14159/180.0)*r + x, sin((i+1)*3.14159/180.0)*r + y);
    glEnd();
  }
}

void obj::drawShaft(float xO, float yO, float x, float y)
{
// as a line
  glBegin(GL_LINES);
  glColor3f(1.0f, 1.0f, 1.0f);
  glVertex2f(xO, yO);
  glVertex2f(x, y);
  glEnd();
}

void obj::drawSpring(float xO, float yO, float x, float y, float l, float theta, int nc)
{
// 10% on each end as line
  float xa = xO + (x-xO)/10 ;
  float ya = yO + (y-yO)/10 ;
  float xb = xO + 9*(x-xO)/10 ;
  float yb = yO + 9*(y-yO)/10 ;
// divide the rest into nc coils, drawn as zigzag line
  glBegin(GL_LINES);
  glColor3f(1.0f, 1.0f, 1.0f);
  glVertex2f(xO, yO);
  glVertex2f(xa, ya);
  glEnd();
  glBegin(GL_LINES);
  glVertex2f(xa, ya);
//  glVertex2f((0.05f-xO), (ya-0.8*l/(2*nc)-yO));
  glVertex2f((0.05f-xO)*cos(theta)-(ya-0.8*l/(2*nc)-yO)*sin(theta)+xO, (0.05f-xO)*sin(theta)+(ya-0.8*l/(2*nc)-yO)*cos(theta)+yO);
  glEnd();
  for(int i=1; i<(2*nc-1); i++){
    glBegin(GL_LINES);
//    glVertex2f((pow(-1,(i+1))*0.05f-xO), (ya-i*0.8*l/(2*nc)-yO));
    glVertex2f((pow(-1,(i+1))*0.05f-xO)*cos(theta)-(ya-i*0.8*l/(2*nc)-yO)*sin(theta)+xO, (pow(-1,(i+1))*0.05f-xO)*sin(theta)+(ya-i*0.8*l/(2*nc)-yO)*cos(theta)+yO);
    glVertex2f((pow(-1,i)*0.05f-xO)*cos(theta)-(ya-(i+1)*0.8*l/(2*nc)-yO)*sin(theta)+xO, (pow(-1,i)*0.05f-xO)*sin(theta)+(ya-(i+1)*0.8*l/(2*nc)-yO)*cos(theta)+yO);
    glEnd();
  }
  glBegin(GL_LINES);
//  glVertex2f(0.05f, ya-(2*nc-1)*0.8*l/(2*nc));
  glVertex2f((0.05f-xO)*cos(theta)-(ya-(2*nc-1)*0.8*l/(2*nc)-yO)*sin(theta)+xO, (0.05f-xO)*sin(theta)+(ya-(2*nc-1)*0.8*l/(2*nc)-yO)*cos(theta)+yO);
  glVertex2f(xb, yb);
  glEnd();
  glBegin(GL_LINES);
  glVertex2f(xb, yb);
  glVertex2f(x, y);
  glEnd();
}

