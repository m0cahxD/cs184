#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <stdio.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/LU>
#include <Eigen/SVD>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include <time.h>
#include <math.h>


using namespace std;
using namespace Eigen;

#define deltaTheta 10
#define PI 3.14159265358979323846

void idleLoop();
Vector4f nextGoal(float t);

//****************************************************
// Some Classes
//****************************************************

class Viewport;

class Viewport {
  public:
    int w, h; // width and height
};

//****************************************************
// Structures
//****************************************************

class Joint {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector4f origin;
  Vector4f end;
  Matrix4f rotation;
  Matrix4f translation;
  float length;
  float theta_x;
  float theta_y;
  float theta_z;

  Joint() {
  this->length = 1;
  this->origin << 0, 0, 0, 1;
  
    // Joint lies along the x axis
  translation << 1, 0, 0, length,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
  this->end = translation * origin;
  rotation.setIdentity();
  setRotation(0, 0, 0);
  }

  Joint(float length) {
    this->length = length;
    this->origin << 0, 0, 0, 1;

    // Joint lies along the x axis
  translation << 1, 0, 0, length,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
    this->end = translation * origin;
    rotation.setIdentity();
    setRotation(0, 0, 0);
  }
  
  void setRotation(float theta_x, float theta_y, float theta_z) {
    this->theta_x = theta_x;
    this->theta_y = theta_y;
    this->theta_z = theta_z;
    
    Matrix4f rot_x;
    Matrix4f rot_y;
    Matrix4f rot_z;
    float ux, uy, uz;
    ux = 1.0;
    uy = 0.0;
    uz = 0.0;
    rot_x <<  cos(theta_x)+ux*ux*(1-cos(theta_x)),    ux*uy*(1-cos(theta_x))-uz*sin(theta_x), ux*uz*(1-cos(theta_x))+uy*sin(theta_x), 0,
            uy*ux*(1-cos(theta_x))+uz*sin(theta_x), cos(theta_x)+uy*uy*(1-cos(theta_x)),    uy*uz*(1-cos(theta_x))-ux*sin(theta_x), 0,
            uz*ux*(1-cos(theta_x))-uy*sin(theta_x), uz*uy*(1-cos(theta_x))+ux*sin(theta_x), cos(theta_x)+uz*uz*(1-cos(theta_x)),    0,
            0,                          0,                          0,                          1;
    ux = 0.0;
    uy = 1.0;
    uz = 0.0;
    rot_y <<  cos(theta_y)+ux*ux*(1-cos(theta_y)),    ux*uy*(1-cos(theta_y))-uz*sin(theta_y), ux*uz*(1-cos(theta_y))+uy*sin(theta_y), 0,
            uy*ux*(1-cos(theta_y))+uz*sin(theta_y), cos(theta_y)+uy*uy*(1-cos(theta_y)),    uy*uz*(1-cos(theta_y))-ux*sin(theta_y), 0,
            uz*ux*(1-cos(theta_y))-uy*sin(theta_y), uz*uy*(1-cos(theta_y))+ux*sin(theta_y), cos(theta_y)+uz*uz*(1-cos(theta_y)),    0,
            0,                          0,                          0,                          1;
    ux = 0.0;
    uy = 0.0;
    uz = 1.0;
    rot_z <<  cos(theta_z)+ux*ux*(1-cos(theta_z)),    ux*uy*(1-cos(theta_z))-uz*sin(theta_z), ux*uz*(1-cos(theta_z))+uy*sin(theta_z), 0,
            uy*ux*(1-cos(theta_z))+uz*sin(theta_z), cos(theta_z)+uy*uy*(1-cos(theta_z)),    uy*uz*(1-cos(theta_z))-ux*sin(theta_z), 0,
            uz*ux*(1-cos(theta_z))-uy*sin(theta_z), uz*uy*(1-cos(theta_z))+ux*sin(theta_z), cos(theta_z)+uz*uz*(1-cos(theta_z)),    0,
            0,                          0,                          0,                          1;
    
    rotation = rot_y * rot_z * rot_x;
  }
  
  void updateRotation(float dtheta_x, float dtheta_y, float dtheta_z) {
    setRotation(theta_x + dtheta_x, theta_y + dtheta_y, theta_z + dtheta_z);
  }
};

class Arm {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector4f basepoint;
  Vector4f endpoint;
  float length;
  vector<Joint, Eigen::aligned_allocator<Joint>> joints;
  
  Arm() {
    this->length = 0;
  }
  
  // Specify the joints to use
  void updateJoints(vector<Joint, Eigen::aligned_allocator<Joint>> joints) {
    this->joints = joints;
    this->length = findLength();
    this->basepoint = joints.front().origin;
    // Forward kinematics
    updateEndpoint();
  }

  // Update the current endpoint
  void updateEndpoint() {
    Matrix4f transf;
    transf << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
    Vector4f origin(0, 0, 0, 1);

    // Compose transformations for each Joint from end to base
    for (int i = 0; i < joints.size(); i++) {
      transf = transf * joints[i].rotation;
      transf = transf * joints[i].translation;
      joints[i].end = transf * origin;
    }
    endpoint = transf * origin;
  }

  Vector4f findEndpoint(int index, int axis, float theta) {
    Joint oldJoint = joints[index];
    Joint newJoint = Joint(oldJoint.length);
    newJoint.setRotation(oldJoint.theta_x, oldJoint.theta_y, oldJoint.theta_z);
    if(axis == 0) {
      newJoint.updateRotation(theta, 0, 0);
    } else if(axis == 1) {
      newJoint.updateRotation(0, theta, 0);
    } else if(axis == 2) {
      newJoint.updateRotation(0, 0, theta);
    }

    Matrix4f transf;
    transf << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
    Vector4f origin(0, 0, 0, 1);

    for (int i = 0; i < joints.size(); i++) {
      if(i == index) {
        transf = transf * newJoint.rotation;
      } else {
        transf = transf * joints[i].rotation;
      }
      transf = transf * joints[i].translation;
    }
    return transf * origin;
  }
  
  // Find the total length of the system
  float findLength() {
    float length = 0;
    for(size_t i = 0; i < joints.size(); i++) {
      length += joints[i].length;
    }
    return length;
  }
  
  MatrixXf getJ() {
    MatrixXf J(3, 12);
    float del = 0.001;
    for(size_t i = 0; i < joints.size(); i++) {
      if(i != 0){
        Vector4f end = findEndpoint(i, 0, del);
        Vector4f dp = end - endpoint;
        dp = dp / del;
        J(0, i * 3) = dp[0];
        J(1, i * 3) = dp[1];
        J(2, i * 3) = dp[2];
        end = findEndpoint(i, 1, del);
        
        dp = end - endpoint;
        dp = dp / del;
        J(0, i * 3 + 1) = dp[0];
        J(1, i * 3 + 1) = dp[1];
        J(2, i * 3 + 1) = dp[2];
        
        end = findEndpoint(i, 2, del);
        dp = end - endpoint;
        dp = dp / del;
        J(0, i * 3 + 2) = dp[0];
        J(1, i * 3 + 2) = dp[1];
        J(2, i * 3 + 2) = dp[2];
      }else{
        Vector4f end = findEndpoint(i, 2, del);
        Vector4f dp = end - endpoint;
        dp = dp / del;
        J(0, i * 3) = 0;
        J(1, i * 3) = 0;
        J(2, i * 3) = 0;

        J(0, i * 3 + 1) = 0;
        J(1, i * 3 + 1) = 0;
        J(2, i * 3 + 1) = 0;

        J(0, i * 3 + 2) = dp[0];
        J(1, i * 3 + 2) = dp[1];
        J(2, i * 3 + 2) = dp[2];
      }
    }
    return J;
  }
  
  // Update angles for each joint in the system
  void updateAngles(float dtheta1[], float dtheta2[], float dtheta3[], float dtheta4[]) {
    joints[0].updateRotation(dtheta1[0], dtheta1[1], dtheta1[2]);
    joints[1].updateRotation(dtheta2[0], dtheta2[1], dtheta2[2]);
    joints[2].updateRotation(dtheta3[0], dtheta3[1], dtheta3[2]);
    joints[3].updateRotation(dtheta4[0], dtheta4[1], dtheta4[2]);
  }
};

//****************************************************
// Global Variables
//****************************************************
Viewport viewport;
Arm arm;
float idle_t = 0.0;
Vector4f path_goal;

//****************************************************
// reshape viewport if the window is resized
//****************************************************
void myReshape(int w, int h) {
  viewport.w = w;
  viewport.h = h;
  glViewport (0,0,viewport.w,viewport.h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  //glOrtho(-5, 5, -5, 5, -5, 5);
  gluPerspective(65.0, (float)viewport.w/viewport.h, 1, 1000);
  //gluLookAt(-5.0, 3.0, 0.0, 1, 0.6, 0, 0, 1, 0);
  gluLookAt(0, 8, 6, 0, 2.5, -1, 0, 1, 0);
}

//****************************************************
// function that does the actual drawing of stuff
//****************************************************
void drawPoint(Vector4f& point) {
  glBegin(GL_TRIANGLES);
  glColor3f(1.0f, 0.0f, 0.0f);
  glVertex3f(point(0), point(1), point(2));
  glVertex3f(point(0)+0.9, point(1)-0.9, point(2));
  glVertex3f(point(0)+0.9, point(1), point(2));
  glEnd();
}

void myDisplay() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  GLfloat ka[] = {0.1, 0.1, 0.1};
  GLfloat kd[] = {1.0, 1.0, 1.0};
  GLfloat ks[] = {1.0, 1.0, 1.0, 20.0};
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ka);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, kd);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, ks);
  glBegin(GL_POINTS);
  glPointSize(3.0);
  Vector4f path;
  for(float t = 0; t < 2*PI; t+=0.02) {
    path = nextGoal(t);
    glVertex3f(path[0], path[1], path[2]);
  }
  glEnd();
  
  glBegin(GL_TRIANGLES);
  GLfloat ka0[] = {0.1, 0.1, 0.1};
  GLfloat kd0[] = {0.53, 0.81, 0.98};
  GLfloat ks0[] = {0.1, 0.1, 1.0, 20.0};
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ka0);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, kd0);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, ks0);
  glVertex3f(100, -20, -50);
  glVertex3f(-100, -20, -50);
  glVertex3f(0, 180, -50);
  glEnd();
  
  glBegin(GL_TRIANGLES);
  GLfloat ka1[] = {0.1, 0.1, 0.1};
  GLfloat kd1[] = {0.0, 0.65, 0.07};
  GLfloat ks1[] = {0.1, 0.1, 0.1, 2.0};
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ka1);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, kd1);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, ks1);
  glVertex3f(100, -10, -45);
  glVertex3f(-100, -10, -45);
  glVertex3f(0, -10, 150);
  glEnd();
  
  GLfloat ka2[] = {0.1, 0.1, 0.1};
  GLfloat kd2[] = {0.99, 0.42, 0.52};
  GLfloat ks2[] = {0.1, 0.05, 0.05, 5.0};
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ka2);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, kd2);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, ks2);
  
  arm.updateEndpoint();
  for(int i = 0; i < arm.joints.size(); i++) {
    Joint j = arm.joints[i];
    glRotatef(j.theta_y * 180 / PI, 0.0, 1.0, 0.0);
    glRotatef(j.theta_z * 180 / PI, 0.0, 0.0, 1.0);
    glRotatef(j.theta_x * 180 / PI, 1.0, 0.0, 0.0);
    glPushMatrix();
    glRotatef(90, 0.0, 1.0, 0.0);
    glutSolidCone(j.length*0.09, j.length, 50, 50);
    glPopMatrix();
    glTranslatef(j.length, 0.0, 0.0);
    
  }
  glFlush();
  glutSwapBuffers();                                        // swap buffers (we earlier set double buffer)
}

//****************************************************
// Helper functions
//****************************************************

bool update(Vector4f& goal) {
  Vector4f goal_t = goal;
  Vector4f g_sys = goal - arm.basepoint;
  Vector3f g_sys_tmp(g_sys(0), g_sys(1), g_sys(2));
  if(g_sys_tmp.norm() > arm.length) {
    Vector3f norm_goal(g_sys(0), g_sys(1), g_sys(2));
    norm_goal = norm_goal.normalized() * arm.length * 0.99;
    goal_t << norm_goal(0), norm_goal(1), norm_goal(2), 1;
  }
  Vector4f tmp = goal_t - arm.endpoint;
  Vector3f dp(tmp(0), tmp(1), tmp(2));
  if (dp.norm() > 0.1) {
    MatrixXf J = arm.getJ();
    VectorXf dtheta = J.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(dp);
    float dtheta1[3] = {dtheta[0], dtheta[1], dtheta[2]};
    float dtheta2[3] = {dtheta[3], dtheta[4], dtheta[5]};
    float dtheta3[3] = {dtheta[6], dtheta[7], dtheta[8]};
    float dtheta4[3] = {dtheta[9], dtheta[10], dtheta[11]};
    arm.updateAngles(dtheta1, dtheta2, dtheta3, dtheta4);
    arm.updateEndpoint();
    return false;
  }
  return true;
}

// takes in a float and calculates the next goal point on our path
Vector4f nextGoal(float t) {
  float r = 3 * (1 - cos(t));
  float x = r * sin(t);
  float y = r * cos(t) + 6;
  Vector4f toRet(x, y, sin(t), 1);
  return toRet;
}

//****************************************************
// Keyboard bindings
//***************************************************

void onKeyPress(unsigned char key, int x, int y) {
}

void onDirectionalKeyPress(int key, int x, int y) {
  switch(key) {
  case GLUT_KEY_LEFT:
    break;
  case GLUT_KEY_RIGHT:
    break;
  case GLUT_KEY_UP:
    break;
  case GLUT_KEY_DOWN:
    break;
  }
}

//****************************************************
// Simple init function
//****************************************************
void initGL(int argc, char *argv[]) {
  //This tells glut to use a double-buffered window with red, green, and blue channels 
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

  // Initialize the viewport size
  viewport.w = 700;
  viewport.h = 700;
  
  // The size and position of the window
  glutInitWindowSize(viewport.w, viewport.h);
  glutInitWindowPosition(0,0);
  glutCreateWindow(argv[0]);
  
  glLoadIdentity();

  // Set default toggles to flat shading and filled mode
  glClearColor(0.0, 0.0, 0.0, 0.0);

  glEnable(GL_NORMALIZE);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);

  GLfloat pl[] = {0.0, 20.0, 0.0, 1.0};
  glLightfv(GL_LIGHT0, GL_POSITION, pl);
  
  glutDisplayFunc(myDisplay);       // function to run when its time to draw something
  glutReshapeFunc(myReshape);       // function to run when the window gets resized

  // Add keyboard bindings
  glutKeyboardFunc(onKeyPress);
  glutSpecialFunc(onDirectionalKeyPress);
  glutIdleFunc(idleLoop);
}

void idleLoop() {
  path_goal = nextGoal(idle_t);
  bool finished = false;
  int count = 0;
  while(!finished) {
    finished = update(path_goal);
  }
  myDisplay();
  idle_t += 0.0008;
}

//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {
  initGL(argc, argv);
  
  // Lower -> higher index corresponds to base -> end of the arm
  Joint j1(2.0);
  Joint j2(2.0);
  Joint j3(1.0);
  Joint j4(0.8);

  vector<Joint, Eigen::aligned_allocator<Joint>> joints;
  joints.push_back(j1);
  joints.push_back(j2);
  joints.push_back(j3);
  joints.push_back(j4);

  // Add joints to the system
  arm.updateJoints(joints);

  glutMainLoop();

  return 0;
}