
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <stdio.h>
#include <Eigen/Core>
#include <Eigen/Dense>

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

struct Joint {
  float length;
  float theta_x;
  float theta_y;
  float theta_z;
  
  Vector4f origin;
  Matrix4f rotation;
  Matrix4f translation;

  Joint(float length) {
    this->length = length;
    this->origin << 0, 0, 0, 1;
    
    // Joint lies along the x axis
    translation << 1, 0, 0, length;
                   0, 1, 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1;
    
    rotation.setIdentity();
    setRotation(0, 0, 0);
  }
  
  void setRotation(float theta_x, float theta_y, float theta_z) {
    this->theta_x = theta_x;
    this->theta_y = theta_y;
    this->theta_z = theta_z;
    
    Matrix4f rotation_x;
    Matrix4f rotation_y;
    Matrix4f rotation_z;
    rotation_x << 1,             0,             0,
                  0,             cos(theta_x),  -sin(theta_x),
                  0,             sin(theta_x),  cos(theta_x);
    rotation_y << cos(theta_y),  0,             sin(theta_y),
                  0,             1,             0,
                  -sin(theta_y), 0,             cos(theta_y);
    rotation_z << cos(theta_z),  -sin(theta_z), 0,
                  sin(theta_z),  cos(theta_z),  0,
                  0,             0,             1;
    rotation = rotation_z * rotation_y * rotation_x;
  }
  
  void updateRotation(float dtheta_x, float dtheta_y, float dtheta_z) {
    setRotation(theta_x + dtheta_x, theta_y + dtheta_y, theta_z + dtheta_z);
  }
};

struct Arm {
  float length;
  vector<Joint> joints;
  Vector4f basepoint;
  Vector4f endpoint;
  
  Arm() {
    this->length = 0;
  }
  
  // Specify the joints to use
  void updateJoints(vector<Joint> joints) {
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
    for (vector<Joint>::size_type i = joints.size() - 1; i >= 0; i--) {
      transf = joints[i].translation * transf;
      transf = joints[i].rotation * transf;
    }
    endpoint = transf * origin;
  }

  Vector4f findEndpoint(int index, float theta) {
    Joint oldJoint = joints[index];
    Joint newJoint = Joint(oldJoint.length);
    newJoint.setRotation(oldJoint.theta_x, oldJoint.theta_y, oldJoint.theta_z);
    newJoint.updateRotation(theta, theta, theta);

    Matrix4f transf;
    transf << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
    Vector4f origin(0, 0, 0, 1);

    for (int count = joints.size()-1; count >= 0; count--){
      transf = joints[count].translation * transf;
      if (count == index){
        transf = newJoint.rotation * transf;
      } else {
        transf = joints[count].rotation * transf;
      }
    }
    return transf * origin;
  }
  
  // Find the total length of the system
  float findLength() {
    float length = 0;
    for(vector<Joint>::size_type i = 0; i < joints.size(); i++) {
      length += joints[i].length;
    }
    return length;
  }
  
  /* Get the Jacobian matrix for the system
  MatrixXf getJ() {
  }
  */
  
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

//****************************************************
// reshape viewport if the window is resized
//****************************************************
void myReshape(int w, int h) {
  viewport.w = w;
  viewport.h = h;

  glViewport (0,0,viewport.w,viewport.h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-4, 4, -4, 4, -4, 4);
}

//****************************************************
// Helper functions
//****************************************************

bool update(Vector4f& goal) {
/*
  Vector4f g_sys = g - system.basepoint;
  
  // if (I can't reach the goal){
  //   g = new goal that can be reached
  // }
  Vector4f dp = g - system.endpoint;
  if (dp.norm() > eps){
    J = system.getJ();
    svd(J);
    dtheta = svd.solve(dp);

    system.updateAngles(dtheta);
    system.updateEndpoint;
    return false;
  }
*/
  return true;
}

// takes in a float and calculates the next goal point on our path
Vector4f nextGoal(float t){
  Vector4f u(sqrt(2), sqrt(2), 0, 0);
  Vector4f uxn(0, 0, -1, 0);
  Vector4f c(4, 4, 0, 1);
  return 4*cos(t)*u + 4*sin(t)*uxn + c;
}

//****************************************************
// function that does the actual drawing of stuff
//***************************************************
void myDisplay() {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);       // clear the color buffer

  glMatrixMode(GL_MODELVIEW);             // indicate we are specifying camera transformations
  //glLoadIdentity();               // make sure transformation is "zero'd"

  glColor3f(1.0f, 1.0f, 1.0f);

  glFlush();
  glutSwapBuffers();          // swap buffers (we earlier set double buffer)
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
void initGL(int argc, char *argv[]){
  // Set up OpenGL
  initGL(argc, argv);

  //This tells glut to use a double-buffered window with red, green, and blue channels 
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

  // Initialize the viewport size
  viewport.w = 700;
  viewport.h = 700;
  
  //The size and position of the window
  glutInitWindowSize(viewport.w, viewport.h);
  glutInitWindowPosition(0,0);
  glutCreateWindow(argv[0]);
  
  glLoadIdentity();

  // Set default toggles to flat shading and filled mode
  glShadeModel(GL_FLAT);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  glClearColor(0.0, 0.0, 0.0, 0.0);

  glEnable(GL_NORMALIZE);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);

  GLfloat pl[] = {1.0, 1.0, 1.0, -1.0};
  GLfloat ka[] = {0.3, 0.0, 1.0};
  GLfloat kd[] = {0.3, 0.0, 1.0};

  glLightfv(GL_LIGHT0, GL_POSITION, pl);
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ka);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, kd);
  
  glutDisplayFunc(myDisplay);       // function to run when its time to draw something
  glutReshapeFunc(myReshape);       // function to run when the window gets resized

  // Add keyboard bindings
  glutKeyboardFunc(onKeyPress);
  glutSpecialFunc(onDirectionalKeyPress);
}

//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {
  float step = 0.05;
  Vector4f goal(2, 2, 2, 1);
  
  // Lower -> higher index corresponds to base -> end of the arm
  Joint j1(2.0);
  Joint j2(2.0);
  Joint j3(1.0);
  Joint j4(1.0);
  
  vector<Joint> joints;
  
  joints.push_back(j1);
  joints.push_back(j2);
  joints.push_back(j3);
  joints.push_back(j4);
  
  // Add joints to the system
  arm.updateJoints(joints);
  
  // Run update once
  for(float t = 0; t < 1; t += step) {
    goal = nextGoal(t);
    bool finished = false;
    while(!finished) {
      finished = update(goal);
    }
  }
  
  for(float t = 0; t < 3; t += step) {
    goal = nextGoal(t);
    bool finished = false;
    while(!finished) {
      finished = update(goal);
    }
    // Todo: draw the updated system
  }

  //glutMainLoop();             // infinite loop that will keep drawing and resizing
  // and whatever else

  return 0;
}