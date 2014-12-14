
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <stdio.h>
#include <Eigen/Dense>

#include <glut-3.7.6-bin/GL/glut.h>
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

//****************************************************
// Global Variables
//****************************************************
Viewport	viewport;

//****************************************************
// Simple init function
//****************************************************
void initScene(){
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
}

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

//****************************************************
// function that does the actual drawing of stuff
//***************************************************
void myDisplay() {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);				// clear the color buffer

  glMatrixMode(GL_MODELVIEW);			        // indicate we are specifying camera transformations
  //glLoadIdentity();				        // make sure transformation is "zero'd"

  glColor3f(1.0f, 1.0f, 1.0f);

  glFlush();
  glutSwapBuffers();					// swap buffers (we earlier set double buffer)
}

//****************************************************
// Keyboard bindings
//***************************************************

void onKeyPress(unsigned char key, int x, int y) {
  switch(key) {
  }
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
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {
  //This initializes glut
  glutInit(&argc, argv);

  //This tells glut to use a double-buffered window with red, green, and blue channels 
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

  // Initalize theviewport size
  viewport.w = 700;
  viewport.h = 700;
  
  //The size and position of the window
  glutInitWindowSize(viewport.w, viewport.h);
  glutInitWindowPosition(0,0);
  glutCreateWindow(argv[0]);

  initScene();							// quick function to set up scene

  glutDisplayFunc(myDisplay);				// function to run when its time to draw something
  glutReshapeFunc(myReshape);				// function to run when the window gets resized

  // Add keyboard bindings
  glutKeyboardFunc(onKeyPress);
  glutSpecialFunc(onDirectionalKeyPress);

  glutMainLoop();							// infinite loop that will keep drawing and resizing
  // and whatever else

  return 0;
}