#include <stdlib.h>
#include <gl\glut.h> // glut.h includes gl.h and glu.h
#include <math.h>
//#include "RgbImage.h"
#include "openGLview.h"
#include "zpr.h"
//#include "../motion_planner/RPCCaller.h"
#include "../functions.h"

void openGLinit(COdeWorld* dynamicWorld, CRobotStructure* robot_structure, CMotionPlanner* motion_planner, CLocalMap* local_map);
void animacja(int value);
void display(void);
void resize(int w, int h);
void keyboard(unsigned char key, int x, int y);
void pick(GLint name);