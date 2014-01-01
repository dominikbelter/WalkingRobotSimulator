#include "openGLview.h"
#include <gl\glut.h> // glut.h includes gl.h and glu.h
#include "light.h"

int GL_LIGHT[8];    // Represent GL lights in a conveniant array

openGLview::openGLview(void)
{
}

openGLview::openGLview(COdeWorld* dynamicWorld, CRobotStructure* robot_structure, CMotionPlanner* motion_planner, CLocalMap* local_map)
{
	// Lights
	GL_LIGHT[0] = GL_LIGHT0; GL_LIGHT[1] = GL_LIGHT1; GL_LIGHT[2] = GL_LIGHT2; GL_LIGHT[3] = GL_LIGHT3;    // Represent GL lights in a conveniant array
	GL_LIGHT[4] = GL_LIGHT4; GL_LIGHT[5] = GL_LIGHT5; GL_LIGHT[6] = GL_LIGHT6; GL_LIGHT[7] = GL_LIGHT7;    // Represent GL lights in a conveniant array
	currentLight = 0;
	numLights = 1;
	this->dynamicWorld=dynamicWorld; //swiat symulujacy dynamike i kolizje pomiedzy obiektami
	this->robot_structure=robot_structure;
	this->motion_planner=motion_planner;
	this->local_map=local_map;
}

openGLview::~openGLview(void)
{ //usuwamy dynamicznie tworzone obiekty
	delete [] light;
}

void openGLview::display(int value) {
	glutSetWindow(GLwin_id);
	glClearColor(0,0,0,0);//ustawiamy kolor t³a na czarny 
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //czyœcimy bufory
	glEnable(GL_DEPTH_TEST); //uruchamiamy bufor g³ebi
	
   for (int i=0; i <= numLights; i++)//swiatelka
    {
        float swiatlo_pozycja[] = { i*7, 4.5+i, -7+9*i, 1 };
        float swiatlo_kolor[] = { 1, 1, 1, 1 } ;
        glLightfv( GL_LIGHT[i], GL_COLOR, swiatlo_kolor);
        glLightfv( GL_LIGHT[i], GL_POSITION, swiatlo_pozycja);
        glEnable(GL_LIGHT[i]);
    }
	rysuj_figury(); //testowe rysowanie figur
	glutPostRedisplay();
	glutSwapBuffers();
	dynamicWorld->showRobotPosition();
}

void openGLview:: resize(int width, int height) {
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION); // GL_MODELVIEW
	glLoadIdentity();
	gluPerspective(1.0, (float)width / (float)height, 0.1, 1000.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(  // ustawienie pocz¹tkowe kamery
	0.0, 0.0, 250.0, // eye location
	0.0, 0.0, 0.0, // center location
	0.0, 1.0, 0.0); // up vector
}

void openGLview::keyboard(unsigned char key, int x, int y) // metoda w zale¿noœci od wcisniêtego przycisku wywo³uje odpowiednie zdarzenia
{
	if (key == 27) {
		exit(0); // escape key is ascii 27
	}
	if (key == 97) { //key am
	}
	if (key == 115) { //key s
	}
	if (key == 100) { //key d
	}
	if (key == 122) { //key z
	}
	if (key == 120) { //key x
	}
	if (key == 99) { //key c
	}
	if (key == 102) { //key f
	}
	if (key == 103) { //key g
	}
	if (key == 104) { //key h
	}
	if (key == 118) { //key v
	}
	if (key == 98) { //key b
	}
	if (key == 110) { //key n
	}
	if (key == 111) { //key o
	}
	if (key == 112) { //key o
	}
}

void openGLview::animacja(void) 
{//funkcjê odpowiedzialna za zmiane pozycji i orientacji wszystkich figur na scenie
}

void openGLview::rysuj_figury(void) 
{
	//rysowanie obiektów ODE
	dynamicWorld->DrawObjects();

	std::vector<robsim::float_type> Q_ref(18,0);
	dynamicWorld->robotODE->readAngles(Q_ref);
	robsim::float_type position[3];
	dynamicWorld->robotODE->getPosition(position);
	robsim::float_type orientation[3];
	dynamicWorld->robotODE->getRPY(orientation);
	const dReal * pos;
	pos = dGeomGetPosition(dynamicWorld->robotODE->getGeomId(0));//pobiera pozycje obiektu
	const dReal * R;
    R = dGeomGetRotation(dynamicWorld->robotODE->getGeomId(0)); //pobiera orientacje obiektu

	float p[3];
	float r[12];
	for (int i=0;i<11;i++)
		r[i]=(float)R[i];
	for (int i=0;i<3;i++)
		p[i]=(float)pos[i]*10;

	//rysowanie robota
	//if (!dynamicWorld->show_geoms)
		//robot_structure->GLDrawRobot(p,r,Q_ref[0],Q_ref[1],Q_ref[2],Q_ref[3],Q_ref[4],Q_ref[5],Q_ref[6],Q_ref[7],Q_ref[8],Q_ref[9],Q_ref[10],Q_ref[11],Q_ref[12],Q_ref[13],Q_ref[14],Q_ref[15],Q_ref[16],Q_ref[17]);

	//rysowanie zaplanowanych trajektorii platformy i nóg robota
	if (dynamicWorld->draw_path)
	{
		if(!motion_planner->robot_platform_traj.plot_stop) //je¿eli rysowanie dozwolone (œcie¿ka nie jest zapisywana do pliku)
			motion_planner->robot_platform_traj.plot(0.2,0.8,0.9,3,'o');
		for (int i=0;i<dynamicWorld->robotODE->getLegsNo();i++)
		{
			if(!motion_planner->legs_traj[i].plot_stop) 
				motion_planner->legs_traj[i].plot(0.7,0.9,0.8,3,'o');
		}
	}
	//motion_planner->rpccaller->control->robot_rc->show_center_of_mass();
	//rysowanie drzew planowania ruchu RRT
	motion_planner->rrt_begin->drawTree(0.2,0.9,0.2,3,'o');
	motion_planner->rrt_finish->drawTree(0.9,0.2,0.2,3,'o');

	//rysowanie mapy terenu budowanej przez robota
	//local_map->DrawLocalMap();
}

void openGLview::drawText(int x, int y, char *string)
{
	int i, len;

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, 300, 0, 300);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glDisable(GL_DEPTH_TEST);

	glRasterPos2f(x, y);
	for (i = 0, len = (int)strlen(string); i < len; i++) {
		glutBitmapCharacter(GLUT_BITMAP_8_BY_13, (int)string[i]);
	}

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glEnable(GL_DEPTH_TEST);
}