#include "openGLmain.h"

openGLview* gl;

//#ifdef _DEBUG
//#define new DEBUG_NEW
//#endif

	//okreslenie swiatel
	static GLfloat light_ambient[]  = { 0.0, 0.0, 0.0, 1.0 };
	static GLfloat light_diffuse[]  = { 1.0, 1.0, 1.0, 1.0 };
	static GLfloat light_specular[] = { 0.0, 0.0, 0.0, 1.0 };
	static GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };

	//okreslenie powierzchni
	static GLfloat mat_ambient[]    = { 0.7, 0.7, 0.7, 1.0 };
	static GLfloat mat_diffuse[]    = { 0.8, 0.8, 0.8, 1.0 };
	static GLfloat mat_specular[]   = { 0.1, 0.1, 0.1, 1.0 };
	static GLfloat high_shininess[] = { 0.0 };

	char* filename = "ground11.bmp";
	int tx;
	int ty;

	//RPCCaller* rpccaller;

void animacja(int value)
{
//	gl.animacja();
//	glutTimerFunc(40,animacja,0); //ustawiamy funkcje timera
}

void display(void)
{
	gl->display(0);
}

void resize(int w, int h)
{
	gl->resize(w, h);
}

void keyboard(unsigned char key, int x, int y)
{
	gl->keyboard(key, x, y);
}

void pick(GLint name) // wybrany zostal, ktorys z elementow na scenie
{
//	gl.selected_id=name;//przepisujemy do klasy opisujacej scene numer wybranego obiektu
}

void openGLinit(COdeWorld* dynamicWorld, RobotStructure* robot_structure, CMotionPlanner* motion_planner, CLocalMap* local_map)
{
	gl=new openGLview(dynamicWorld, robot_structure, motion_planner, local_map);
	// TODO: Add extra initialization here
	srand ( (unsigned)time(NULL) ); // zainicjowanie losowosci
	// TODO: Add your control notification handler code here
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);//ustawienie kolorow, precyzji i glebi
	glutInitWindowSize (800, 600);// ustawiamy poczatkowe wielkosc okna
	glutInitWindowPosition (200, 150);// poczatkowa pozycja okna (lewy gorny rog)
	int GLwin_id = glutCreateWindow("MessorSim");//tworzymy nowe okno
	gl->GLwin_id = GLwin_id;
	glViewport(0, 0, 500,500);
	// set ustawienie odpowiednich funkcji, ktore bade wywolywane przy odpowiednich zdarzeniach
	glutDisplayFunc(display);//ustawiamy funkcje display
	glutReshapeFunc(resize);//ustawiamy funkcje resize
	glutKeyboardFunc(keyboard); // ustawiamy funkcje obslugujaca klawiature
	//glutTimerFunc(30,animacja,0); //ustawiamy funkcje timera
	//this->ShowWindow(SW_HIDE);// ukrywamy glowne okno

	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT1, GL_POSITION, light_position);

    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);

	glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);

	zprInit();						// inicjalizacja obslugi myszki
    zprSelectionFunc(display);     // wskazujemy funkcje rysujaca obiekty na scenie
    zprPickFunc(pick);            // wskazujemy funkcje obslugujaca zdarzenie wybrania obiektu myszka

	robot_structure->initStructures();
	dynamicWorld->ground->initializeList(dynamicWorld->indexes,dynamicWorld->triVert);

}