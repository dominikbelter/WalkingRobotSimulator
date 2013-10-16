#pragma once
//#include "stdlib.h"
//#include <gl\glut.h> // glut.h includes gl.h and glu.h
#include <ode/ode.h>    // ode library header
#include "Ground.h"
#include "OdeGeom.h"
#include "ODErobot.h"
#include "../resource.h"

//#include "../openGL/openGL1Dlg.h"
#include "../position_recorder/PositionRecorder.h"

class COdeWorld
{
public:
	COdeWorld(int sizex, int sizey);
public:
	~COdeWorld(void);	
	void SimStep();	/// krok symulacji z ODE
	void InitODE(double dt, bool ct, int sizex, int sizey); /// inicjalizacja ODE
	void CloseODE(); /// zamkniecie ODE
	void DrawGeom(dGeomID g, const dReal *pos, const dReal *R, char corp = 0, char leg = 0); /// rysuje prymitywy na scenie
	void DrawObjects(); /// rysuje wszystkie figury na scenie
	void checkFootContatcs(void);
	void COdeWorld::showRobotPosition(); /// wyswietla pozycje robota
	int foot_groundx[6];//wybrany punkt na podparcie stopy (os x)
	int foot_groundy[6];//wybrany punkt na podparcie stopy (os y)
	int foot_groundx_def[6];//wybrany punkt na podparcie stopy (os x) - domyslny
	int foot_groundy_def[6];//wybrany punkt na podparcie stopy (os y) - domyslny

	int tx;
	int ty;
	//int time;  //kopniecie robota
	// zmienne
	// dynamics and collision objects
	/// collision space
	dSpaceID Space;           
	/// contact group for the new joint
	dJointGroupID jointgroup;   
	/// the joint ID
	dJointID Joint;
	/// robot position recorder
	CPositionRecorder rec_robot_platform;
	/// robot position recorder
	CPositionRecorder rec_robot_orientation;
	/// robot position recorder (3 leg)
	CPositionRecorder rec_robot_leg[6];

	// setings
	/// show_geoms - show robot shape
	bool show_geoms;

	///ziemia
	CGround * ground; 
	/// indexy punktow terenu wg kolejnosci wyswietlania
	int * indexes; 
	/// tablica wpolrzednych trojkatow terenu
	dVector3 * triVert;
	/// dane do przechowywania powierzchni
	dTriMeshDataID triMesh; 

	/// ground in ode
	dGeomID plane; 
	/// klasa rysujaca obiekty na scenie
	COdeGeom geometry; 
	/// robot
	CODERobot robotODE;
	///krok symulacji [s]
	double stepDT;
	///czas symulacji [s]
	double actual_time;
	/// wskaznik do glownej formatki - do wyswietlania stanu robota
//	CopenGL1Dlg * main_dialog;
	//double* COdeWorld::CalculateGlobalCoordinates(double x, double y);
		/// przechowuje indeksy puntkow do zapalenia
	/// trajectory of the robot platform
//	CPositionRecorder * robot_platform_traj;
//	CPositionRecorder * legs_traj[6];
	CPunctum body;
	CPunctum feet[6];
	bool draw_path;
};