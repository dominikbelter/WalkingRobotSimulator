#pragma once

#include "pathPlanner.h"
#include "../mapping/idealMap.h"
#include "../robot_controller/RPCCaller.h"
#include "robot.h"

class CFootPlanner: public CPathPlanner
{
public:
	CFootPlanner(void);
	~CFootPlanner(void);

	// int num_points - liczba punktow mapy wycinanej wokol stopy
	void createFootQualityMatrix(int num_points);
	/// oblicz wspolczynniki jakosci size - rozmiar wycietej mapy, x,y wpolrzedne srodka mapy
	void ComputeKcoef(int size, int x, int y);
	/// generate foothold trajectory
	void generateFootholdTrajSimple(void);
	/// generate foothold trajectory
	bool generateFootholdTraj(int shift);
	bool selectFoothold(CPunctum robot_pos, int x, int y, int legnumber, float *pos);
	// float distance - odleglosc nad terenem
	//iter_no - liczba punktow trajektorii
	bool generateFootTraj(CPositionRecorder *feet_traj, CPunctum foot_start, CPunctum foot_finish,CPunctum body_start, CPunctum body_finish,float distance,int leg_no, int iter_no);
	//wygladzanie trajektorii
	void smooth(float *points_x, float *points_y, float * points_z, int size);
	/// check traverse - simpliefied
	bool checkTraverse(float * init_pos, float * dest_pos);

	//offset wynikajacy z przesuniecia stopy przy "madrym" kroczeniu
	int raster_x[6];
	int raster_y[6];
	float ** K1;//odleglosc od punktu domyslnego
	float ** K2;//wykrywanie maksimum i minimum
	float ** K3;//wielkosc zmian nachylenia podloza
	float ** K4;//zgodnosc z kierunkiem ruchu robota
	float ** K5;//reliability
	int search_size; //liczba punktow mapy wycinanej wokol stopy
	float desired_legx[6];
	float desired_legy[6];
	float desired_legz[6];
	/// trajectory of the robot' legs
	CPositionRecorder legs_traj[6];
	CIdealMap* map;
};
