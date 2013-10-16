#include "../mapping/idealMap.h"
#include "robot.h"
#include "../robot_controller/RPCCaller.h"

#pragma once
#define MAX_Z_ROT 0.2
#define MAX_X_ROT 0.05
#define MAX_X_TR 0.04
#define MAX_Y_TR 0.04

class CPathPlanner
{
public:
	CPathPlanner(void);
	~CPathPlanner(void);

	bool EvaluateLineTr(float x, float y, float z, float alpha, float beta, float gamma, int num_steps, int step_dividor);
	/// Simulate ODE for x miliseconds
	void SleepODE(int miliseconds);
	/// wyznacza sekwencje przestawien stop na podstawie "zapasu" kinematycznego
	void computeSequence(CPunctum * feet_start, CPunctum * body_start, CPunctum * body_finish, int num_iter, int * sequence);
	/// sprawdza czy punkty lezace na sciezce stopy sa wewnatrz przestrzenii roboczej
	bool verifyAchievability(CPunctum body, CPunctum foot, int leg_no);
	//sprawdza czy punkty lezace na sciezce stopy sa wewnatrz przestrzenii roboczej
	bool verifyAchievability(CPunctum body, CPunctum * feet, float scale=1.0);
	//sprawdza czy punkty lezace na sciezce stopy sa wewnatrz przestrzenii roboczej
	bool verifyAchievability(CPunctum body, CPunctum foot1, CPunctum foot2, CPunctum foot3, CPunctum foot4, CPunctum foot5, CPunctum foot6, float scale=1.0);
	/// rrt connect
	//bool rrtConnect(float pos_end[], float rot_end[], float distance2ground);

	/// trajectory of the robot platform
	CPositionRecorder robot_platform_traj;
	RPCCaller* rpccaller;
	CRobot* robot;

};
