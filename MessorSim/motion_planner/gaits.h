/*
* gait generation for a hexapod walking robot
* Author: Dominik Belter 
* Poznan University of Technology
* institute of Control and Information Engineering
* www: www.walkingrobots.pl
* created: 04.2007, last updated: 05.2009
* If you find this code useful please cite:
* - D. Belter, System sterowania ruchem sześcionożnego robota kroczącego (Parameterized gait generation system for hexapod walking robot), Krajowa Konferencja Robotyki, pp. September 3-6 2008 (in Polish)
* - D. Belter, Parametryzowane wzorce ruchu dla sześcionożnych robotów kroczących, Studia z automatyki i informatyki, tom 33, pp. 45-58, 2008
*/

#pragma once
#include "robot.h"
#include "footPlanner.h"
#include "Crrt.h"

#define PI 3.14159265

class CGaits : public CFootPlanner
{
public:
	CGaits(void);
	~CGaits(void);
	/// ustawia poczatkowa konfiguracje neutralna dla robota (0, 24, -114)
	void setInitZeroAngle();
	/// ustawia konczyne w pozycji neutralnej, offset_up - wysokosc podnoszenia stop
	bool zeroPosition(unsigned char leg_no, float offset_up, float speed);
	/// set neutral position of the robot according to the zero_angle table
	bool SetNeutralPosition();
	/// ustawia konczyny w pozycji neutralnej
	bool setZeroPosition(float offset_up, float speed);
	// steps - liczba krokow
	// delay_up - przerwa po uniesieniu nogi [us]
	// delay_forward - przerwa po przeniesieniu nogi do przodu/tylu [us]
	// delay_steps - przerwa czasowa pomiedzy krokami [us]
	// offset_up - wysokosc na jaka sa podnoszone nogi
	//x,y,z,alpha,beta,gamma - zadane translacje i rotacje
	/// prepare to tripod gait - set neutral position of legs and move platform to init level
	bool tripodPrepare(unsigned int delay_up, unsigned int delay_forward, float x, float y, float z, float alpha, float beta, float gamma, float offset_up, float speed);
	/// tripod step
	bool tripodStep(unsigned int delay_up, unsigned int delay_forward, float x, float y, float z, float alpha, float beta, float gamma, float offset_up, float speed, int accel);
	/// short tripod step  - cycle is shorter:triangular trajectory of the leg's tip
	bool shortTripodStep(unsigned int delay_up, unsigned int delay_forward, float x, float y, float z, float alpha, float beta, float gamma, float offset_up, float speed, int accel);
	/// finish tripod gait - set neutral position of legs and move platform to init level
	bool tripodFinish(unsigned int delay_up, unsigned int delay_forward, float x, float y, float z, float alpha, float beta, float gamma, float offset_up, float speed);
	/// prepare to wave gait - set neutral position of legs and move platform to init level
	bool wavePrepare(unsigned int delay_up, unsigned int delay_forward, float x, float y, float z, float alpha, float beta, float gamma, float offset_up, float speed, int accel);
	/// wave step
	bool waveStep(unsigned int delay_up, unsigned int delay_forward, float x, float y, float z, float alpha, float beta, float gamma, float offset_up, float speed, int accel);
	/// short wave step - cycle is shorter:triangular trajectory of the leg's tip
	bool shortWaveStep(unsigned int delay_up, unsigned int delay_forward, float x, float y, float z, float alpha, float beta, float gamma, float offset_up, float speed, int accel);
    /// prepare to smart gait
	bool SmartGait(float x, float y, float z, float rotx, float roty, float rotz, float foot_up, float speed, int accel);
	/// prepare to smart gait
	bool SmartGaitFoothold(float x, float y, float z, float rotx, float roty, float rotz, float foot_up, float speed, int accel);
	//wyciecie z zaplanowanej sciezki fragmentow wykraczajacych poza zadana odleglosc
	bool reduceTrajectory(float distance);
	//robot porusza sie po trajektorii zadanej
	bool executeTrajectory(float speed);
	//robot porusza sie po trajektorii zadanej
	bool executeTrajectoryWave(float speed);
	/// connect operation
	int Connect(Crrt * tree, CrrtNode * q_new);
	/// rrt connect
	bool rrtConnect(float pos_end[], float rot_end[], float distance2ground, float *map_boundaries, int shift);
	/// rrt connect begin
	bool rrtConnectBegin(float pos_end[], float rot_end[], float distance2ground, float *map_boundaries, int shift);
	/// create path according to obtained tree
	void createPath(CrrtNode q_begin, CrrtNode q_finish, int shift);
	///finds path using a-star algorithm
	int astarFind(std::vector< void* > *path, float * init_pos, float * dest_pos);
	/// create path according to obtained tree
	void createPath(CrrtNode q_begin, int shift);
	/// path smoothing
	void pathSmoothing(float alpha = 0.01, float beta = 0.99, float alpha_leg = 0.5, float beta_leg = 0.1);

	//zmienne
	/// konfiguracja neutralna robota
	float zero_angle[18];
	/// robot
	//CRobot robot;
	/// smart gait iterator
	int smart_gait_iterator;
	FILE * fposlizg;
	/// RRT begin
	Crrt rrt_begin;
	/// RRT finish
	Crrt rrt_finish;
};
