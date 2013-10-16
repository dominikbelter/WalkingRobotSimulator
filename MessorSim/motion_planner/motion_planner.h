#pragma once
#include "Crrt.h"
#include "../mapping/idealMap.h"
#include "../robot_controller/RPCCaller.h"
#include "../math/punctum.h"
#include "footPlanner.h"
#include "robot.h"

class CMotionPlanner: public CFootPlanner
{
public:
	CMotionPlanner(CIdealMap* map, RPCCaller* rpccaller);
	~CMotionPlanner();
	////wyciecie z zaplanowanej sciezki fragmentow wykraczajacych poza zadana odleglosc
	bool reduceTrajectory(float distance);
	////execute trajectory
	bool executeTrajectory(float speed);
	////robot porusza sie po trajektorii zadanej
	bool executeTrajectoryWave(float speed);
	///// connect operation
	int Connect(Crrt * tree, CrrtNode * q_new);
	///// rrt connect
	bool rrtConnect(float pos_end[], float rot_end[], float distance2ground, float *map_boundaries, int shift);
	///// rrt connect begin
	bool rrtConnectBegin(float pos_end[], float rot_end[], float distance2ground, float *map_boundaries, int shift);
	///// create path according to obtained tree
	void createPath(CrrtNode q_begin, CrrtNode q_finish, int shift);
	/////finds path using a-star algorithm
	int astarFind(std::vector< void* > *path, float * init_pos, float * dest_pos);
	///// create path according to obtained tree
	void createPath(CrrtNode q_begin, int shift);
	///// path smoothing
	void pathSmoothing(float alpha = 0.01, float beta = 0.99, float alpha_leg = 0.5, float beta_leg = 0.1);
	/// save trajectory to file
	void savePath2File(const char * filename);
	/// load path from file
	void loadPathFromFile(const char * filename);

	//RPCCaller
	RPCCaller* rpccaller;
public:
	FILE * fposlizg;
	/// RRT begin
	Crrt* rrt_begin;
	/// RRT finish
	Crrt* rrt_finish;
	int smart_gait_iterator;
};