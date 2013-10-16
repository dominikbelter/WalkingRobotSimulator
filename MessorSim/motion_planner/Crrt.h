#pragma once
#include "footPlanner.h"
#include "CrrtNode.h"
#include "../mapping/idealMap.h"
#include "../robot_controller/RPCCaller.h"
#include <iostream>
#include <vector>
#include <time.h>
#include "robot.h"

using namespace std;
#include "micropather.h"
using namespace micropather;

struct SAnode {
	float x;
	float y;
};

struct SParticle {
	float pos[6];
	float p_change[6];
	float p_best[6];
	float fitness;
	CPunctum feet[6];
};

class Crrt : public CFootPlanner, public Graph
{
public:
	Crrt(CIdealMap* map, RPCCaller* rpccaller, CRobot* robot);
	Crrt(float x, float y, float z, float alpha, float beta, float gamma, CIdealMap* map, RPCCaller* rpccaller, CRobot* robot);
	Crrt(float * position, float * orientation, CIdealMap* map, RPCCaller* rpccaller, CRobot* robot);
public:
	~Crrt(void);
	/// set root node
	void setRoot(float * position, float * orientation, CPunctum body, CPunctum * feet);
	///add node
	void AddNode(int parent, CrrtNode *new_node);
	///get node
	CrrtNode getNode(int node_no);
	/// gets number of nodes
	int GetNodeNo();
	/// sets number of nodes
	void SetNodeNo(int _node_no);
	/// erase vector
	void erase();
	/// finds the nearest neighbour
	CrrtNode NearestNeighbour(CrrtNode test_node);
	/// computes distance between nodes
	double dist(CrrtNode node1, CrrtNode node2);
	///EXTEND
	int Extend(CrrtNode, CrrtNode *q_new, bool reverse, float distance2ground, int shift, bool can_connect, float *robot_pos);
	///EXTEND with optimization
	int ExtendOpt(CrrtNode, CrrtNode *q_new, bool reverse, float distance2ground, int shift, bool can_connect, float *robot_pos);
	///EXTEND FREE Gait
	int ExtendFreeGait(CrrtNode, CrrtNode *q_new, bool reverse, float distance2ground, int shift, bool can_connect, float *robot_pos);
	///EXTEND simple
	int ExtendSimple(CrrtNode, CrrtNode *q_new, bool reverse, float distance2ground, int shift, bool can_connect);
	/// initialize parameters
	void initializeParameters();
	/// compute orientation according to terrain shape
	void computeOrientationAndHeight(float x, float y, float * height, float * rot_xy, float distance2ground);
	/// get full body state
	bool createRobotState(float pos_end[], float rot_end[], CPunctum * body, CPunctum *feet);
	/// get full body state Foothold
	bool createRobotStateFoothold(float pos_end[], float rot_end[], CPunctum * body, CPunctum *feet);
	/// draw tree
	void drawTree(double red, double green, double blue, double thickness, const char marker);
	/// check if the change of the robots position is possible
	int checkPassing(CPositionRecorder * feet_traj, CPunctum * body_traj, CPunctum *body_start, CPunctum *body_finish, CPunctum *feet_start, CPunctum * feet_finish, float distance2ground, int shift);
	/// check if the change of the robots position is possible
	int checkPassingOpt(CPositionRecorder * feet_traj, CPunctum * body_traj, CPunctum *body_start, CPunctum *body_finish, CPunctum *feet_start, CPunctum * feet_finish, float distance2ground, int shift);
	/// check if the change of the robots position is possible - wave gait
	int checkPassingWave(CPositionRecorder * feet_traj, CPunctum * body_traj, CPunctum *body_start, CPunctum *body_finish, CPunctum *feet_start, CPunctum * feet_finish, float distance2ground, int shift);
	/// check if the change of the robots position is possible - wave gait
	int checkPassingFree(CPositionRecorder * feet_traj, CPunctum * body_traj, CPunctum *body_start, CPunctum *body_finish, CPunctum *feet_start, CPunctum * feet_finish, float distance2ground, int shift);
	/// save to file
	void save2file(const char * filename);
	void NodeToXY( void* node, int* x, int* y );
	void* XYToNode( int x, int y );
	virtual float LeastCostEstimate( void* nodeStart, void* nodeEnd );
	virtual void AdjacentCost( void* node, std::vector< StateCost > *neighbors );
	virtual void PrintStateInfo( void* node );
	///finds path using a-star algorithm
	int astarFind(std::vector< void* > *path, float * init_pos, float * dest_pos);
	/// save A* to file
	void saveAstar2file(std::vector< void* > *path, const char * filename);
	/// optimize posture
	bool optimizePosture(CPunctum *body_init, CPunctum * feet_init, CPunctum *body_opt);
	/// optimize posture - approx
	bool optimizePostureApprox(CPunctum *body_init, CPunctum * feet_init, CPunctum *body_opt, bool only_stance=false);
	/// optimize posture - swing
	bool optimizePostureSwingApprox(CPunctum *body_init, CPunctum * feet_init, CPunctum *body_opt);
	/// optimize swing leg using gradient method
	bool optimizeSwingGrad(CPunctum body_init, CPunctum * foot_init, int leg_no);
	/// Compute optimal posture - approx
	bool computeOptimalPosture(CPunctum *body_init, CPunctum * feet_init, CPunctum *body_opt);
	///does body collide with ground?
	bool collisionsWithGround(CPunctum *current_body);
	///does body collide with ground?
	bool fastCollisionsWithGround(CPunctum * current_body);
	/// get footholds
	bool createFootholds(CPunctum * body, CPunctum *feet);

	float rrt_goalx[2];
	float rrt_goaly[2];
	float rrt_goalz[2];

private:
	/// tree is stroed in vector
	vector<CrrtNode> tree;
	/// the number of nodes in tree
	int node_no;
	/// max rotation along z axis
	float max_z_rotation;
	MicroPather* pather;
};
