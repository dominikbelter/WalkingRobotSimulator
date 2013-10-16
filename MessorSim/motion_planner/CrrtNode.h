#pragma once
#include "../math/punctum.h"
#include "../functions.h"
#include <gl\glut.h> // glut.h includes gl.h and glu.h

class CrrtNode
{
public:
	CrrtNode(void);
	CrrtNode(float x, float y, float z, float alpha, float beta, float gamma);
	CrrtNode(float * position, float * orientation);
public:
	~CrrtNode(void);
	///set node index
	void SetNodeIndex(int index);
	///get node index
	int GetNodeIndex();
	///set parent index
	void SetParentIndex(int index);
	///get parent index
	int GetParentIndex();
	/// rand ronfiguration
	void randomConfig(float min_x, float max_x, float min_y, float max_y);
	/// draw node
	void drawNode(double red, double green, double blue, double thickness, const char marker);

public:
	/// position of the robot
	float pos[3];
	/// orientation of the robot
	float rot[3];
	/// Robots body position and orientation
	CPunctum body;
	/// feet positions
	CPunctum feet[6];//tablica zawierajaca pozycje stop w danej konfiguracji
	/// odd or even feet while tripod gait
	//bool even;
	/// feet trajectory from actual to next foothold
	CPunctum legs_traj[6][30];
	/// body_traj
	CPunctum body_traj[40];
	/// gait type 0 - free gait, 3-tripod gait, 5 - wave gait
	int gait_type;

private:
	///index of the parent
	int parent_index;
	///index of the node
	int node_index;
};
