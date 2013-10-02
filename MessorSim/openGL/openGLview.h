#pragma once
#define NOMINMAX
#include <windows.h>
#include "light.h"
#include "../ODE/OdeWorld.h"
#include "../collision_detection/RobotStructure.h"
#include "../motion_planner/motion_planner.h"
#include "../mapping/localMap.h"

class openGLview
{
public:
	openGLview(void);
	openGLview(COdeWorld* dynamicWorld, CRobotStructure* robot_structure, CMotionPlanner* motion_planner, CLocalMap* local_map);
public:
	~openGLview(void);
	void display(int value);
	void resize(int width, int height);
	void rysuj_figury(void);
	void animacja(void);
	void keyboard(unsigned char key, int x, int y);
	void drawText(int x, int y, char *string);

	COdeWorld * dynamicWorld; //swiat symulujacy dynamike i kolizje pomiedzy obiektami
	CRobotStructure * robot_structure; //fizyczna struktura robota
	CMotionPlanner * motion_planner; //modu³ planowania ruchu
	CLocalMap* local_map; //mapa tworzona przez robota

	// Lights
	int currentLight;
	int numLights;

	// Lighting
	LIGHT * light;
	float lightColor[3];
	int GLwin_id;
	/// draw trees
	bool draw_trees;
};
