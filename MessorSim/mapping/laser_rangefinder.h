#pragma once
#include "raytracing.h"
#include "idealMap.h"
#include <gl/glut.h>
#include <vector>

using namespace std;

class CRangefinder
{
public:
	CRangefinder(void);
	~CRangefinder(void);
	virtual void rangefinding(){};
	CIdealMap* map;
};

//czujnik Hokuyo
class CHokuyoRangefinder:CRangefinder
{
public:
	CHokuyoRangefinder(CIdealMap* map);
	~CHokuyoRangefinder(void);
	//pomiar odleg³oœci punktu od robota
	void rangefinding(float min, float max, float* robot_position, float* robot_orientation, vector<long>* data, vector<float>* kat_radiany);
public:
	//po³o¿enie sensora w uk³adzie robota
	float sensor_position[3];
	//rotacja sensora w uk³adzie robota
	float sensor_orientation[3];
	//zasiêg czujnika
	int range;
	//rozdzielczoœæ dokonywania pomiarów
	float resolution;
};