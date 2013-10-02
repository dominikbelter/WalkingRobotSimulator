#pragma once
#include "../math/punctum.h"

class CJoint
{
public:
	CJoint(void);
	~CJoint(void);
	/// zwraca kat w stawie w radianach
	double getAngle(void);
	/// zmienia wartosc kata w stawie
	void setAngle(double new_angle);

	//zmienne
private:
	/// kat w stawie w radianach
	double angle;
	/// pozycja stawu w postaci macierzy jednorodnej
	CPunctum joint_position;
};