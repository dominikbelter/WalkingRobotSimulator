#include "joint.h"

CJoint::CJoint(void)
: angle(0)
{
}

CJoint::~CJoint(void)
{
}

/// zwraca kat w w stawie w radianach
double CJoint::getAngle(void)
{
	return angle;
}

/// zmienia wartosc kata w stawie
void CJoint::setAngle(double new_angle)
{
	angle = new_angle;
}
