#include "joint_rc.h"

CJoint_RC::CJoint_RC(void)
: angle(0)
{
}

CJoint_RC::~CJoint_RC(void)
{
}

/// zwraca kat w w stawie w radianach
double CJoint_RC::getAngle(void)
{
	return angle;
}

/// zmienia wartosc kata w stawie
void CJoint_RC::setAngle(double new_angle)
{
	angle = new_angle;
}
