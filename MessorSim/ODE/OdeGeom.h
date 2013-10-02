#pragma once
#include "stdlib.h"
#include <gl\glut.h> // glut.h includes gl.h and glu.h
#include <ode/ode.h>
#include "../math/matrix.h"
#include "../math/punctum.h"//biblioteka obslugujaca macierze 4x4;
#define PI 3.14159265

class COdeGeom
{
public:
	COdeGeom(void);
public:
	~COdeGeom(void);
	void DrawSphere(float radius, const float *pos, const float *R);//rysuje sfere
	void DrawCappedCylinder(const float * pos, const float *R, float radius, float length);//rysuje pigule
	void DrawBox(dReal * sides, const float * pos, const float * R);//rysuje szescian
	void DrawKorpus(dReal * sides, const float * pos, const float * R); //rysuje korpus robota
	void drawCapsule (float l, float r,int quality); // rysuje kapsule
	void DrawCCylinderBox(float radius,float dlugosc, const float* pos, const float* R); //rysuje kapsu³e - pigu³e + wiêksze kule, cz³ony nogi
	void COdeGeom::DrawCCylinder(float radius,float dlugosc, const float* pos, const float* R);///rysuje kapsu³e - pigu³e
	void COdeGeom::DrawCoordinateSystem(float rotx,float roty,float rotz,float posx, float posy, float posz);
	void COdeGeom::DrawCoordinateSystem(CPunctum m);
	void COdeGeom::DrawCoordinateSystem();
	///rysuje walec
	void COdeGeom::DrawCylinder(const float * pos, const float *R, float base, float top, float length);
	MATRIX GeomMatrix; //macierz do przeksztalcen
};
