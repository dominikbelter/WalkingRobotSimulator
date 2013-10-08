#pragma once

#include "../math/punctum.h"
#include <math.h>
#include "coldet.h"
#include "3dsloader.h"
#include "objects3DS.h"
#include "gl/glut.h"

//**********DEFINICJE LIST*********************
#define korpus_dol			1 
#define korpus_gora			2 
#define noga_blacha_1		3
#define noga_blacha_2		4
#define noga_czlon_1		5
#define noga_czlon_2a		6
#define podstawka			7
#define stopka				8

//*********************************************

class CRobotStructure
{
public:
	CRobotStructure(void);
public:
	~CRobotStructure(void);
	void init_structures(void);
	void struct_korpus_dol(void);
	void struct_korpus_gora(void);
	void struct_noga_blacha_1(void);
	void struct_noga_blacha_2(void);
	void struct_noga_czlon_1(void);
	void struct_noga_czlon_2a(void);
	void struct_podstawka(void);
	void struct_stopka(void);
	void CoordinateSystem_Local(void);
	void Noga_1(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga);
	void Noga_2(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga);
	void Noga_3(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga);
	void Noga_4(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga);
	void Noga_5(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga);
	void Noga_6(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga);
	void GLNoga_1(float Qn_1, float Qn_2, float Qn_3);
	void GLNoga_2(float Qn_1, float Qn_2, float Qn_3);
	void GLNoga_3(float Qn_1, float Qn_2, float Qn_3);
	void GLNoga_4(float Qn_1, float Qn_2, float Qn_3);
	void GLNoga_5(float Qn_1, float Qn_2, float Qn_3);
	void GLNoga_6(float Qn_1, float Qn_2, float Qn_3);
	CPunctum makeTransformMatrix(const char * type, float value);
	void copyTable(CPunctum * src, float * dest);
	void DrawRobot(float x, float y, float z, float alpha, float beta, float gamma, float Q1,  float Q2,  float Q3,  float Q4,  float Q5,  float Q6, 
			   float Q7,  float Q8,  float Q9,  float Q10, float Q11, float Q12, 
			   float Q13, float Q14, float Q15, float Q16, float Q17, float Q18);
	void GLDrawRobot(float x, float y, float z, float alpha, float beta, float gamma, float Q1,  float Q2,  float Q3,  float Q4,  float Q5,  float Q6, 
			   float Q7,  float Q8,  float Q9,  float Q10, float Q11, float Q12, 
			   float Q13, float Q14, float Q15, float Q16, float Q17, float Q18);
	void GLDrawRobot(float *pos, float * rot, float Q1,  float Q2,  float Q3,  float Q4,  float Q5,  float Q6, 
			   float Q7,  float Q8,  float Q9,  float Q10, float Q11, float Q12, 
			   float Q13, float Q14, float Q15, float Q16, float Q17, float Q18);
	void drawTerrain(float x, float y);
	bool checkCollision(float x, float y, float z, float alpha, float beta, float gamma, float * angles, bool * collision_table,bool check_ground, double ***map, int offset_x, int offset_y);

	//zadane katy dla serwomechanizmow
	float angles[18];
	///model 3DS
	CObjects3DS robot_model;
};
