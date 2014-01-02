#ifndef _MESSORSTRUCTURE_H_
#define _MESSORSTRUCTURE_H_

#include "RobotStructure.h"

using namespace robsim;

//**********OpenGL Call Lists*********************
#define GL_PLATFORM_BOTTOM	1 
#define GL_PLATFORM_TOP		2 
#define GL_LEG_SHEET1		3
#define GL_LEG_SHEET2		4
#define GL_LEG_PART1		5
#define GL_LEG_PART2		6
#define GL_BASE				7
#define GL_FOOT				8

//*********************************************

namespace robsim {
	/// create a single robot structure (Messor)
	RobotStructure* createMessorRobotStructure(void);
}

class MessorStructure : public RobotStructure {
public:
	enum MechParts
	{
		PLATFORM_BOTTOM, PLATFORM_TOP, //0,1
		FOOT1, FOOT2, FOOT3, FOOT4, FOOT5, FOOT6,//2,3,4,5,6,7
		SEGMENT_I1, SEGMENT_I2, SEGMENT_I3, SEGMENT_I4, SEGMENT_I5, SEGMENT_I6,//8,9,10,11,12,13
		SEGMENT_II1, SEGMENT_II2, SEGMENT_II3, SEGMENT_II4, SEGMENT_II5, SEGMENT_II6,//14,15,16,17,18,19
		LINK_C1, LINK_C2, LINK_C3, LINK_C4, LINK_C5, LINK_C6//20,21,22,23,24,25,26
	};

	/// Pointer
	typedef std::unique_ptr<MessorStructure> Ptr;

	/// Default constructor
	MessorStructure(void);
	~MessorStructure(void);

	/// Draw robot using openGL
	void GLDrawRobot(robsim::float_type *pos, robsim::float_type * rot, std::vector<robsim::float_type> config) const;

	/// Check collisions
	bool checkCollision(robsim::float_type* pos, robsim::float_type* rot, robsim::float_type * angles, bool * collision_table) const;

private:
	/// Initialize robot structure
	void init_structures(void);
	/// initialize collision model
	void initCollisionModel(uint_fast8_t objectNo, CollisionModel3D& model);
	/// initialize collision models
	void CollisionModels(void);
	/// initialize GL lists
	void structPlatformBottom(void);
	void structPlatformTop(void);
	void structLegSheet1(void);
	void structLegSheet2(void);
	void structLegPart1(void);
	void structLegPart2(void);
	void structBase(void);
	void structFoot(void);
	void drawCoordinateSystem(void);
	void Leg1(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const;
	void Leg2(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const;
	void Leg3(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const;
	void Leg4(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const;
	void Leg5(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const;
	void Leg6(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const;
	void GLLeg1(float Qn_1, float Qn_2, float Qn_3) const;
	void GLLeg2(float Qn_1, float Qn_2, float Qn_3) const;
	void GLLeg3(float Qn_1, float Qn_2, float Qn_3) const;
	void GLLeg4(float Qn_1, float Qn_2, float Qn_3) const;
	void GLLeg5(float Qn_1, float Qn_2, float Qn_3) const;
	void GLLeg6(float Qn_1, float Qn_2, float Qn_3) const;
	void copyTable(CPunctum * src, float * dest) const;
	void DrawRobot(robsim::float_type* pos, robsim::float_type* rot, robsim::float_type * angles) const;

	std::vector<CollisionModel3D*> meshModel;
	//zadane katy dla serwomechanizmow
	float angles[18];
	///model 3DS
	CObjects3DS robot_model;
};

#endif _MESSORSTRUCTURE_H_