#ifndef _STARLETHSTRUCTURE_H_
#define _STARLETHSTRUCTURE_H_

#include "RobotStructure.h"

using namespace robsim;

namespace robsim {
	/// create a single robot structure (StarlETH)
	RobotStructure* createStarlETHRobotStructure(void);
}

class StarlETHStructure : public RobotStructure {
public:
	//**********OpenGL Call Lists*********************
	static const uint_fast8_t GL_PLATFORM = 1;
	static const uint_fast8_t GL_HIP	  = 2;
	static const uint_fast8_t GL_THIGH	  = 3;
	static const uint_fast8_t GL_SHANK    = 4;
	//*********************************************

	enum MechParts
	{
		PLATFORM, //0
		HIP1, HIP2, HIP3, HIP4, //1,2,3,4
		THIGH1, THIGH2, THIGH3, THIGH4, //5,6,7,8
		SHANK1, SHANK2, SHANK3, SHANK4, // 9,10,11,12
	};

	/// Pointer
	typedef std::unique_ptr<StarlETHStructure> Ptr;

	/// Default constructor
	StarlETHStructure(void);
	~StarlETHStructure(void);

	/// Draw robot using openGL
	void GLDrawRobot(robsim::float_type *pos, robsim::float_type * rot, std::vector<robsim::float_type> config) const;

	/// Check collisions
	bool checkCollision(robsim::float_type* pos, robsim::float_type* rot, robsim::float_type * angles, bool * collision_table) const;

private:
	/// Initialize robot structure
	void initStructures(void);
	/// initialize collision model
	void initCollisionModel(uint_fast8_t objectNo, CollisionModel3D& model);
	/// initialize collision models
	void CollisionModels(void);
	/// initialize GL lists
	void structPlatform(void);
	void structLegHip(void);
	void structLegThigh(void);
	void structLegShank(void);
	void drawCoordinateSystem(void);
	void Leg1(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const;
	void Leg2(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const;
	void Leg3(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const;
	void Leg4(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const;
	void GLLeg1(float Qn_1, float Qn_2, float Qn_3) const;
	void GLLeg2(float Qn_1, float Qn_2, float Qn_3) const;
	void GLLeg3(float Qn_1, float Qn_2, float Qn_3) const;
	void GLLeg4(float Qn_1, float Qn_2, float Qn_3) const;
	void copyTable(CPunctum * src, float * dest) const;
	void DrawRobot(robsim::float_type* pos, robsim::float_type* rot, robsim::float_type * angles) const;

	std::vector<CollisionModel3D*> meshModel;
	//zadane katy dla serwomechanizmow
//	float angles[12];
	///model 3DS
	CObjects3DS robot_model;
};

#endif _STARLETHSTRUCTURE_H_