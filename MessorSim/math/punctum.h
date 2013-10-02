#pragma once
//#include "matrix_robot.h"
#include "CMat44.h"
#include "CQuaternion.h"

class CPunctum : public CMat44, public CQuaternion
{
public:
	CPunctum(void);
	~CPunctum(void);

	CPunctum operator= (CMat44 param);
	/*CPunctum operator * (CPunctum);
	CPunctum operator + (CPunctum);
	CPunctum operator - (CPunctum);
	CPunctum inv(CPunctum param);*/
	/// convert matrix to quaternion
    void mat2quat(void);
    /// convert quaternion to matrix
	void quat2mat(void);
    /// create matrix according to given quaternion and translation vectors
    void createMatrixQuatPos(double* _quat, double * _pos);
	// ustaw jako punkt podparcia
	void setFoothold(bool is_foothold);
	// czy jest punktem podparcia
	bool isFoothold(void);
private:
	/// czy punkt jest punktem podparcia
	bool foothold;
};
