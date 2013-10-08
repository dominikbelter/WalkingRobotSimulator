#ifndef _CPUNCTUM_H
#define _CPUNCTUM_H

#include "CMat44.h"
#include "CQuaternion.h"

class CPunctum : public CMat44, public CQuaternion
{
public:
	CPunctum(void);
	~CPunctum(void);

	CPunctum operator= (CMat44 param);
	/// convert matrix to quaternion
    void mat2quat(void);
    /// convert quaternion to matrix
	void quat2mat(void);
    /// create matrix according to given quaternion and translation vectors
    void createMatrixQuatPos(double* _quat, double * _pos);
	// set point as a foothold
	void setFoothold(bool is_foothold);
	// is foothold?
	bool isFoothold(void);
	/// ODE to OpenGL conversion
	void ODEtoOGL(const float* p, const float* R, float * matGL);

private:
	/// foothold
	bool foothold;
};

#endif
