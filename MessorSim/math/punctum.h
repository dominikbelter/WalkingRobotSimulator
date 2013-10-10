#ifndef _CPUNCTUM_H
#define _CPUNCTUM_H

#include "CMat44.h"
#include "CQuaternion.h"

class CPunctum : public CMat44, public CQuaternion
{
public:
	CPunctum(void);
	CPunctum(float * init_tab) : CMat44 (&init_tab[1])
	{
		 init_tab[0] ? foothold = true : foothold = false;
	}
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
	///export to file
	void export2file(ofstream * file);

private:
	/// foothold
	bool foothold;
};

#endif
