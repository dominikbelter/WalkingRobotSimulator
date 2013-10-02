// Matrix Class    by Alan Baylis 2001

#ifndef _Matrix_H
#define _Matrix_H

#include "vector.h"
#include "CQuaternion.h"

class MATRIX
{
    public:
        MATRIX();
        ~MATRIX();

        void LoadIdentity();
        void ODEtoOGL(const float* p, const float* R);
        void CopyMatrix(float m[16]);
        void MultMatrix(float m[16]);
        void MatrixInverse(); 
        void MatrixFromAxisAngle(VECTOR axis, float theta);
		void QuatToMatrix(CQuaternion quat); 

        float Element[16];
};

#endif

