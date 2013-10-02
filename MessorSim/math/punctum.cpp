#include "punctum.h"
#include "math.h"
#include "../functions.h"

CPunctum::CPunctum(void)
{
  setZero();
  for (int i=0;i<3;i++) orientation[i]=0;
  foothold=false;
}

CPunctum::~CPunctum(void)
{
}


CPunctum CPunctum::operator= (CMat44 param){
    memcpy(this->pos, param.pos, sizeof(param.pos));
    memcpy(this->rotation, param.rotation, sizeof(param.rotation));
    mat2quat();
    return *this;
}

/*CPunctum CPunctum::operator* (CPunctum param) {
	CPunctum temp;
	temp.position=position * param.position;
	return temp;
}
CPunctum CPunctum::operator+ (CPunctum param) {
  CPunctum temp;
  temp.position=position + param.position;
	return temp;
}
CPunctum CPunctum::operator- (CPunctum param) {
  CPunctum temp;
  temp.position=position - param.position;
	return temp;
}

CPunctum CPunctum::inv(CPunctum param) {
	CPunctum temp;
	temp.position = temp.position.inv();
	return temp;
}*/

void CPunctum::mat2quat(){
    q1 = sqrt( dmax( 0, 1 + m11 + m22 + m33 ) ) / 2;
    q2 = sqrt( dmax( 0, 1 + m11 - m22 - m33 ) ) / 2;
    q3 = sqrt( dmax( 0, 1 - m11 + m22 - m33 ) ) / 2;
    q4 = sqrt( dmax( 0, 1 - m11 - m22 + m33 ) ) / 2;

    q2 = copySign( q2, m32 - m23 );
    q3 = copySign( q3, m13 - m31 );
    q4 = copySign( q4, m21 - m12 );
    if (((m11<-0.9)&&(m22<-0.99)&&(m33>0.99))||((m11<-0.9)&&fabs(m22-m33)<0.1))
        q4=1;
}

void CPunctum::quat2mat(){
    m11 = w*w + x*x - y*y - z*z;
    m12 = 2*x*y - 2*w*z;
    m13 = 2*x*z + 2*w*y;

    m21 = 2*x*y + 2*w*z;
    m22 = w*w - x*x + y*y - z*z;
    m23 = 2*y*z - 2*w*x;

    m31 = 2*x*z - 2*w*y;
    m32 = 2*y*z + 2*w*x;
    m33 = w*w - x*x - y*y + z*z;
}

/// create matrix according to given quaternion and translation vectors
void CPunctum::createMatrixQuatPos(double* _quat, double * _pos){
    memcpy(quat, _quat,sizeof(quat));
    memcpy(pos,_pos,sizeof(pos));
    quat2mat();
}

// ustaw jako punkt podparcia
void CPunctum::setFoothold(bool is_foothold){
	foothold=is_foothold;
}

// czy jest punktem podparcia
bool CPunctum::isFoothold(void){
	return foothold;
}