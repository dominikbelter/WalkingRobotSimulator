#include "IMU.h"

CIMU::CIMU(void)
{
}

CIMU::~CIMU(void)
{
}

///ustawia poczatkowa pozycje i orientacje robota
void CIMU::setInitialPosition(float x, float y, float z, float alpha, float beta, float gamma){
	pos[0]=x; pos[1]=y; pos[2]=z;
	rot[0]=alpha; rot[1]=beta; rot[2]=gamma;
	for (int i=0;i<3;i++) {
		dpos[i]=0; ddpos[i]=0;
		drot[i]=0; ddrot[i]=0;
	}
}

///ustawia id ciala, ktorego pozycja bedzie mierzona
void CIMU::setIMUBody(dBodyID body_id, dGeomID geom_id){
	body = body_id;
	geom = geom_id;
}

///ustawia krok symulacji - potrzebne przy obliczaniu predkosci i przyspieszen
void CIMU::setDT(float DT){
	dt = DT;
}

///odczytuje pozycje robota
void CIMU::getIMUposition(float position[]) {
	measure();
	for (int i=0;i<3;i++)
		position[i]=pos[i];
}

///odczytuje predkosc robota
void CIMU::getIMUdposition(float speed[]) const{
	for (int i=0;i<3;i++)
		speed[i]=dpos[i];
}

///odczytuje przyspieszenie robota
void CIMU::getIMUddposition(float accel[]) const {
	for (int i=0;i<3;i++)
		accel[i]=ddpos[i];
}

///odczytuje orientacje robota
void CIMU::getIMUorientation(float orientation[]) const {
	for (int i=0;i<3;i++)
		orientation[i]=rot[i];
}

///odczytuje predkosc katowa robota
void CIMU::getIMUdorientation(float dorientation[]) const {
	for (int i=0;i<3;i++)
		dorientation[i]=drot[i];
}

///odczytuje przyspieszenie katowe robota
void CIMU::getIMUddorientation(float ddorientation[]) const {
	for (int i=0;i<3;i++)
		ddorientation[i]=ddrot[i];
}

///pomiar pozycji i orientacji
void CIMU::measure(void){
	//odczyt pozycji
	dReal Dpos[3];
    const dReal* position = Dpos;
	if ((unsigned int)body!=0xcdcdcdcd){
		position = dBodyGetPosition(body);
		float prev_value[3],prev_value1[3];//poprzednia wartosc

		for(int i=0;i<3;i++) {//zapisujemy poprzednia pozycje robota
			prev_value[i]=pos[i];
			prev_value1[i]=dpos[i];
		}

		pos[0]=position[0];
		pos[1]=-position[2];
		pos[2]=position[1];
		
		for(int i=0;i<3;i++)
		{
			dpos[i]=(pos[i]-prev_value[i])/dt;
			ddpos[i]=(dpos[i]-prev_value1[i])/dt;
		}

		/// odczyt orientacji
		const dReal *R;
		R = dGeomGetRotation(geom);
		dReal alpha,beta,gamma;
		get_euler(R,alpha,beta,gamma);

		for(int i=0;i<3;i++) {//zapisujemy poprzednia orientacje
			prev_value[i]=rot[i];
			prev_value1[i]=drot[i];
		}

		rot[0]=alpha;
		rot[1]=beta;
		rot[2]=gamma;

		for(int i=0;i<3;i++)
		{
			drot[i]=(rot[i]-prev_value[i])/dt;
			ddrot[i]=(drot[i]-prev_value1[i])/dt;
		}
	}
}


/// odczytuje katy Eulera na podstawie macierzy rotacji
void CIMU::get_euler(const dReal * matrix,dReal &kx,dReal &ky,dReal &kz) const {
  float Element[9];
  Element[0]  = matrix[0]; Element[1]  = matrix[8]; Element[2]  = -matrix[4]; 
  Element[3]  = matrix[2]; Element[4]  = matrix[10]; Element[5]  = matrix[6]; 
  Element[6]  = -matrix[9]; Element[7]  = matrix[1]; Element[8] = matrix[5];

   const dReal epsilon=0.0005;
   if((matrix[2] < (1-epsilon)) && (matrix[2] > (-1+epsilon)))
   {
	   kx= -atan2(Element[5],Element[8]);
	   kz= -atan2(Element[1],Element[0]);
	   ky= atan2(Element[2],-sin(kx)*Element[5]+cos(kx)*Element[8]);
   } else {//osobliwosc!!!
	  if (matrix[2]<0)
		kz=-1.57;
	  else
	    kz=1.57;
      kx= -atan2(Element[5],Element[8]);
	  ky=atan2(Element[2],-sin(kx)*Element[5]+cos(kx)*Element[8]);
   }
}

void CIMU::QuaternionToEuler(const dReal * matrix,dReal &alpha,dReal &beta,dReal &gamma){
   dQuaternion quaternion;
  dRtoQ (matrix,quaternion);
  dReal w,x,y,z;
  w=quaternion[0];
  x=quaternion[1];
  y=quaternion[2];
  z=quaternion[3];
  double sqw = w*w;    
  double sqx = x*x;    
  double sqy = y*y;    
  double sqz = z*z; 
  
  alpha = atan2(2.0 * (x*y + z*w),(sqx - sqy - sqz + sqw));
  beta = atan2(2.0 * (y*z + x*w),(-sqx - sqy + sqz + sqw));  
  gamma = asin(-2.0 * (x*z - y*w));
}
