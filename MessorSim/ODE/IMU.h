#pragma once
#include <ode/ode.h>

#define PI 3.14159265

class CIMU
{
public:
	CIMU(void);
public:
	~CIMU(void);
	///ustawia poczatkowa pozycje i orientacje robota
	void setInitialPosition(float x, float y, float z, float alpha=0, float beta=0, float gamma=0);
	///ustawia id ciala, ktorego pozycja bedzie mierzona
	void setIMUBody(dBodyID body_id, dGeomID geom_id);
	///ustawia krok symulacji - potrzebne przy obliczaniu predkosci i przyspieszen
	void setDT(float DT);
	///odczytuje pozycje robota
	void getIMUposition(float position[]);
	///odczytuje predkosc robota
	void getIMUdposition(float speed[]) const;
	///odczytuje przyspieszenie robota
	void getIMUddposition(float accel[]) const;
	///odczytuje orientacje robota
	void getIMUorientation(float orientation[]) const;
	///odczytuje predkosc katowa robota
	void getIMUdorientation(float dorientation[]) const;
	///odczytuje przyspieszenie katowe robota
	void getIMUddorientation(float ddorientation[]) const;
	///pomiar pozycji i orientacji
	void measure(void);
	/// ODE odczytuje katy Eulera na podstawie macierzy rotacji
	void get_euler(const dReal * matrix,dReal &kx,dReal &ky,dReal &kz) const;
	/// ODE wyznacza wartosc katow eulera na podstawie kwaterionow
	void QuaternionToEuler(const dReal * matrix,dReal &alpha,dReal &beta,dReal &gamma);

private:
	//zmienne
	///pozycja robota
	float pos[3];
	///predkosc liniowa 
	float dpos[3];
	///przyspieszenie liniowe
	float ddpos[3];
	///orientacja robota
	float rot[3];
	///predkosc obrotowa
	float drot[3];
	///przyspieszenie katowe
	float ddrot[3];
	/// id korpusu
	dBodyID body;
	/// id geometrii korpusu
	dGeomID geom;
	/// krok symulacji
	float dt;

};
