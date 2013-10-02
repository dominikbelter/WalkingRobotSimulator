#pragma once
#include "../math/punctum.h"
#include <ode/ode.h>
#include "../functions.h"
#include "IMU.h"

// some constants
#define DENSITY (0.5)        // density of all objects
#define GEOMSPERBODY 1       // maximum number of geometries per body
#define MAX_CONTACTS 2048      // maximum number of contact points per body

typedef struct MyObject //struktura reprezentujaca obiekty
{
	dBodyID Body;                     // the dynamics body
	dGeomID Geom[GEOMSPERBODY];       // geometries representing this body
} MyObject;

#define PI 3.14159265
const double width_max = 0.13;  ///odleglosc od srodka robota do srodkowej nogi
const double width_min = 0.065;  ///odleglosc od osi podluznej robota do przedniej nogi
const double lenght = 0.153289;		  ///odleglosc nog przednich od srodka robota wzdluz osi podluznej 
//parametry nogi zmiany wymagaja rowniez parametry w leg.h
const double segment1=0.055; //lenght pierwszego segmentu
const double segment2=0.16;  //lenght drugiego segmentu
const double segment3=0.230;  //lenght trzeciego segmentu
const double foot_length=0.02;  //dlugosc stopy
const double offset3=0.0;	//przsuniecie d w ostatnim jointie
const double max_servo_speed=12000.1568;	//przsuniecie d w ostatnim jointie

class CODERobot
{
	//funkcje
public:
	CODERobot(void);
	~CODERobot(void);
	///ustawia poczatkowa pozycje i orientacje robota
	void setInitialPosition(float x, float y, float z, float alpha=0, float beta=0, float gamma=0);
	/// ustawia wartosci katow we wszystkich stawach
	void setLegs(int a, int b, int c, int d, int e, int f, int g, int h, int i, int j, int k, int l, int m, int n, int o, int p, int r, int s);
	/// ustawia predkosc zadana dla konczyny
	void setLegSpeed(int leg_no, short * leg_speed);
	/// ustawia wartosci zadane W STOPNIACH!!!
	void setLeg(int leg_no, float * leg_deg);
	/// odczytuje rzeczywiste wartosci katów w stawach
	void readLegPosition(int leg_no, short * read_angles);

	/*-----*/
//////////ODE////////////////
	/// definicja robota w srodowisku ODE
	void ODEcreateRobot(dWorldID World, dSpaceID Space, dJointGroupID jointgroup);
	/// ODE - symulacja regulatora w serwomechanizmie
	void setServo(int servo_nr, double value);
	/// ODE - ustawia wartosci katow we wszystkich stawach
	void setAllServos();
	/// ODE ustawia tablice z pozycja i orientacja robota na podstawie stanu robota w ODE
	void setPositionSensors();
	/// ODE pobiera pozycje stopy numeracja stop 1 do 6
	void getFootPosition(int foot, dReal position[]);
	/// odczytuje polozenie i orientacjê robota
	CPunctum getRobotState();
	/// odczytuje orientacje robota (k¹ty)
	void getRotAngles(float *angles);
	/// odczytuje wartosci zadane dla serwomechanizmow
	float getAngle(int leg, int joint);
	/// ODE - pobiera rzeczywiste wartosci katow we wszystkich stawach
	void getRealServoValues(float angles[]);
	/// odczytuje stan przycisku w stopie
	bool getContact(int leg);

	//zmienne
private:
	/// wspolrzedne srodka platformy w ukladzie globalnyn
	CPunctum centre; //TG:koniecznie prywatne
	/// wartoœci zadane 
	float ref_angles[18];
	/// predkosci zadane
	float ref_speed[18];

public:
	/** nogi robota (leg[0] - pierwsza noga)
                /\		
	           /||\
                ||
	      6 o ----- o 1 
	       /  o   o  \
	      /     |     \
	   5 o     \__/    o 2
          \           /
	       \         / 
	      4 o ----- o 3
	
	*/
	/// ODE - tablica obiektow, z ktorych sklada sie robot
	MyObject Object[25];
	/// ODE - tablica zlacz
	dJointID Joints[24];
	//IMU
	CIMU imu;
	/// styczniki
	char contact[6];
	char filtered_contact[6]; 
};
