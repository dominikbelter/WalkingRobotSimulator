#pragma once
#include "joint.h"
#include "foot.h"
#include "../math/punctum.h"

#define PI 3.14159265
const double segment_1=0.055; //lenght pierwszego segmentu
const double segment_2=0.16;  //lenght drugiego segmentu
const double segment_3=0.230;  //lenght trzeciego segmentu
const double offset_3=0.0;	//przsuniecie d w ostatnim jointie
enum segment {T1,T2,T3,T4};


class CLeg
{
public:
	CLeg(double x,double y, double z,double alpha,double beta,double gamma);
	CLeg(void);
	~CLeg(void);
	/// funkcja ustawiajaca nogi dla stanu poczatkowego
	void reset(double x,double y, double z,double alpha,double beta,double gamma);
	/// ustawia wartosci katow w stawach
	void setAngle(unsigned char nr_joint,double new_angle);
	/// zwraca kat z wstawie nr_joint
	double getAngle(unsigned char nr_joint);
	/// zwraca poprzedni kat z wstawie nr_joint
	double getPreviousAngle(unsigned char nr_joint);
	/// ustawia aktualna(poprzednia jezeli ustawiamy nowa) wartosc kata w stawie
	double setPreviousAngle(unsigned char nr_joint, float value);
	/// ustawia stan stopy
	void setFoot(bool state);
	/// podaje pozycje danej stopy
	CPunctum getPosition(int part);
	///podaje pozycje neutralna danej stopy
	CPunctum getNeutralPosition(int part);
	/// pobiera stan stycznika
	bool getContact(void);
	/// funkcja obliczajaca kinematyke prosta nogi
	CPunctum forward_kinematic(double theta1, double theta2, double theta3,int part );
	/// zadanie kinematyki odwrotnej
	bool inverse_kinematic(double x, double y, double z,int part, double angles[], float scale = 1.0);
	/// kinematyka odwrotna dla nogi
	bool inverse_kinematic(double x, double y, double z,int part, float angles[], float scale = 1.0);
	/// przesuwa stope o zadana odleglosc w ukladzie nogi
	bool moveFoot(double x, double y, double z, int part);
	/// przesuwa stope we wspolrzednych nogi
	bool setFootPosition(double x, double y, double z, int part);
	/// obliczenie zmiany kata w stawach przy przesunieciu stopy o zadana odleglosc w ukladzie nogi
	bool computeMoveDeltaAngle(double x, double y, double z, int part, float * delta_angle);
	/// obliczenie zmiany kata w stawach przy zmianie wartosci zadanych
	void computeMoveDeltaAngle(double alpha, double beta, double gamma, float * delta_angle);
	/// sprawdza czy pozycja stopy jest osiagalna
	bool isFootPositionAvailableLocal(double x,double y, double z,int part,int up=0, float scale=1.0);
	/// oblicza srodek masy konczyny w ukladzie konczyny 
	CPunctum computeCenterOfMass(float * angles, int part);
	/// compute kinematic margin for given legs position
	float computeKinematicMargin(double alpha,double beta, double gamma,int part,int up);
	/// compute kinematic margin for given legs position
	float computeKinematicMarginApprox(double alpha,double beta, double gamma,int part,int up);
	/// compute kinematic margin for given legs position
	float computeKinematicMarginPos(double x,double y, double z,int part,int up);
	/// computes delta_z to maximize kinematic margin
	float maximizeKinematicMarginPos(double x,double y, double z,int part,int up);

	//zmienne
private:
	/// tablica z kolejnymi katami w stawach nogi robota (joint[0] - kat polozony najblizej korpusu robota)
	CJoint joint[3];
	/// tablica z poprzednimi katami w stawach nogi robota (joint[0] - kat polozony najblizej korpusu robota)
	CJoint prev_joint[3];
	/// stan stopy w danej nodze
	CFoot foot;
public:
	/// macierz przeksztalcenia srodek robota -> poczatek nogi
	CPunctum start;
	/// tablica z neutralnymi katami w stawach nogi robota (joint[0] - kat polozony najblizej korpusu robota)
	float zero_angle[3];
};
