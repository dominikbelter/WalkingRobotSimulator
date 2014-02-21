#pragma once
#include "leg.h"
#include "../math/punctum.h"
#include "../functions.h"
#include "../robot_controller/RPCCaller.h"

//const double width_max = (const double) 0.13;  ///odleglosc od srodka robota do srodkowej nogi
//const double width_min = (const double) 0.065;  ///odleglosc od osi podluznej robota do przedniej nogi
//const double lenght = (const double) 0.153289;		  ///odleglosc nog przednich od srodka robota wzdluz osi podluznej
const unsigned char MAX_SPEED = 114;  ///maksymalna predkosc serwomechanizmu (wartosc wysylana do serwa)
const float SERVO_DELAY = (const float) 0.029;  ///przerwa pomiedzy wysterowaniem tej samej nogi (czas wysterowania wszystkich nóg), wynika z wlasciwosci systemu sterowania, jezeli w kazdym cyklu odczytywany jest prad w konczynie ten czas sie wydluza
const float MAX_SERVO_SPEED = (const float) 4.1568; /// [rad/s] maksymalna predkosc serwomechanizmu przy napieciu 7.4V 

class CRobot
{
	//funkcje
public:
	CRobot::CRobot(void);
	CRobot(RPCCaller* rpccaller);
	~CRobot(void);
	// pomocnicze
	///ustawia poczatkowe wartosci katow w stawie [45, 24, -114, 0 24 -114, -45 24 -114,-45, 24, -114, 0 24 -114, 45 24 -114]
	void setInitZeroAngle();
	/// oblicza predkosci w wezlach danej nogi
	double computeLegSpeed(char leg_no, float speed, short int *leg_speed);

	//obliczenia wynikajace z kinematyki
	/// funkcja obliczajaca zmiane katów przy ruchu platformy robota, kazda noga zadaje inny ruch (w odniesieniu do pozycji neutralnej)
	bool computeRobotKinematicDeltaAngle(float * x, float * y, float * z, float * alpha, float * beta, float * gamma,float *angle);
	/// funkcja obliczajaca zmiane katów przy ruchu platformy robota (w odniesieniu do pozycji neutralnej)
	bool computeRobotKinematicDeltaAngle(float x, float y, float z, float alpha, float beta, float gamma,float *angle);
	/// funkcja obliczajaca zmiane katow przy ruchu platformy robota, kazda noga zadaje inny ruch
	bool computeRobotRelativeKinematicDeltaAngleNeutral(float * x, float * y, float * z, float * alpha, float * beta, float * gamma,float *angle,float foot_up, int legs);
	/// funkcja obliczajaca kinematyke calego robota- alpha rotX, beta rotY,gamma rotZ (wzgledem pozycji neutralnej)
	bool robotKinematic(float x, float y, float z, float alpha, float beta, float gamma,float * angle);
	/// funkcja obliczajaca kinematyke calego robota- alpha rotX, beta rotY,gamma rotZ (wzgledem pozycji neutralnej), kazda konczyna moze zadawac inny ruch
	bool robotKinematic(float * x, float * y, float * z, float * alpha, float * beta, float * gamma,float * angle);
	/// funkcja obliczajaca zmiane katow przy ruchu platformy robota
	bool computeRobotRelativeKinematicDeltaAngle(float x, float y, float z, float alpha, float beta, float gamma,float *angle);
	/// funkcja obliczajaca zmiane katów przy ruchu platformy robota, kazda noga zadaje inny ruch
	bool computeRobotRelativeKinematicDeltaAngle(float * x, float * y, float * z, float * alpha, float * beta, float * gamma,float *angle);
	/// funkcja obliczajaca kinematyke calego robota- alpha rotX, beta rotY,gamma rotZ - przesuniecia wzgledne
	bool robotRelativeKinematic(float x, float y, float z, float alpha, float beta, float gamma,float angle[]);
	/// funkcja obliczajaca kinematyke calego robota- alpha rotX, beta rotY,gamma rotZ - przesuniecia wzgledne dla kazdej konczyny osobno
	bool robotRelativeKinematic(float * x, float * y, float * z, float * alpha, float * beta, float * gamma,float * angle);
	/// przesuwa stope we wspolrzednych globalnych
	bool setFootPositionGlobal(double globalx, double globaly, double globalz, int legnumber);
	/// ustawia pozycje stopy w ukladzie konczyny
	bool setFootPositionLeg(double x,double y, double z, char leg_no, float speed);
	/// ustawia pozycje stopy w ukladzie robota
	bool setFootPositionRobot(double x,double y, double z, char leg_no, float speed);
	
	// ustawienie/odczytanie wartosci zmiennych
	/// ustawia wartosci katow we wszystkich stawach
	void setLegs(float * angles);
	/// ustawia wartosc (value) kata w nodze leg_number i zlaczu joint number
	void setAngle(unsigned char leg_number, unsigned char joint_number, double value);
	/// pobiera wartosc kata w nodze leg_number i zlaczu joint_number
	double getAngle(unsigned char leg_number, unsigned char joint_number);
	/// pobiera neutralna wartosc kata w nodze leg_number i zlaczu joint_number
	double getNeutralAngle(unsigned char leg_number, unsigned char joint_number);
	/// zmienia wartosci zadane w stawach danej konczyny, katy podawane sa w kolejnosci od korpusu
	bool changeLegAngles(unsigned char leg_no, float alpha, float beta, float gamma, float speed);

	//sterowanie, wysylanie/odbieranie wartosci zadanych, komunikacja ze sprzetem
	/// odczytuje sile w stopie z czujnika nacisku
	short int getFootForce(char leg_no);
	/// odczytuje sile w stopie bazujac na czujnnikach pradu
	void getLegForce(char leg_no, unsigned int force[3]);
	/// odczytuje prad pobierany przez silniki w stawach
	void getLegCurrent(char leg_no, float current[3]);	
	/// przeksztalca i wysyla wartosci zadane dla serwomechanizmow
	double sendAngles(float speed);
	/// odczytuje i odbiera wartosci rzeczywiste katow w stawach dla nogi leg_no
//	void readLegAngles(char leg_no);
	/// odczytuje i odbiera wartosci rzeczywiste katow w stawach
//	double readRobotAngles();
	/// odczytuje polozenie i orientacje robota
	CPunctum getRobotState();
	/// odczytuje orientacje robota (katy)
	void getRotAngles(float *angles);
	/// zwraca stan robota w postaci macierzy odpowiedzialnych z polozenie stop i korpusu
	void getFullRobotState(CPunctum * body, CPunctum * feet);

	/*sterowanie ruchem robota kroczacego*/
	//real robot initialization
	void initializeRealRobot();
	/// zmienia katy w serwomechanizmach
	bool changeAngles(double * angles, float speed);
	/// przesuwa stope o zadana odleglosc w ukladzie nogi
	bool changeFoot(float x, float y, float z, unsigned char leg_no, float speed);
	/// przesuwa stope o zadana odleglosc w ukladzie robota
	bool changeFootRobot(float x, float y, float z, unsigned char leg_no, float speed);
	/// przesuwa wszystkie stopy o zadane odleglosci w ukladzie nogi
	bool changeAllfeet(float * x, float * y, float * z, float speed);
	/// przesuwa wszystkie stopy o zadane odleglosci w ukladzie robota
	bool changeAllfeetRobot(float * x, float * y, float * z, float speed);
	/// przesuwa platforme o zadana odleglosc liniowa i katowa w aktualnym ukladzie robota
	bool changePlatformRobot(float x, float y, float z, float alpha, float beta, float gamma, float speed, int accel);
	/// przesuwa platforme o zadana odleglosc liniowa i katowa w aktualnym ukladzie robota (kazda noga moze zadawac inny kierunek)
	bool changePlatformRobot(float * x, float * y, float * z, float * alpha, float * beta, float * gamma, float speed, int accel);
	/// przesuwa platforme o zadana odleglosc liniowa i katowa w aktualnym ukladzie robota poslugujac sie nogami parzystymi lub nieparzystymi
	bool changePlatformRobotTripod(float x, float y, float z, float alpha, float beta, float gamma, int even, float speed, int accel);
	/// przesuwa platforme o zadana odleglosc liniowa i katowa wzgledem konfiguracji neutralnej
	/// uwaga nie używać naprzemiennie wersji jedno i wielowymiarowej - nie będzie dzialalo poprawnie
	/// ze wzgledu na brak znajomosci poprzedniej pozycji platformy (TODO) lub nalezy konczyc ruch w pozycji neutralnej
	bool changePlatform(float x, float y, float z, float alpha, float beta, float gamma, float speed, int accel);
	/// stabilizuje platforme na podstawie czujnikow z imu
	bool stabilizePlatform(float speed, int accel);
	/// przesuwa platforme o zadana odleglosc liniowa i katowa wzgledem konfiguracji neutralnej
	/// uwaga nie używać naprzemiennie wersji jedno i wielowymiarowej - nie będzie dzialalo poprawnie
	/// ze wzgledu na brak znajomosci poprzedniej pozycji platformy (TODO) lub nalezy konczyc ruch w pozycji neutralnej
	bool changePlatform(float * x, float * y, float * z, float * alpha, float * beta, float * gamma, float speed, int accel);
	//przemieszcza robota do zadanych pozycji w ukladzie globalnym, soft - pozwala na zmiane konfiguracji jezeli pozycja jest niemozliwa do wykonania
	bool move2GlobalPosition(CPunctum body_prev, CPunctum body, CPunctum * feet_prev, CPunctum * feet, float speed, int soft = 0);
	/// sprawdza czy pozycja stopy jest osiagalna
	bool isFootPositionAvailableGlobal(CPunctum robot_pos,double globalx, double globaly, double globalz, int legnumber, float scale = 1.0);
	///modifies global robot position
	void modifyRobotPosition(float x, float y, float z, float alpha, float beta, float gamma);
	/// stawia stopy na podlodze - opuszcza do momentu uzyskania kontaktu
	bool Placefeet(float dz, int legs, float speed);
	/// stawia stopy na podlodze - opuszcza do momentu uzyskania kontaktu
	bool Placefeet(float dz, float speed);
	///check stability
	bool checkStability(CPunctum body, CPunctum * feet, float stability_margin);
	//check stability
	bool isStable(CPunctum body, CPunctum * feet);
	///computes stability margin
	float computeStabilityMargin(CPunctum body, CPunctum * feet);
	///move body closer to stability center
	void increaseStabilityMargin(CPunctum * body, CPunctum * feet, float distance);
	///move body closer to stability center
	void increaseStabilityMarginSwing(CPunctum * body, CPunctum * feet, float distance);
	///computes kinematic margin
	float computeKinematicMargin(CPunctum body, CPunctum * feet);
	///computes kinematic margin
	float computeKinematicMarginApprox(CPunctum *body, CPunctum *feet);
	///computes kinematic margin for single leg
	float computeKinematicMarginApprox(CPunctum *body, CPunctum *foot, int leg_no);
	/// check collisions
	bool checkCollisions(CPunctum body, CPunctum *feet);

	///ODE functions
//	Simulate ODE for x miliseconds
	void sleepODE(int miliseconds);

private:
	/// wspolrzedne srodka platformy w ukladzie globalnyn
	CPunctum centre; //TG:koniecznie prywatne
	/// robot's position in global coordinates
	double robot_position[3]; //
	/// robot's orientation in global coordinates
	double robot_orientation[3]; //
	/// sterowanie sprzetem (reprezentacja sprzetu znajdujacego sie na robocie)
//	CBoard board;
	/// poprzdnia pozycja robota zadana przy użyciu kinematyki liczonej wzgledem pozycji neutralnej
	float robot_platform_pos[6];
	/// poprzednia wartosc zadana przy kinematyce liczonej wzgledem pozycji neutralnej
	float x_ref_prev[6];
	/// poprzednia wartosc zadana przy kinematyce liczonej wzgledem pozycji neutralnej
	float y_ref_prev[6];
	/// poprzednia wartosc zadana przy kinematyce liczonej wzgledem pozycji neutralnej
	float z_ref_prev[6];
	/// poprzednia wartosc zadana przy kinematyce liczonej wzgledem pozycji neutralnej
	float alpha_ref_prev[6];
	/// poprzednia wartosc zadana przy kinematyce liczonej wzgledem pozycji neutralnej
	float beta_ref_prev[6];
	/// poprzednia wartosc zadana przy kinematyce liczonej wzgledem pozycji neutralnej
	float gamma_ref_prev[6];
	/// maksymalna predkosc [0-1] 
	float speed;
	//wysyłanie poleceń do robota
	RPCCaller* rpccaller;
	
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
	CLeg leg[6];
	/// stan platformy z akcelerometru - os x
	double accel_x;
	/// stan platformy z akcelerometru - os y
	double accel_y;
	/// obsluga zyroskopu - kat
	double gyro_angle;
	/// obsluga zyroskopu - temperatura
	double gyro_temp;
	/// obsluga zyroskopu - napiecie referencyjne
	double gyro_reference;
	/// position of the robot
	CPunctum robot_global_pos;
};
