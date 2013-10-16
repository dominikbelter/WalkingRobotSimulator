/// Wywoływanie zdalnych procedur na serwerze

/*
* primary authors: Tomasz Blajek, Jakub Frankowski, Dominik Belter
This file is part of the TCPRobotControl library.
In general, you do not need to modify it.
*/

#ifndef RPCCallerH
#define RPCCallerH

#include "gaits_rc.h"
#include "../collision_detection/RobotStructure.h"
#include "../mapping/localMap.h"

typedef signed char ERR;
/*
This is the client-side implementation of the RPC mechanism.
This class overrides all the remote procedures declared in RPCInterface
as remote calls to the server. As a result, an instance of this class
behaves just like the request handler currently used on the server side.

Note that a one-time to Run() is necessary before calling any of the
remote procedures. See the ClientTest file for an example of use.
This class has also the method Stop() that causes the server to close. 
*/
class RPCCaller
{

  public:
    RPCCaller(const char* host,int port,COdeWorld* dynamicWorld,CRobotStructure* robot_struct, CLocalMap* local_map);
    ~RPCCaller();

  public:
    ERR Run();

  public:
    ///  1. ruch platformy wzgledem pozycji neutralnej
    ERR movePlatform(float x,float y,float z,float alpha,float beta,float gamma, float speed, int _accel);
    ///  2. ruch platformy niezaleznie od konfiguracji konczyn
    ERR movePlatformRobot(float x,float y,float z,float alpha,float beta,float gamma, float speed, int _accel);
    ///  3. pozycja stopy w ukladzie konczyny
    ERR setFootInLeg(char leg,float x,float y,float z, float speed);
    ///  4. pozycja stopy w ukladzie robota
    ERR setFootInRobot(char leg,float x,float y,float z, float speed);
    ///  5. pozycja stopy w ukladzie globalnym
//     ERR setFootInGlobal(char leg,float x,float y,float z);
    ///  6. ruch stopy w ukladzie konczyny
    ERR moveFootInLeg(char leg,float dx,float dy,float dz, float speed);
    ///  7. ruch stopy w ukladzie robota
    ERR moveFootInRobot(char leg,float dx,float dy,float dz, float speed);
    //  8.
//     ERR moveFootInGlobal(char leg,float dx,float dy,float dz);
    ///  9.ustawia katy zadane w konczynie
    ERR setLeg(char leg,float alpha,float beta,float gamma, float speed);
    /// 10. odczytuje stan czujnika nacisku w stopie
    ERR readFootForce(char leg,float res[1]);
    /// 11.odczytuje katy w stawach
    ERR readLegState(char leg,float res[3]);
    /// 12.odczytuje katy w stawach dla dwoch konczyn jednoczesnie
    ERR readLegState(char leg1,char leg2,float res[6]);
    /// 13. odczytuje wartosci pradow pobieranych przez sewromechanizmy
    ERR readLegCurrent(char leg,float res[3]);
    /// 14. odczytuje wartosci pradow pobieranych przez sewromechanizmy(dla dwoch konczyn jednoczesnie)
    ERR readLegCurrent(char leg1,char leg2,float res[6]);
    /// 15. odczytuje stan czujnikow nacisku w stopach robota
    ERR readLegForces(float res[6]);
    /// 16. odczytuje sile kontaktu stopy na podloze obliczana na podstawie czujnikow pradu
    ERR readLegForceInLeg(char leg,float res[3]);
    /// 17. odczytuje sile kontaktu stopy na podloze obliczana na podstawie czujnikow pradu (dla dwoch konczyn jednoczesnie)
    ERR readLegForceInLeg(char leg1,char leg2,float res[6]);
    /// 18. odczytuje sile kontaktu stopy na podloze obliczana na podstawie czujnikow pradu (wartosc w ukladzie robota)
    ERR readLegForceInRobot(char leg,float res[3]);
    /// 19. odczytuje sile kontaktu stopy na podloze obliczana na podstawie czujnikow pradu (wartosc w ukladzie robota, dla dwoch konczyn jednoczesnie)
    ERR readLegForceInRobot(char leg1,char leg2,float res[6]);
    /// 20. odczytuje sile kontaktu stopy na podloze obliczana na podstawie czujnikow pradu (wartosc w ukladzie globalnym)
    ERR readLegForceInGlobal(char leg,float res[3]);
    /// 21. odczytuje sile kontaktu stopy na podloze obliczana na podstawie czujnikow pradu (wartosc w ukladzie globalnym, dla dwoch konczyn jednoczesnie)
    ERR readLegForceInGlobal(char leg1,char leg2,float res[6]);
    /// 22. odczytuje pozycje robota mierzona przez IMU
    ERR readPositionIMU(float res[3]);
    /// 23. resetuje pomiar IMU - ustawia wartosci zero
    ERR resetIMU();
    /// 24. odczytuje stan substancji niebezpiecznych
    ERR readSensors(float res[6]);
    /// 25. krok ruchem trojpodporowym
    ERR tripodStep(float x,float y,float z,float alpha,float beta,float gamma, float speed, int _accel);
    /// 26. krok ruchem pieciopodporowym
    ERR waveStep(float x,float y,float z,float alpha,float beta,float gamma, float speed, int _accel);
    /// 27. przygotowanie do ruchu trojpodporowego
    ERR tripodStepPrepare(float x,float y,float z,float alpha,float beta,float gamma, float speed);
    /// 28. zakonczenie ruchu trojpodporowego
    ERR tripodStepFinish(float x,float y,float z,float alpha,float beta,float gamma, float speed);
    /// 29. przygotowanie do ruchu pieciopodporowego
    ERR wavePrepare(float x,float y,float z,float alpha,float beta,float gamma, float speed);
    /// 30. zakonczenie ruchu pieciopodporowego
    ERR waveFinish(float x,float y,float z,float alpha,float beta,float gamma, float speed);
    /// 31. krok ruchem trojpodporowym o trajektorii trojkatnej w fazie przenoszenia
    ERR tripodTriangleStep(float x,float y,float z,float alpha,float beta,float gamma, float speed, int _accel);
    /// 32. krok ruchem pieciopodporowym o trajektorii trojkatnej w fazie przenoszenia
    ERR waveTriangleStep(float x,float y,float z,float alpha,float beta,float gamma, float speed, int _accel);
    /// 33. ruch platformy wzgledem pozycji neutralnej (kazda noga zadaje inny ruch)
    /// funkcja rozpoczyna ruch, wczesniej trzeba wyslac wartosci zadane przy uzyciu funkcji setReferenceValues
    ERR movePlatformComplex(float * x, float * y, float * z, float * alpha, float * beta, float * gamma, float speed, int accel);
    /// 34. ruch platformy niezaleznie od konfiguracji konczyn (kazda noga zadaje inny ruch)
    /// funkcja rozpoczyna ruch, wczesniej trzeba wyslac wartosci zadane przy uzyciu funkcji setReferenceValues
    ERR movePlatformRobotComplex(float * x, float * y, float * z, float * alpha, float * beta, float * gamma, float speed, int accel);
	///funkcje obsługujace biblioteke movement (chod omnidirectional)
	/// 35. odswiezenie nastaw czasu kolejnych faz ruchu 
	ERR refresh_time(float res[4]);
	/// 36. odswiezenie nastaw wysokosci podnoszenia nogi podczas chodu
	ERR refresh_offset_up(float offset_up);
	/// 37. odswiezenie dowolnego z parametrow ruchu param_t {d_plat,d_robot,d_legx,d_legy,d_leg_up};
	ERR refresh(float param_num, float n_1,float n_2,float n_3,float n_4,float n_5,float n_6);
	/// 38. ustawienie tablicy chodu, count - licznik wyslanych res[6], musi byc szesc;
	ERR set_gait(float count,float n_1,float n_2,float n_3,float n_4,float n_5,float n_6);	
	/// 39. chod omni
	ERR omni_gait(float size);	


	// 40.
	ERR getIMUorientation(float* angles);
	// 41.
	ERR Placefeet(float dz, int legs, float speed);
	// 42.
	ERR Contact(int i);
	// 43.
	ERR checkCollisions(CPunctum body, CPunctum* feet);
	// 44.
	ERR move2GlobalPosition(CPunctum body_prev, CPunctum body, CPunctum * feet_prev, CPunctum * feet, float speed, int soft);
	// 45.
	ERR stabilizeRobot(void);

    
    /// funkcje pomocnicze
    /// 101. wartosci zadane dla konczyny przy generowaniu ruchu platformy robota
    ERR setReferenceValues(float x,float y,float z,float alpha,float beta,float gamma, int flag);
    /// 102. ruch platformy wzgledem pozycji neutralnej (kazda noga zadaje inny ruch)
    /// funkcja rozpoczyna ruch, wczesniej trzeba wyslac wartosci zadane przy uzyciu funkcji setReferenceValues
    ERR movePlatformComplexStart(float speed, int accel);
    /// 103. ruch platformy niezaleznie od konfiguracji konczyn (kazda noga zadaje inny ruch)
    /// funkcja rozpoczyna ruch, wczesniej trzeba wyslac wartosci zadane przy uzyciu funkcji setReferenceValues
    ERR movePlatformRobotComplexStart(float speed, int accel);
    /// 104. przyspieszenie
	ERR setAcceleration(int _accel);
	// 105. wysylanie zadanych katow
	ERR setAngles(int speed);
	// 106.
	ERR sleepODE(int miliseconds);

  public:
    ERR Stop();
	
	CGaits_RC* control;

  private:
    ERR call(float* res=0,int nres=0);
    void copyres(float* res,int nres);

  private:
    char host[200];
    int port;
    int s;
    ERR cerr;
    float* rqd;
    float* rsd;
    /// desired values
    float x_ref[6],y_ref[6],z_ref[6],alpha_ref[6],beta_ref[6],gamma_ref[6];
	///acceleration
	int accel;
	//building local map
	CLocalMap* local_map;
	float robot_position[3];
	float robot_orientation[3];
	float robot_position_temp[3];
	float robot_orientation_temp[3];
};

#endif

