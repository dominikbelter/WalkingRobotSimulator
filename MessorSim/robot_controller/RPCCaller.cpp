/// Wywoływanie zdalnych procedur na serwerze

/*
This file is part of the TCPRobotControl library.
In general, you do not need to modify it.
*/

#include "RPCCaller.h"

#include <string.h>

RPCCaller::RPCCaller(const char* host,int port,COdeWorld* dynamicWorld,CRobotStructure* robot_structure, CLocalMap* local_map)
{
	control = new CGaits_RC(dynamicWorld,robot_structure);
	this->local_map=local_map;
	control->robot_rc->initializeRealRobot();
	strcpy(this->host,host);
	this->port=port;

	// wstępne zapamiętanie pozycji robota
	readPositionIMU(robot_position_temp);
	getIMUorientation(robot_orientation_temp);
	for (int i=0; i<3; i++)
	{
		robot_position[i]=robot_position_temp[i];
		robot_orientation[i]=robot_orientation_temp[i];
	}
}

RPCCaller::~RPCCaller()
{
	delete control;
}

ERR RPCCaller::Run()
{
  return 0;
}

ERR RPCCaller::call(float* res,int nres)
{
  return cerr=0;
}

ERR RPCCaller::Stop()
{
  return 0;
}

///  1. ruch platformy wzgledem pozycji neutralnej
ERR RPCCaller::movePlatform(float x,float y,float z,float alpha,float beta,float gamma, float speed, int _accel)
{
	setAcceleration(_accel);
	if (!control->robot_rc->changePlatform(x,y,z,alpha,beta,gamma,speed,4))
		return 1;//poza zasiegiem stopy
	return 0;
}

///  2. ruch platformy niezaleznie od konfiguracji konczyn
ERR RPCCaller::movePlatformRobot(float x,float y,float z,float alpha,float beta,float gamma, float speed, int _accel)
{
	setAcceleration(_accel);
	if (!control->robot_rc->changePlatformRobot(x,y,z,alpha,beta,gamma,speed,4))
		return 1;//poza zasiegiem stopy
	return 0;
}

///  3. pozycja stopy w ukladzie konczyny
ERR RPCCaller::setFootInLeg(char leg,float x,float y,float z, float speed)
{
  if (!control->robot_rc->setFootPositionLeg( x, y, z, leg, speed))
		return 1;//poza zasiegiem stopy
  return 0;
}

///  4. pozycja stopy w ukladzie robota
ERR RPCCaller::setFootInRobot(char leg,float x,float y,float z, float speed)
{
  if (!control->robot_rc->setFootPositionRobot( x, y, z, leg, speed))
		return 1;//poza zasiegiem stopy
  return 0;
}

///  5. pozycja stopy w ukladzie globalnym
// ERR RPCCaller::setFootInGlobal(char leg,float x,float y,float z)
// {
//   reqf.req_id=5;
//   rqd[0]=leg;
//   rqd[1]=x;
//   rqd[2]=y;
//   rqd[3]=z;
//   if(call())
//     return cerr;
//   return resf.err;
// }

///  6. ruch stopy w ukladzie konczyny
ERR RPCCaller::moveFootInLeg(char leg,float dx,float dy,float dz, float speed)
{
  if (!control->robot_rc->changeFoot(dx,dy,dz, leg, speed))
		return 1;//poza zasiegiem stopy
  return 0;
}

///  7. ruch stopy w ukladzie robota
ERR RPCCaller::moveFootInRobot(char leg,float dx,float dy,float dz, float speed)
{
  if(!control->robot_rc->changeFootRobot(dx,dy,dz, leg, speed))
		return 1;//poza zasiegiem stopy
  return 0;
}

//  8.
// ERR RPCCaller::moveFootInGlobal(char leg,float dx,float dy,float dz)
// {
//   reqf.req_id=8;
//   rqd[0]=leg;
//   rqd[1]=dx;
//   rqd[2]=dy;
//   rqd[3]=dz;
//   if(call())
//     return cerr;
//   return resf.err;
// }

///  9.ustawia katy zadane w konczynie
ERR RPCCaller::setLeg(char leg,float alpha,float beta,float gamma, float speed)
{
	if (!control->robot_rc->changeLegAngles(leg,alpha,beta,gamma,speed))
		return 1;//poza zasiegiem stopy
	return 0;
}

/// 10. odczytuje stan czujnika nacisku w stopie
ERR RPCCaller::readFootForce(char leg,float res[1])
{
  res[0]=control->robot_rc->getFootForce(leg);
  return 0;
}

/// 11.odczytuje katy w stawach
ERR RPCCaller::readLegState(char leg,float res[3])
{
  control->robot_rc->readLegAngles(leg);
  res[0]=control->robot_rc->leg[leg].getPreviousAngle(0);
  res[1]=control->robot_rc->leg[leg].getPreviousAngle(1);
  res[2]=control->robot_rc->leg[leg].getPreviousAngle(2);
  return 0;
}

/// 12.odczytuje katy w stawach dla dwoch konczyn jednoczesnie
ERR RPCCaller::readLegState(char leg1,char leg2,float res[6])
{
  control->robot_rc->readLegAngles(leg1);
  res[0]=control->robot_rc->leg[leg1].getPreviousAngle(0);
  res[1]=control->robot_rc->leg[leg1].getPreviousAngle(0);
  res[2]=control->robot_rc->leg[leg1].getPreviousAngle(0);
  control->robot_rc->readLegAngles(leg2);
  res[3]=control->robot_rc->leg[leg2].getPreviousAngle(0);
  res[4]=control->robot_rc->leg[leg2].getPreviousAngle(0);
  res[5]=control->robot_rc->leg[leg2].getPreviousAngle(0);
  return 0;
}

/// 13. odczytuje wartosci pradow pobieranych przez sewromechanizmy
ERR RPCCaller::readLegCurrent(char leg,float res[3])
{
  printf("TODO %s(%d)\n","readLegCurrent",(int)leg);
  control->robot_rc->getLegCurrent(leg,res);
  return 0;
}

/// 14. odczytuje wartosci pradow pobieranych przez sewromechanizmy(dla dwoch konczyn jednoczesnie)
ERR RPCCaller::readLegCurrent(char leg1,char leg2,float res[6])
{
  printf("TODO %s(%d,%d)\n","readLegCurrent",(int)leg1,(int)leg2);
  control->robot_rc->getLegCurrent(leg1,res);
  res[0]=res[0];
  res[1]=res[1];
  res[2]=res[2];
  control->robot_rc->getLegCurrent(leg2,res);
  res[3]=res[0];
  res[4]=res[1];
  res[5]=res[2];
  return 0;
}

/// 15. odczytuje stan czujnikow nacisku w stopach robota
ERR RPCCaller::readLegForces(float res[6])
{
  printf("TODO %s()\n","readLegForces");
  res[0]=1.5;
  res[1]=1.6;
  res[2]=1.7;
  res[3]=1.8;
  res[4]=1.9;
  res[5]=2.0;
  return 0;
}

/// 16. odczytuje sile kontaktu stopy na podloze obliczana na podstawie czujnikow pradu
ERR RPCCaller::readLegForceInLeg(char leg,float res[3])
{
  printf("TODO %s(%d)\n","readLegForceInLeg",(int)leg);
  res[0]=1.6;
  res[1]=1.7;
  res[2]=1.8;
  return 0;
}

/// 17. odczytuje sile kontaktu stopy na podloze obliczana na podstawie czujnikow pradu (dla dwoch konczyn jednoczesnie)
ERR RPCCaller::readLegForceInLeg(char leg1,char leg2,float res[6])
{
  printf("TODO %s(%d,%d)\n","readLegForceInLeg",(int)leg1,(int)leg2);
  res[0]=1.7;
  res[1]=1.8;
  res[2]=1.9;
  res[3]=2.0;
  res[4]=2.1;
  res[5]=2.2;
  return 0;
}

/// 18. odczytuje sile kontaktu stopy na podloze obliczana na podstawie czujnikow pradu (wartosc w ukladzie robota)
ERR RPCCaller::readLegForceInRobot(char leg,float res[3])
{
  printf("TODO %s(%d)\n","readLegForceInRobot",(int)leg);
  res[0]=1.8;
  res[1]=1.9;
  res[2]=2.0;
  return 0;
}

/// 19. odczytuje sile kontaktu stopy na podloze obliczana na podstawie czujnikow pradu (wartosc w ukladzie robota, dla dwoch konczyn jednoczesnie)
ERR RPCCaller::readLegForceInRobot(char leg1,char leg2,float res[6])
{
  printf("TODO %s(%d,%d)\n","readLegForceInRobot",(int)leg1,(int)leg2);
  res[0]=1.9;
  res[1]=2.0;
  res[2]=2.1;
  res[3]=2.2;
  res[4]=2.3;
  res[5]=2.4;
  return 0;
}

/// 20. odczytuje sile kontaktu stopy na podloze obliczana na podstawie czujnikow pradu (wartosc w ukladzie globalnym)
ERR RPCCaller::readLegForceInGlobal(char leg,float res[3])
{
  printf("TODO %s(%d)\n","readLegForceInGlobal",(int)leg);
  res[0]=2.0;
  res[1]=2.1;
  res[2]=2.2;
  return 0;
}

/// 21. odczytuje sile kontaktu stopy na podloze obliczana na podstawie czujnikow pradu (wartosc w ukladzie globalnym, dla dwoch konczyn jednoczesnie)
ERR RPCCaller::readLegForceInGlobal(char leg1,char leg2,float res[6])
{
  printf("TODO %s(%d,%d)\n","readLegForceInGlobal",(int)leg1,(int)leg2);
  res[0]=2.1;
  res[1]=2.2;
  res[2]=2.3;
  res[3]=2.4;
  res[4]=2.5;
  res[5]=2.6;
  return 0;
}

/// 22. odczytuje pozycje robota mierzona przez IMU
ERR RPCCaller::readPositionIMU(float res[3])
{
  control->robot_rc->dynamicWorld->robotODE.imu.getIMUposition(res);
  return 0;
}

/// 23. resetuje pomiar IMU - ustawia wartosci zero
ERR RPCCaller::resetIMU()
{
  printf("TODO %s()\n","resetIMU");
  return 0;
}

/// 24. odczytuje stan substancji niebezpiecznych
ERR RPCCaller::readSensors(float res[6])
{
  printf("TODO %s()\n","readSensors");
  res[0]=2.4;
  res[1]=2.5;
  res[2]=2.6;
  res[3]=2.7;
  res[4]=2.8;
  res[5]=2.9;
  return 0;
}

/// 25. krok ruchem trojpodporowym
ERR RPCCaller::tripodStep(float x,float y,float z,float alpha,float beta,float gamma, float speed, int _accel)
{
	setAcceleration(_accel);
    if (!control->tripodStep(1000, 1000, x, y, z, alpha, beta, gamma, 0.04, speed, accel))
		return 1;//poza zasiegiem stopy
    return 0;
}

/// 26. krok ruchem pieciopodporowym
ERR RPCCaller::waveStep(float x,float y,float z,float alpha,float beta,float gamma, float speed, int _accel)
{
	setAcceleration(_accel);
  if (!control->waveStep(1000, 1000, x, y, z, alpha, beta, gamma, 0.04, speed,accel))
		return 1;//poza zasiegiem stopy
  return 0;
}

/// 27. przygotowanie do ruchu trojpodporowego
ERR RPCCaller::tripodStepPrepare(float x,float y,float z,float alpha,float beta,float gamma, float speed)
{
  if (!control->tripodPrepare(1000, 1000, x, y, z, alpha, beta, gamma, 0.04, speed))
		return 1;//poza zasiegiem stopy
  return 0;
}

/// 28. zakonczenie ruchu trojpodporowego
ERR RPCCaller::tripodStepFinish(float x,float y,float z,float alpha,float beta,float gamma, float speed)
{
  if (!control->tripodFinish(1000, 1000, x, y, z, alpha, beta, gamma, 0.04, speed))
		return 1;//poza zasiegiem stopy
  return 0;
}

/// 29. przygotowanie do ruchu pieciopodporowego
ERR RPCCaller::wavePrepare(float x,float y,float z,float alpha,float beta,float gamma, float speed)
{
  if (!control->wavePrepare(1000, 1000, x, y, z, alpha, beta, gamma, 0.04, speed,1))
		return 1;//poza zasiegiem stopy
  return 0;
}

/// 30. zakonczenie ruchu pieciopodporowego
ERR RPCCaller::waveFinish(float x,float y,float z,float alpha,float beta,float gamma, float speed)
{
  if (!control->tripodFinish(1000, 1000, x, y, z, alpha, beta, gamma, 0.04, speed))
		return 1;//poza zasiegiem stopy
  return 0;
}

/// 31. krok ruchem trojpodporowym o trajektorii trojkatnej w fazie przenoszenia
ERR RPCCaller::tripodTriangleStep(float x,float y,float z,float alpha,float beta,float gamma, float speed, int _accel)
{
	setAcceleration(_accel);
  if (!control->shortTripodStep(1000, 1000, x, y, z, alpha, beta, gamma, 0.04, speed, accel))
		return 1;//poza zasiegiem stopy
  return 0;
}

/// 32. krok ruchem pieciopodporowym o trajektorii trojkatnej w fazie przenoszenia
ERR RPCCaller::waveTriangleStep(float x,float y,float z,float alpha,float beta,float gamma, float speed, int _accel)
{
	setAcceleration(_accel);
  if (!control->shortWaveStep(1000, 1000, x, y, z, alpha, beta, gamma, 0.04, speed,accel))
		return 1;//poza zasiegiem stopy
  return 0;
}

/// 33. ruch platformy wzgledem pozycji neutralnej (kazda noga zadaje inny ruch)
/// funkcja rozpoczyna ruch, wczesniej trzeba wyslac wartosci zadane przy uzyciu funkcji setReferenceValues
ERR RPCCaller::movePlatformComplex(float * x, float * y, float * z, float * alpha, float * beta, float * gamma, float speed, int accel){
  	for (int i=0;i<6;i++)
		setReferenceValues(x[i],y[i],z[i],alpha[i],beta[i],gamma[i],i);
	movePlatformComplexStart(speed, accel);
  return 0;
}

/// 34. ruch platformy niezaleznie od konfiguracji konczyn (kazda noga zadaje inny ruch)
/// funkcja rozpoczyna ruch, wczesniej trzeba wyslac wartosci zadane przy uzyciu funkcji setReferenceValues
ERR RPCCaller::movePlatformRobotComplex(float * x, float * y, float * z, float * alpha, float * beta, float * gamma, float speed, int accel){
	for (int i=0;i<6;i++)
		setReferenceValues(x[i],y[i],z[i],alpha[i],beta[i],gamma[i],i);
	movePlatformRobotComplexStart(speed, accel);
  //printf("do not use this function");
  return 0;
}

///funkcje obsługujace biblioteke movement (chod omnidirectional)
	/// 35. odswiezenie nastaw czasu kolejnych faz ruchu 
ERR RPCCaller::refresh_time(float res[4]){
	return 0;
}
/// 36. odswiezenie nastaw wysokosci podnoszenia nogi podczas chodu
ERR RPCCaller::refresh_offset_up(float offset_up){
	return 0;
}

/// 37. odswiezenie dowolnego z parametrow ruchu param_t {d_plat,d_robot,d_legx,d_legy,d_leg_up};
ERR RPCCaller::refresh(float param_num, float n_1,float n_2,float n_3,float n_4,float n_5,float n_6){
	return 0;
}
/// 38. ustawienie tablicy chodu, count - licznik wyslanych res[6], musi byc szesc;
ERR RPCCaller::set_gait(float count,float n_1,float n_2,float n_3,float n_4,float n_5,float n_6){
	return 0;
}

/// 39. chod omni
ERR RPCCaller::omni_gait(float size ){
	return 0;
}

// 45.
ERR RPCCaller::stabilizeRobot(void){
	control->robot_rc->stabilizeRobot();
	return 0;
}
    
/// funkcje pomocnicze
/// 101. wartosci zadane dla konczyny przy generowaniu ruchu platformy robota
ERR RPCCaller::setReferenceValues(float x,float y,float z,float alpha,float beta,float gamma, int flag)
{
  if (flag<6) {//ustawienie wartosci zadanych dla poszczegolnych konczyn
    x_ref[flag]=x;
    y_ref[flag]=y;
    z_ref[flag]=z;
    alpha_ref[flag]=alpha;
    beta_ref[flag]=beta;
    gamma_ref[flag]=gamma;
  }
  return 0;
}

/// 102. ruch platformy wzgledem pozycji neutralnej (kazda noga zadaje inny ruch)
/// funkcja rozpoczyna ruch, wczesniej trzeba wyslac wartosci zadane przy uzyciu funkcji setReferenceValues
ERR RPCCaller::movePlatformComplexStart(float speed, int accel){
	if (!control->robot_rc->changePlatform(x_ref, y_ref, z_ref, alpha_ref, beta_ref, gamma_ref, speed, accel))
		return 1;//poza zasiegiem stopy
	return 0;
}

/// 103. ruch platformy niezaleznie od konfiguracji konczyn (kazda noga zadaje inny ruch)
/// funkcja rozpoczyna ruch, wczesniej trzeba wyslac wartosci zadane przy uzyciu funkcji setReferenceValues
ERR RPCCaller::movePlatformRobotComplexStart(float speed, int accel){
    if (!control->robot_rc->changePlatformRobot(x_ref, y_ref, z_ref, alpha_ref, beta_ref, gamma_ref, speed,accel))
		return 1;//poza zasiegiem stopy
	return 0;
}

/// funkcje pomocnicze
/// 104. przyspieszenie
ERR RPCCaller::setAcceleration(int _accel)
{
	accel = _accel;
	return 0;
}

ERR RPCCaller::setAngles(int speed)
{
	return control->robot_rc->sendAngles(speed);
}


ERR RPCCaller::sleepODE(int miliseconds)
{
	control->robot_rc->sleepODE(miliseconds);
	return 0;
}


///////////////////////////////////////funkcje tymczasowe, do zaimplementowania i opisania jak powyżej

ERR RPCCaller::getIMUorientation(float* angles)
{
	control->robot_rc->dynamicWorld->robotODE.imu.getIMUorientation(angles);
	return 0;
}

ERR RPCCaller::Placefeet(float dz, int legs, float speed)
{
	return control->robot_rc->Placefeet(dz,legs,speed);
}

ERR RPCCaller::Contact(int i)
{
	return control->robot_rc->dynamicWorld->robotODE.getContact(i);
}

ERR RPCCaller::checkCollisions(CPunctum body, CPunctum * feet)
{
	return control->robot_rc->checkCollisions(body,feet);
}

ERR RPCCaller::move2GlobalPosition(CPunctum body_prev, CPunctum body, CPunctum * feet_prev, CPunctum * feet, float speed, int soft)
{
	/*//pobranie aktualnej pozycji robota
	readPositionIMU(robot_position);
	getIMUorientation(robot_orientation);

	//obliczenie różnic między położeniem oraz obrotem poprzednim i obecnym
	float dx=robot_position[0]-robot_position_temp[0];
	float dy=robot_position[1]-robot_position_temp[1];
	//float dz=robot_position[2]-robot_position_temp[2];
	float dalpha=robot_orientation[0]-robot_orientation_temp[0];
	float dbeta=robot_orientation[1]-robot_orientation_temp[1];
	float dgamma=robot_orientation[2]-robot_orientation_temp[2];

	//wykonanie skanu terenu (kąt min i max, liczba skanów, przesunięcie w osiach x i y, wysokość z której wykonywany jest pomiar [w układzie robota], obroty wokół poszczególnych osi - x,y,z)
	local_map->makeScan(-1.570796326794,1.570796326794,1,dx,dy,0.202,dalpha,dbeta,dgamma);

	//zapisanie położenia i obrotu w którym wykonywany był pomiar jako położenia poprzedniego
	readPositionIMU(robot_position_temp);
	getIMUorientation(robot_orientation_temp);
	*/
	//przemieszczenie robota
	return control->robot_rc->move2GlobalPosition(body_prev,body,feet_prev,feet,speed,soft);
}