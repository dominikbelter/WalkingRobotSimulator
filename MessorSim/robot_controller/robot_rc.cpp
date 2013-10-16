#include "robot_rc.h"
#include <math.h>
#include <stdio.h>

CRobot_RC::CRobot_RC(COdeWorld* dynamicWorld, CRobotStructure* robot_structure)
: accel_x(0)
, accel_y(0)
{
	this->dynamicWorld=dynamicWorld;
	this->robot_structure=robot_structure;
	//leg[0].reset(lenght,-width_min,0,0,0,0);
	//wyrazenie poczatkow nog w ukladzie lokalnym robota
	for (int i=0;i<6;i++){
		robot_platform_pos[i]=0;
		x_ref_prev[i]=0; y_ref_prev[i]=0; z_ref_prev[i]=0;
		alpha_ref_prev[i]=0; beta_ref_prev[i]=0; gamma_ref_prev[i]=0;
	}
	leg[0].start.setIdentity();
	leg[0].start.setElement(width_min,1,4);
	leg[0].start.setElement(lenght,2,4);

	leg[1].start.setIdentity();
	leg[1].start.setElement(width_max,1,4);

	leg[2].start.setIdentity();
	leg[2].start.setElement(width_min,1,4);
	leg[2].start.setElement(-lenght,2,4);

	leg[3].start.setIdentity();
	leg[3].start.setElement(-1,1,1);
	leg[3].start.setElement(-1,3,3);
	leg[3].start.setElement(-width_min,1,4);
	leg[3].start.setElement(-lenght,2,4);

	leg[4].start.setIdentity();
	leg[4].start.setElement(-1,1,1);
	leg[4].start.setElement(-1,3,3);
	leg[4].start.setElement(-width_max,1,4);
	
	leg[5].start.setIdentity();
	leg[5].start.setElement(-1,1,1);
	leg[5].start.setElement(-1,3,3);
	leg[5].start.setElement(-width_min,1,4);
	leg[5].start.setElement(lenght,2,4);

	for (int i=0;i<6;i++){
	    	if ((i==0)||(i==5)) leg[i].zero_angle[0]=(float)0.785;
		if ((i==1)||(i==4)) leg[i].zero_angle[0]=(float)0;
	    	if ((i==2)||(i==3)) leg[i].zero_angle[0]=(float)-0.785;
		leg[i].zero_angle[1]=(float)0.31867;
		leg[i].zero_angle[2]=(float)-1.4887;
	}

	robot_position[0]=0;
	robot_position[1]=0;
	robot_position[2]=0;
	for (int i=1;i<3;i++) robot_orientation[i]=0;

	//konfiguracja robota
	//board.ConfigureRobot();

	//readRobotAngles();
	/*
	for (int i=0;i<6;i++) {
		for (int j=0;j<3;j++) {
			setAngle(i,j,leg[i].getPreviousAngle(j));
		}
	}
	sendAngles(0.1);
	*/

	robot_global_pos.setIdentity();
	robot_global_pos.setElement(0.189,3,4);
}

CRobot_RC::~CRobot_RC(void)
{
}

//real robot initialization
void CRobot_RC::initializeRealRobot(){
	float x[6]={-0.05,-0.05,-0.05,0.05,0.05,0.05};
	float y[6]={0.0,0.0,0.0,0.0,0.0,0.0};
	float z[6]={-0.18,-0.18,-0.18,-0.18,-0.18,-0.18};
	float alfa[6]={0,0,0,0,0,0};
	float beta[6]={0,0,0,0,0,0};
	float gamma[6]={0,0,0,0,0,0};

	//----------powolne ustawienie do pozycji neutralnej
	float angles[18];
	x[0]=0;x[1]=0;x[2]=0;x[3]=0;x[4]=0;x[5]=0;
	y[0]=0;y[1]=0;y[2]=0;y[3]=0;y[4]=0;y[5]=0;
	z[0]=0.05;z[1]=0;z[2]=0.05;z[3]=0;z[4]=0.05;z[5]=0;

	setAngle(0,0,deg2rad(45));setAngle(0,1,deg2rad(24));setAngle(0,2,deg2rad(-114));
	setAngle(2,0,deg2rad(-45));setAngle(2,1,deg2rad(24));setAngle(2,2,deg2rad(-114));
	setAngle(4,0,deg2rad(0));setAngle(4,1,deg2rad(24));setAngle(4,2,deg2rad(-114));

	setAngle(1,0,deg2rad(0));setAngle(1,1,deg2rad(24));setAngle(1,2,deg2rad(-114));
	setAngle(3,0,deg2rad(-45));setAngle(3,1,deg2rad(24));setAngle(3,2,deg2rad(-114));
	setAngle(5,0,deg2rad(45));setAngle(5,1,deg2rad(24));setAngle(5,2,deg2rad(-114));
	sendAngles(0.1); //opuszczamy konczyne do pozycji neutralnej
}

/// ustawia wartosc (value) kata w nodze leg_number i zlaczu joint number
void CRobot_RC::setAngle(unsigned char leg_number, unsigned char joint_number, double value)
{
	leg[leg_number].setAngle(joint_number,value);
}

/// pobiera wartosc kata w nodze leg_number i zlaczu joint_number
double CRobot_RC::getAngle(unsigned char leg_number, unsigned char joint_number)
{
	return leg[leg_number].getAngle(joint_number);
}

/// pobiera neutralna wartosc kata w nodze leg_number i zlaczu joint_number
double CRobot_RC::getNeutralAngle(unsigned char leg_number, unsigned char joint_number){
	return leg[leg_number].zero_angle[joint_number];
}

/// funkcja obliczajaca zmiane katów przy ruchu platformy robota, kazda noga zadaje inny ruch (w odniesieniu do pozycji neutralnej)
bool CRobot_RC::computeRobotKinematicDeltaAngle(float * x, float * y, float * z, float * alpha, float * beta, float * gamma,float *angle)
{
//rotacja i translacja platformy
	CPunctum ruch[6],rotX[6],rotY[6],rotZ[6];

	for (int i=0;i<6;i++) {
	    rotX[i].setIdentity();	rotY[i].setIdentity();	rotZ[i].setIdentity();

	    rotX[i].setElement(cos(alpha[i]),2,2);	rotX[i].setElement(-sin(alpha[i]),2,3);
	    rotX[i].setElement(sin(alpha[i]),3,2);	rotX[i].setElement(cos(alpha[i]),3,3);

	    rotY[i].setElement(cos(beta[i]),1,1);	rotY[i].setElement(sin(beta[i]),1,3);
	    rotY[i].setElement(-sin(beta[i]),3,1);	rotY[i].setElement(cos(beta[i]),3,3);

	    rotZ[i].setElement(cos(gamma[i]),1,1);	rotZ[i].setElement(-sin(gamma[i]),1,2);
	    rotZ[i].setElement(sin(gamma[i]),2,1);	rotZ[i].setElement(cos(gamma[i]),2,2);

	    ruch[i]=rotX[i]*rotY[i]*rotZ[i];
	    ruch[i].setElement(x[i],1,4);	ruch[i].setElement(y[i],2,4);	ruch[i].setElement(z[i],3,4);
	}
//position poczatkow nog po przemieszczeniu platformy
	CPunctum p[6];
	CPunctum inverse,prosta[6];
	int part;
	float angleTemp[3];
	CPunctum a;

	for (int i=0;i<6;i++){
	    if (i<3) part = 1;
	    else part = 0;
		inverse = ruch[i]*leg[i].start;
	    inverse.invThis();
	    if ((i==0)||(i==5)) prosta[i] = leg[i].forward_kinematic(0.785,0.41867,-1.9887,part);
	    if ((i==1)||(i==4)) prosta[i] = leg[i].forward_kinematic(0,0.41867,-1.9887,part);
	    if ((i==2)||(i==3)) prosta[i] = leg[i].forward_kinematic(-0.785,0.41867,-1.9887,part);
	    p[i]=inverse*leg[i].start*prosta[i]-prosta[i];
	    a=prosta[i]+p[i];
	    if (!leg[i].inverse_kinematic(a.getElement(1,4),a.getElement(2,4),a.getElement(3,4),part,angleTemp))
			return false;
	    angle[i*3]=angleTemp[0]-getAngle(i,0);	angle[i*3+1]=angleTemp[1]-getAngle(i,1);	angle[i*3+2]=angleTemp[2]-getAngle(i,2);
	}
	return true;
}

/// funkcja obliczajaca zmiane katów przy ruchu platformy robota (w odniesieniu do pozycji neutralnej)
bool CRobot_RC::computeRobotKinematicDeltaAngle(float x, float y, float z, float alpha, float beta, float gamma,float *angle)
{
	float x_ref[6],y_ref[6],z_ref[6],alpha_ref[6],beta_ref[6],gamma_ref[6];
	for (int i=0;i<6;i++){
	    x_ref[i]=x; y_ref[i]=y; z_ref[i]=z;
	    alpha_ref[i]=alpha; beta_ref[i]=beta; gamma_ref[i]=gamma;
	}
	if (!computeRobotKinematicDeltaAngle(x_ref, y_ref, z_ref, alpha_ref, beta_ref, gamma_ref, angle))
		return false;
	return true;
}

/// funkcja obliczajaca kinematyke calego robota- alpha rotX, beta rotY,gamma rotZ (wzgledem pozycji neutralnej)
bool CRobot_RC::robotKinematic(float x, float y, float z, float alpha, float beta, float gamma,float * angle)
{
	if (!computeRobotKinematicDeltaAngle(x, y, z, alpha, beta, gamma, angle))
		return false;
	for (int i=0;i<6;i++)
		for (int j=0;j<3;j++)
			angle[i*3+j]+=getAngle(i,j);
	return true;
}

/// funkcja obliczajaca kinematyke calego robota- alpha rotX, beta rotY,gamma rotZ (wzgledem pozycji neutralnej), kazda konczyna moze zadawac inny ruch
bool CRobot_RC::robotKinematic(float * x, float * y, float * z, float * alpha, float * beta, float * gamma,float * angle)
{
	if (!computeRobotKinematicDeltaAngle(x, y, z, alpha, beta, gamma, angle))
		return false;
	for (int i=0;i<6;i++)
		for (int j=0;j<3;j++)
			angle[i*3+j]+=getAngle(i,j);
	return true;
}

/// funkcja obliczajaca zmiane katów przy ruchu platformy robota
bool CRobot_RC::computeRobotRelativeKinematicDeltaAngle(float x, float y, float z, float alpha, float beta, float gamma,float *angle)
{
	float x_ref[6],y_ref[6],z_ref[6],alpha_ref[6],beta_ref[6],gamma_ref[6];
	for (int i=0;i<6;i++){
	    x_ref[i]=x; y_ref[i]=y; z_ref[i]=z;
	    alpha_ref[i]=alpha; beta_ref[i]=beta; gamma_ref[i]=gamma;
	}
	if (!computeRobotRelativeKinematicDeltaAngle(x_ref, y_ref, z_ref, alpha_ref, beta_ref, gamma_ref, angle))
		return false;
	return true;
}


/// funkcja obliczajaca zmiane katow przy ruchu platformy robota, kazda noga zadaje inny ruch
bool CRobot_RC::computeRobotRelativeKinematicDeltaAngleNeutral(float * x, float * y, float * z, float * alpha, float * beta, float * gamma,float *angle,float foot_up, int legs) {
	//rotacja i translacja platformy
	CPunctum ruch[6],rotX[6],rotY[6],rotZ[6];

	for (int i=0;i<6;i++) {
	    rotX[i].setIdentity();	rotY[i].setIdentity();	rotZ[i].setIdentity();

	    rotX[i].setElement(cos(alpha[i]),2,2);	rotX[i].setElement(-sin(alpha[i]),2,3);
	    rotX[i].setElement(sin(alpha[i]),3,2);	rotX[i].setElement(cos(alpha[i]),3,3);

	    rotY[i].setElement(cos(beta[i]),1,1);	rotY[i].setElement(sin(beta[i]),1,3);
	    rotY[i].setElement(-sin(beta[i]),3,1);	rotY[i].setElement(cos(beta[i]),3,3);

	    rotZ[i].setElement(cos(gamma[i]),1,1);	rotZ[i].setElement(-sin(gamma[i]),1,2);
	    rotZ[i].setElement(sin(gamma[i]),2,1);	rotZ[i].setElement(cos(gamma[i]),2,2);

	    ruch[i]=rotX[i]*rotY[i]*rotZ[i];
	    ruch[i].setElement(x[i],1,4);	ruch[i].setElement(y[i],2,4);	ruch[i].setElement(z[i],3,4);
	}
//position startatkow nog po przemieszczeniu platformy
	CPunctum p[6];
	CPunctum inverse,prosta[6];
	int part;
	double angleTemp[3];
	CPunctum a;

	for (int i=0;i<6;i++){
	    if (i<3) part=1;
	    else part=0;
		inverse = ruch[i]*leg[i].start;
	    inverse=inverse.inv();
		if ((((i==0)||(i==2)||(i==4))&&(legs==0))||(((i==1)||(i==3)||(i==5))&&(legs==1))){
			prosta[i] = leg[i].forward_kinematic(getNeutralAngle(i,0),getNeutralAngle(i,1),getNeutralAngle(i,2),part);
			a=prosta[i];
			if (i>2)
				a.setElement(a.getElement(3,4)-foot_up,3,4);
			else
				a.setElement(a.getElement(3,4)+foot_up,3,4);
		}
		else {
			prosta[i] = leg[i].forward_kinematic(getAngle(i,0),getAngle(i,1),getAngle(i,2),part);
			a=prosta[i];
		}
	    p[i]=inverse*leg[i].start*prosta[i]-prosta[i];

	    a=a+p[i];
	    if (!leg[i].inverse_kinematic(a.getElement(1,4),a.getElement(2,4),a.getElement(3,4),part,angleTemp))
			return false;
	//	if ((_isnan(getAngle(i,0)))||(_isnan(getAngle(i,1)))||(_isnan(getAngle(i,2)))||(_isnan(angleTemp[0]))||(_isnan(angleTemp[1]))||(_isnan(angleTemp[2])))
			volatile int s=1;
	    angle[i*3]=float(angleTemp[0]-getAngle(i,0));	angle[i*3+1]=float(angleTemp[1]-getAngle(i,1));	angle[i*3+2]=float(angleTemp[2]-getAngle(i,2));
	}
	return true;
}


/// funkcja obliczajaca zmiane katów przy ruchu platformy robota, kazda noga zadaje inny ruch
bool CRobot_RC::computeRobotRelativeKinematicDeltaAngle(float * x, float * y, float * z, float * alpha, float * beta, float * gamma,float *angle)
{
//rotacja i translacja platformy
	CPunctum ruch[6],rotX[6],rotY[6],rotZ[6];

	for (int i=0;i<6;i++) {
	    rotX[i].setIdentity();	rotY[i].setIdentity();	rotZ[i].setIdentity();

	    rotX[i].setElement(cos(alpha[i]),2,2);	rotX[i].setElement(-sin(alpha[i]),2,3);
	    rotX[i].setElement(sin(alpha[i]),3,2);	rotX[i].setElement(cos(alpha[i]),3,3);

	    rotY[i].setElement(cos(beta[i]),1,1);		rotY[i].setElement(sin(beta[i]),1,3);
	    rotY[i].setElement(-sin(beta[i]),3,1);	rotY[i].setElement(cos(beta[i]),3,3);

	    rotZ[i].setElement(cos(gamma[i]),1,1);	rotZ[i].setElement(-sin(gamma[i]),1,2);
	    rotZ[i].setElement(sin(gamma[i]),2,1);	rotZ[i].setElement(cos(gamma[i]),2,2);

	    ruch[i]=rotX[i]*rotY[i]*rotZ[i];
	    ruch[i].setElement(x[i],1,4);	ruch[i].setElement(y[i],2,4);	ruch[i].setElement(z[i],3,4);
	}
//position startatkow nog po przemieszczeniu platformy
	CPunctum p[6];
	CPunctum inverse,prosta[6];
	int part;
	double angleTemp[3];

	for (int i=0;i<6;i++){
	    if (i<3) part=1;
	    else part=-1;
		inverse = ruch[i]*leg[i].start;
	    inverse=inverse.inv();
	    prosta[i] = leg[i].forward_kinematic(getAngle(i,0),getAngle(i,1),getAngle(i,2),part);
		p[i]=inverse*leg[i].start*prosta[i];//-prosta[i];

//	    a=leg[i].forward_kinematic(getAngle(i,0),getAngle(i,1),getAngle(i,2),part);
//	    a=a+p[i];
	    if (!leg[i].inverse_kinematic(p[i].getElement(1,4),p[i].getElement(2,4),p[i].getElement(3,4),part,angleTemp))
			return false;
	    angle[i*3]=angleTemp[0]-getAngle(i,0);	angle[i*3+1]=angleTemp[1]-getAngle(i,1);	angle[i*3+2]=angleTemp[2]-getAngle(i,2);
	}
	return true;
}

/// funkcja obliczajaca kinematyke calego robota- alpha rotX, beta rotY,gamma rotZ
bool CRobot_RC::robotRelativeKinematic(float x, float y, float z, float alpha, float beta, float gamma,float angle[]){
	if (!computeRobotRelativeKinematicDeltaAngle(x, y, z, alpha, beta, gamma, angle))
		return false;
	for (int i=0;i<6;i++)
		for (int j=0;j<3;j++)
			angle[i*3+j]+=getAngle(i, j);
	return true;
}

/// funkcja obliczajaca kinematyke calego robota- alpha rotX, beta rotY,gamma rotZ - przesuniecia wzgledne dla kazdej konczyny osobno
bool CRobot_RC::robotRelativeKinematic(float * x, float * y, float * z, float * alpha, float * beta, float * gamma,float * angle){
	if (!computeRobotRelativeKinematicDeltaAngle(x, y, z, alpha, beta, gamma, angle))
		return false;
	for (int i=0;i<6;i++)
		for (int j=0;j<3;j++)
			angle[i*3+j]+=getAngle(i, j);
	return true;
}


/// funkcja obliczajaca kinematyke calego robota- alpha rotX, beta rotY,gamma rotZ - przesuniecia wzgledne dla kazdej konczyny osobno
bool CRobot_RC::robotRelativeKinematicNeutral(float * x, float * y, float * z, float * alpha, float * beta, float * gamma, int legs, float * angle){
/*	if (!computeRobotRelativeKinematicDeltaAngleNeutral(x, y, z, alpha, beta, gamma, angle, legs))
		return false;
	for (int i=0;i<6;i++)
		for (int j=0;j<3;j++)
			angle[i*3+j]+=getAngle(i, j);*/
	return true;
	
}

/// ustawia wartosci katow we wszystkich stawach
void CRobot_RC::setLegs(float a, float b, float c, float d, float e, float f, float g, float h, float i, float j, float k, float l, float m, float n, float o, float p, float r, float s)
{
	float control[18]={a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,r,s};
	for (int i=0;i<6;i++)
		for (int j=0;j<3;j++)
			setAngle(i, j, control[i*3+j]);
}

/// ustawia wartosci katow we wszystkich stawach
void CRobot_RC::setLegs(float * angles){
	for (int i=0;i<6;i++)
		for (int j=0;j<3;j++)
			setAngle(i, j, angles[i*3+j]);
}

/// zmienia wartosci zadane w stawach danej konczyny, katy podawane sa w kolejnosci od korpusu
bool CRobot_RC::changeLegAngles(unsigned char leg_no, float alpha, float beta, float gamma, float speed) {
	float delta_angle[3];
	float max_value;
	leg[leg_no].computeMoveDeltaAngle(alpha,beta,gamma,delta_angle);
	int max = findMax(delta_angle, 3, &max_value);
	float real_speed = speed*MAX_SERVO_SPEED_RC; //rzeczywista, zmniejszona predkosc serwomechanizmu
	float delta_t = max_value/real_speed;
	int iterator = (int)(delta_t/SERVO_DELAY_RC); // znajdujemsetServoy liczbe iteracji potrzebnych na wykonanie ruchu
	for (int i=1; i<=iterator;i++){
		leg[leg_no].setAngle(0,leg[leg_no].getAngle(0)+(delta_angle[0]/(float) iterator));		
		leg[leg_no].setAngle(1,leg[leg_no].getAngle(1)+(delta_angle[1]/(float) iterator));		
		leg[leg_no].setAngle(2,leg[leg_no].getAngle(2)+(delta_angle[2]/(float) iterator));
		sendAngles(speed);
		this->sleepODE(SERVO_DELAY_RC*1000);
	}
	sendAngles(speed);
	return true;
}

/// oblicza predkosci w wezlach danej nogi
double CRobot_RC::computeLegSpeed(char leg_no, float speed, short int *leg_speed){
	float delta_angle[3];
	unsigned char max;
	for(int i=0;i<3;i++) {delta_angle[i]=fabs(leg[leg_no].getAngle(i)-leg[leg_no].getPreviousAngle(i));}
	float max_value;
	max = findMax(delta_angle, 3, &max_value);
	for (int i=0;i<3;i++) {
		leg_speed[i] = short(((delta_angle[i]/delta_angle[max])*speed)*MAX_SPEED_RC)+1;
	}
	return 0;
}

/// przeksztalca i wysyla wartosci zadane dla serwomechanizmow
double CRobot_RC::sendAngles(float speed){
	float leg_deg[3];
	short int leg_speed[3];
	computeLegSpeed(0,speed, leg_speed);
	dynamicWorld->robotODE.setLegSpeed(0, leg_speed);
	computeLegSpeed(1,speed, leg_speed);
	dynamicWorld->robotODE.setLegSpeed(1, leg_speed);
	computeLegSpeed(2,speed, leg_speed);
	dynamicWorld->robotODE.setLegSpeed(2, leg_speed);
	computeLegSpeed(3,speed, leg_speed);
	dynamicWorld->robotODE.setLegSpeed(3, leg_speed);
	computeLegSpeed(4,speed, leg_speed);
	dynamicWorld->robotODE.setLegSpeed(4, leg_speed);
	computeLegSpeed(5,speed, leg_speed);
	dynamicWorld->robotODE.setLegSpeed(5, leg_speed);

	float Q_ref[18];
	for (int i=0;i<6;i++){
		Q_ref[i*3] = getAngle(i,0);
		Q_ref[i*3+1] = getAngle(i,1);
		Q_ref[i*3+2] = getAngle(i,2);
	}
	bool collision_table[44];
	float rob_pos[3];
	dynamicWorld->robotODE.imu.getIMUposition(rob_pos);
	float orientation[3];
	dynamicWorld->robotODE.imu.getIMUorientation(orientation);
	int offset[2];
	dynamicWorld->ground->CalculateGroundCoordinates(rob_pos[0],rob_pos[1],offset);
	if (robot_structure->checkCollision(rob_pos[0],rob_pos[1],rob_pos[2],orientation[0],orientation[1],orientation[2], Q_ref,collision_table,false,dynamicWorld->ground->points,offset[0],offset[1])) 
		return 1;//collision exist

	leg_deg[0] = getAngle(0, 0);
	leg_deg[1] = getAngle(0, 1);
	leg_deg[2] = getAngle(0, 2);
	dynamicWorld->robotODE.setLeg(0, leg_deg);

	leg_deg[0] = getAngle(1, 0);
	leg_deg[1] = getAngle(1, 1);
	leg_deg[2] = getAngle(1, 2);
	dynamicWorld->robotODE.setLeg(1, leg_deg);

	leg_deg[0] = getAngle(2, 0);
	leg_deg[1] = getAngle(2, 1);
	leg_deg[2] = getAngle(2, 2);
	dynamicWorld->robotODE.setLeg(2, leg_deg);

	leg_deg[0] = getAngle(3, 0);
	leg_deg[1] = getAngle(3, 1);
	leg_deg[2] = getAngle(3, 2);
	dynamicWorld->robotODE.setLeg(3, leg_deg);

	leg_deg[0] = getAngle(4, 0);
	leg_deg[1] = getAngle(4, 1);
	leg_deg[2] = getAngle(4, 2);
	dynamicWorld->robotODE.setLeg(4, leg_deg);

	leg_deg[0] = getAngle(5, 0);
	leg_deg[1] = getAngle(5, 1);
	leg_deg[2] = getAngle(5, 2);
	dynamicWorld->robotODE.setLeg(5, leg_deg);
	return 0;
}

/// przeksztalca i wysyla wartosci zadane dla serwomechanizmow
double CRobot_RC::sendAngles(float * speed){
	float leg_deg[3];
	short int leg_speed[3];
	computeLegSpeed(0,speed[0], leg_speed);
	dynamicWorld->robotODE.setLegSpeed(0, leg_speed);
	computeLegSpeed(1,speed[1], leg_speed);
	dynamicWorld->robotODE.setLegSpeed(1, leg_speed);
	computeLegSpeed(2,speed[2], leg_speed);
	dynamicWorld->robotODE.setLegSpeed(2, leg_speed);
	computeLegSpeed(3,speed[3], leg_speed);
	dynamicWorld->robotODE.setLegSpeed(3, leg_speed);
	computeLegSpeed(4,speed[4], leg_speed);
	dynamicWorld->robotODE.setLegSpeed(4, leg_speed);
	computeLegSpeed(5,speed[5], leg_speed);
	dynamicWorld->robotODE.setLegSpeed(5, leg_speed);

	float Q_ref[18];
	for (int i=0;i<6;i++){
		Q_ref[i*3] = getAngle(i,0);
		Q_ref[i*3+1] = getAngle(i,1);
		Q_ref[i*3+2] = getAngle(i,2);
	}
	bool collision_table[44];
	float rob_pos[3];
	dynamicWorld->robotODE.imu.getIMUposition(rob_pos);
	float orientation[3];
	dynamicWorld->robotODE.imu.getIMUorientation(orientation);
	int offset[2];
	dynamicWorld->ground->CalculateGroundCoordinates(rob_pos[0],rob_pos[1],offset);
	if (robot_structure->checkCollision(rob_pos[0],rob_pos[1],rob_pos[2],orientation[0],orientation[1],orientation[2], Q_ref,collision_table,false,dynamicWorld->ground->points,offset[0],offset[1])) 
		return 1;//collision exist

	leg_deg[0] = getAngle(0, 0);
	leg_deg[1] = getAngle(0, 1);
	leg_deg[2] = getAngle(0, 2);
	dynamicWorld->robotODE.setLeg(0, leg_deg);

	leg_deg[0] = getAngle(1, 0);
	leg_deg[1] = getAngle(1, 1);
	leg_deg[2] = getAngle(1, 2);
	dynamicWorld->robotODE.setLeg(1, leg_deg);

	leg_deg[0] = getAngle(2, 0);
	leg_deg[1] = getAngle(2, 1);
	leg_deg[2] = getAngle(2, 2);
	dynamicWorld->robotODE.setLeg(2, leg_deg);

	leg_deg[0] = getAngle(3, 0);
	leg_deg[1] = getAngle(3, 1);
	leg_deg[2] = getAngle(3, 2);
	dynamicWorld->robotODE.setLeg(3, leg_deg);

	leg_deg[0] = getAngle(4, 0);
	leg_deg[1] = getAngle(4, 1);
	leg_deg[2] = getAngle(4, 2);
	dynamicWorld->robotODE.setLeg(4, leg_deg);

	leg_deg[0] = getAngle(5, 0);
	leg_deg[1] = getAngle(5, 1);
	leg_deg[2] = getAngle(5, 2);
	dynamicWorld->robotODE.setLeg(5, leg_deg);
	return 0;
}

/// odczytuje sile w stopie z czujnika nacisku
short int CRobot_RC::getFootForce(char leg_no){
	return 0;
}

/// odczytuje sile w stopie bazujac na czujnnikach pradu
void CRobot_RC::getLegForce(char leg_no, unsigned int force[3]){
	//TODO read force 
	force[0]=0;
	force[1]=0;
	force[2]=0;
}

/// odczytuje prad pobierany przez silniki w stawach
void CRobot_RC::getLegCurrent(char leg_no, float current[3]){
	//TODO read current
	current[0]=0;
	current[1]=0;
	current[2]=0;
}

/// odczytuje i odbiera wartosci rzeczywiste katow w stawach dla nogi leg_no
void CRobot_RC::readLegAngles(char leg_no){
	short int read_angles[3];
	dynamicWorld->robotODE.readLegPosition(leg_no, read_angles);
	leg[leg_no].setPreviousAngle(0, deg2rad(read_angles[0]));
	leg[leg_no].setPreviousAngle(1, deg2rad(read_angles[1]));
	leg[leg_no].setPreviousAngle(2, deg2rad(read_angles[2]));
}

/// odczytuje i odbiera wartosci rzeczywiste katow w stawach
double CRobot_RC::readRobotAngles(){
	for (int i=0;i<6;i++){
		readLegAngles(i);
	}
	short int read_angles[3];
	for (int i=0;i<6;i++){
		dynamicWorld->robotODE.readLegPosition(i, read_angles);
	}
	return 0;
}

/// ustawia pozycje stopy w ukladzie konczyny
bool CRobot_RC::setFootPositionLeg(double x,double y, double z, char leg_no, float speed) {
	CPunctum foot;
	if (leg_no<3)
	  foot = leg[leg_no].forward_kinematic(leg[leg_no].getAngle(0), leg[leg_no].getAngle(1), leg[leg_no].getAngle(2), 1 );
	else
	  foot = leg[leg_no].forward_kinematic(leg[leg_no].getAngle(0), leg[leg_no].getAngle(1), leg[leg_no].getAngle(2), -1 );
	if (!changeFoot(x-foot.getElement(1,4), y-foot.getElement(2,4), z-foot.getElement(3,4), leg_no, speed))
		return false;
	return true;
}

/// ustawia pozycje stopy w ukladzie robota
bool CRobot_RC::setFootPositionRobot(double x,double y, double z, char leg_no, float speed) {
	CPunctum foot;
	if (leg_no<3)
	  foot = leg[leg_no].forward_kinematic(leg[leg_no].getAngle(0), leg[leg_no].getAngle(1), leg[leg_no].getAngle(2), 1 );
	else
	  foot = leg[leg_no].forward_kinematic(leg[leg_no].getAngle(0), leg[leg_no].getAngle(1), leg[leg_no].getAngle(2), -1 );
	foot = leg[leg_no].start*foot;
	if (!changeFoot(x-foot.getElement(1,4), y-foot.getElement(2,4), z-foot.getElement(3,4), leg_no, speed))
		return false;
	return true;
}

/*kroczenie po trudnym terenie*/

///pobiera dane o pozycji i orientacji robota
CPunctum CRobot_RC::getRobotState()
{
	CPunctum robot_pos;
	float angles[3];
	float position[3];
	getRotAngles(angles);
	dynamicWorld->robotODE.imu.getIMUposition(position);
	robot_pos.createTRMatrix( angles[0],angles[1],angles[2],position[0],position[1],position[2]);
	robot_pos.orientation[0]=angles[0]; robot_pos.orientation[1]=angles[1]; robot_pos.orientation[2]=angles[2];
	return robot_pos;

}
void CRobot_RC::getRotAngles(float *angles)
{
	dynamicWorld->robotODE.imu.getIMUorientation(angles);
}
/// ustawia pozycje stopy w ukladzie globalnym
bool CRobot_RC::setFootPositionGlobal(double globalx, double globaly, double globalz, int legnumber)
{
	int part;
	if (legnumber>2){
		part=-1;
	}
	else part=1;
 	// macierz transformacji
	CPunctum transform_matrix;
	CPunctum robot_pos;//pozycja robota w ukladzie globalnym
	CPunctum point; //zadana pozycja punktu w ukladzie globalnym
	CPunctum point_leg;//szukana pozycja stopyw ukladzie konczyny

	point.createTRMatrix(0,0,0,globalx,globaly,globalz);
	//robot_pos.createTRMatrix(deg2rad(robot_orientation[0]),deg2rad(robot_orientation[1]),deg2rad(robot_orientation[2]),robot_position[0],robot_position[1],robot_position[2]);
	//robot_pos.createTRMatrix(0,0,0,0,0,0);
	//macierz przeksztalcenia uklad globalny->uklad poczatku nogi
	//transform_matrix=robot_global_pos*leg[legnumber].start;
	transform_matrix=CRobot_RC::getRobotState()*leg[legnumber].start;
	transform_matrix.invThis();
	//wyznaczenie punktu docelowego dla nogi we wspolrzednych globalnym
	point_leg=transform_matrix*point;
	//wykonanie ruchu nogi

	//CPunctum s = leg[legnumber].forward_kinematic(leg[legnumber].getAngle(0),leg[legnumber].getAngle(1),leg[legnumber].getAngle(2),part);

	//CPunctum wynik = robot_pos*leg[legnumber].start*s;

	////obliczenie katow w stawach, ktore przeniosa stope do zadanego punktu
	////double temp_angle[3];
	////leg[legnumber].inverse_kinematic(point_leg.getElement(1,4),point_leg.getElement(2,4),point_leg.getElement(3,4), right, temp_angle);
	//przeniesienie stopy do punktu we wspolrzednych nogi
	if (!leg[legnumber].setFootPosition(point_leg.getElement(1,4),point_leg.getElement(2,4),point_leg.getElement(3,4),part))
		return false;
    //// wpisanie wartosci zadanych
	////leg[legnumber].setAngle(0, temp_angle[0]);
	////leg[legnumber].setAngle(1, temp_angle[1]);
	////leg[legnumber].setAngle(2, temp_angle[2]);
	return true;
}

/// sprawdza czy pozycja stopy jest osiagalna
bool CRobot_RC::isFootPositionAvailableGlobal(CPunctum robot_pos, double globalx, double globaly, double globalz, int legnumber, float scale){
	int part;
		if (legnumber>2)
		{
			part=0;
		}
		else part=1;
	 // macierz transformacji
	CPunctum transform_matrix;
	CPunctum point; //zadana pozycja punktu w ukladzie globalnym
	CPunctum point_leg;//szukana pozycja stopyw ukladzie konczyny

	point.createTRMatrix(0,0,0,globalx,globaly,globalz);
	//robot_pos.createTRMatrix(deg2rad(robot_orientation[0]),deg2rad(robot_orientation[1]),deg2rad(robot_orientation[2]),robot_position[0],robot_position[1],robot_position[2]);
	//robot_pos.createTRMatrix(0,0,0,0,0,0);
	//macierz przeksztalcenia uklad globalny->uklad poczatku nogi
	//transform_matrix=robot_global_pos*leg[legnumber].start;
	transform_matrix=robot_pos*leg[legnumber].start;
	transform_matrix.invThis();
	//wyznaczenie punktu docelowego dla nogi we wspolrzednych globalnym
	point_leg=transform_matrix*point;
	//wykonanie ruchu nogą

//	CPunctum s = leg[legnumber].forward_kinematic(leg[legnumber].getAngle(0)*180/PI,leg[legnumber].getAngle(1)*180/PI,leg[legnumber].getAngle(2)*180/PI,right);

	//przeniesienie stopy do punktu we wspolrzednych nogi
	if (leg[legnumber].isFootPositionAvailableLocal(point_leg.getElement(1,4),point_leg.getElement(2,4),point_leg.getElement(3,4),part,0,scale))
		return true;
	else
		return false;
}

//check stability
//stability margin 0(min)-1(max-normal)
bool CRobot_RC::checkStability(CPunctum body, CPunctum * feet, float stability_margin){
	float angles[18];
	CPunctum leg_m[6], tmp;//masy konczyn w ukladzie globalnym
	int part;
	for (int i=0;i<6;i++){
	    if (i<3) part=1;
	    else part=0;
		tmp = body*leg[i].start;
		tmp.invThis();
		CPunctum foot_pos;
		foot_pos = tmp * feet[i];
		if (!leg[i].inverse_kinematic(foot_pos.getElement(1,4),foot_pos.getElement(2,4),foot_pos.getElement(3,4),part,&angles[3*i]))
			return false;
		leg_m[i]=body*leg[i].start*leg[i].computeCenterOfMass(&angles[i*3],(i<3?1:0));//obliczenie srodka masy w ukladzie konczyny i przeliczenie do ukladu globalnego
	}
	float mass_leg = 0.405;
	float mass_body = 1.908;
	CPunctum com;
	com.setIdentity();
	float pos[3]={0,0,0};//wspolrzedne dla konczyn
	for (int i=0;i<6;i++){
		pos[0]+=leg_m[i].getElement(1,4)*mass_leg;
		pos[1]+=leg_m[i].getElement(2,4)*mass_leg;
		pos[2]+=leg_m[i].getElement(3,4)*mass_leg;
	}
	com.setElement((body.getElement(1,4)*mass_body+pos[0])/(mass_leg*6+mass_body),1,4);
	com.setElement((body.getElement(2,4)*mass_body+pos[1])/(mass_leg*6+mass_body),2,4);
	com.setElement((body.getElement(3,4)*mass_body+pos[2])/(mass_leg*6+mass_body),3,4);
	float a[2],b[2],c[2];//wierzcholki trojkata
	float barycentrum[2];//srodek ciezkosci
	float com2d[2]={com.getElement(1,4),com.getElement(2,4)};
	if (feet[0].isFoothold()){//trojkat podparcia na nieparzystych
		a[0]=feet[0].getElement(1,4); a[1]=feet[0].getElement(2,4);
		b[0]=feet[2].getElement(1,4); b[1]=feet[2].getElement(2,4);
		c[0]=feet[4].getElement(1,4); c[1]=feet[4].getElement(2,4);
	}
	else {
		a[0]=feet[1].getElement(1,4); a[1]=feet[1].getElement(2,4);
		b[0]=feet[3].getElement(1,4); b[1]=feet[3].getElement(2,4);
		c[0]=feet[5].getElement(1,4); c[1]=feet[5].getElement(2,4);
	}
	if (stability_margin>0){//zmniejszenie trojkata podparcia
		barycentrum[0]=(a[0]+b[0]+c[0])/3.0; barycentrum[1]=(a[1]+b[1]+c[1])/3.0;
		a[0]+=((barycentrum[0]-a[0])*stability_margin); a[1]+=((barycentrum[1]-a[1])*stability_margin);
		b[0]+=((barycentrum[0]-b[0])*stability_margin); b[1]+=((barycentrum[1]-b[1])*stability_margin);
		c[0]+=((barycentrum[0]-c[0])*stability_margin); c[1]+=((barycentrum[1]-c[1])*stability_margin);		
	}
	if (triangleIncludePoint(a,b,c,com2d))
		return true;
	else
		return false;
}

//check stability
bool CRobot_RC::isStable(CPunctum body, CPunctum * feet){
	computeComPosition(body,feet);
	float ** vertices;
	vertices = new float *[6];
	for (int i=0;i<6;i++)
		vertices[i]=new float[2];
	float com2d[2]={com.getElement(1,4),com.getElement(2,4)};
	int support_no=0;
	for (int i=0;i<6;i++){//wielokat podparcia na nieparzystych
		//if (feet[i].isFoothold()){
		if (this->dynamicWorld->robotODE.getContact(i)) {
			vertices[support_no][0]=feet[i].getElement(1,4); 
			vertices[support_no][1]=feet[i].getElement(2,4);
			support_no++;
		}
	}
	if (support_no<3)
		return false;
	computePolygonCentroid(vertices, support_no, &supp_centroid[0], &supp_centroid[1]);
	if (polygonIncludePoint(vertices,com2d,support_no)){
		for (int i=0;i<6;i++)
			delete [] vertices[i];
		delete [] vertices;
		return true;
	}
	else {
		for (int i=0;i<6;i++)
			delete [] vertices[i];
		delete [] vertices;
		return false;
	}
}

//check stability
bool CRobot_RC::computeComPosition(CPunctum body, CPunctum * feet){
	float angles[18];
	CPunctum leg_m[6], tmp;//masy konczyn w ukladzie globalnym
	int part;
	CPunctum foot_pos;
	for (int i=0;i<6;i++){
	    if (i<3) part=1;
	    else part=0;
		tmp = body*leg[i].start;
		tmp.invThis();
		foot_pos = tmp * feet[i];
		if (!leg[i].inverse_kinematic(foot_pos.getElement(1,4),foot_pos.getElement(2,4),foot_pos.getElement(3,4),part,&angles[3*i]))
			return false;
		leg_m[i]=body*leg[i].start*leg[i].computeCenterOfMass(&angles[i*3],(i<3?1:0));//obliczenie srodka masy w ukladzie konczyny i przeliczenie do ukladu globalnego
	}
	float mass_leg = 0.405;
	float mass_body = 1.908;
	com.setIdentity();
	float pos[3]={0,0,0};//wspolrzedne dla konczyn
	for (int i=0;i<6;i++){
		pos[0]+=leg_m[i].getElement(1,4)*mass_leg;
		pos[1]+=leg_m[i].getElement(2,4)*mass_leg;
		pos[2]+=leg_m[i].getElement(3,4)*mass_leg;
	}
	com.setElement((body.getElement(1,4)*mass_body+pos[0])/(mass_leg*6+mass_body),1,4);
	com.setElement((body.getElement(2,4)*mass_body+pos[1])/(mass_leg*6+mass_body),2,4);
	com.setElement((body.getElement(3,4)*mass_body+pos[2])/(mass_leg*6+mass_body),3,4);
}

/// show com
void CRobot_RC::show_center_of_mass(void) {
	CPunctum body;
	CPunctum feet[6];
	getFullRobotState(&body, feet);
	this->isStable(body,feet);

	glColor3f(1,0,0);
	glPushMatrix();
	glTranslatef(com.getElement(1,4)*10, com.getElement(3,4)*10, -com.getElement(2,4)*10);
	glutSolidSphere(1.3*0.1,5,5);
	glPopMatrix();

	glColor3f(0,1,0);
	glPushMatrix();
	glTranslatef(supp_centroid[0]*10, 0.01*10, -supp_centroid[1]*10);
	glutSolidSphere(1.3*0.1,5,5);
	glPopMatrix();
}

///robot stabilization procedure
void CRobot_RC::stabilizeRobot(void){
	CPunctum body;
	CPunctum feet[6];
	for (int i=0;i<80;i++){
		getFullRobotState(&body, feet);
		computeComPosition(body,feet);
		isStable(body, feet);
		float curr_pos[3]={com.getElement(1,4), com.getElement(2,4), com.getElement(3,4)};
		float dest_pos[3]={supp_centroid[0], supp_centroid[1], com.getElement(3,4)};
		react(curr_pos, dest_pos, 0.005,0.9,0.02);
		//this->Placefeet(0.005,0.9);
		//dynamicWorld->SimStep();
	}
}

void CRobot_RC::react(float * curr_pos, float * dest_pos, float d_z, float speed, float speed_horizontal){
	float x_leg[6]={0,0,0,0,0,0};
	float y_leg[6]={0,0,0,0,0,0};
	float z_leg[6]={0,0,0,0,0,0};
    CPunctum movement;//ruch w ukladzie robota
	float track=false;
	///----obliczenia wartosci zadanych: nogi w powietrzu sa opuszczane, na ziemi przesuwaja platforme do wielokatu podparcia
	float dx = dest_pos[0]-curr_pos[0];
	float dy = dest_pos[1]-curr_pos[1];
	float dz = dest_pos[2]-curr_pos[2];
	if (sqrt(pow(dx,2.0)+pow(dy,2.0)+pow(dz,2.0))>0.01){// sledzimy srodek wielokata podparcia
		track=true;
	}
	int stycznik[6]={0,0,0,0,0,0};
	bool stable=true;
	for (int i=0;i<6;i++){
		if (dynamicWorld->robotODE.getContact(i))
			stycznik[i]=1;
		else
			stable = false;
	}
	if (stable) {
		this->sleepODE(SERVO_DELAY_RC*1000);
		return;
	}
	float speed_ref[6]={0,0,0,0,0,0};
	for (int i=0; i<6;i++){//obliczenie ruchu w ukladzie konczyny, pomijamy obliczenia na macierzach ze wzgledu na szybkosc dzialania
		if (stycznik[i]==1) {
			z_leg[i]=0;
			if (track){
				x_leg[i]=dx*speed_horizontal;
				y_leg[i]=dy*speed_horizontal;
			}
			speed_ref[i]=speed_horizontal;
		}
		else {
			z_leg[i]=d_z;
			speed_ref[i]=speed;
		}
	}
	///----koniec obliczen wartosci zadanych

	float zeros[6]={0,0,0,0,0,0};
	float delta_angle[18]; //zmiany katow w stawach konczyny
	float real_speed = speed*MAX_SERVO_SPEED_RC; //rzeczywista, zmniejszona predkosc serwomechanizmu
	///----uwzglednienie rotacji
	float rota[3];
	getRotAngles(rota);
	float rotx[6]={0,0,0,0,0,0};
	float roty[6]={0,0,0,0,0,0};
	float rotz[6]={0,0,0,0,0,0};
	for (int i=0; i<6;i++){
		if (stycznik[i]==1) {
			rotx[i]=-rota[0]*speed_horizontal;
			roty[i]=-rota[1]*speed_horizontal;
		}
	}
	///----uwzglednienie rotacji
	while (!computeRobotRelativeKinematicDeltaAngle(x_leg, y_leg, z_leg, rotx, roty, rotz, delta_angle)) {
		for (int i=0;i<6;i++){
			x_leg[i]=x_leg[i]/2;
			y_leg[i]=y_leg[i]/2;
			rotx[i]=rotx[i]/2;
			roty[i]=roty[i]/2;
		}
	}

	float max_delta_angle;
	findAbsMax(delta_angle, 18, &max_delta_angle);//znajdujemy najwiekszy kat

	float delta_t = max_delta_angle/real_speed;//czas spedzony na poruszanie sie z maksymalna predkoscia
	int iterator = (int)(delta_t/SERVO_DELAY_RC); // znajdujemy liczbe iteracji potrzebnych na wykonanie ruchu
	if (iterator==0) iterator=1;
	float x_zad[6], y_zad[6], z_zad[6];
	for (int j=0; j<6;j++){// krok ruchu dla kazdego wezla kinematycznego
		x_zad[j]=x_leg[j]/iterator; y_zad[j]=y_leg[j]/iterator; z_zad[j]=z_leg[j]/iterator;
		rotx[j]=rotx[j]/iterator, roty[j]=roty[j]/iterator; rotz[j]=rotz[j]/iterator;
	}
		if (!robotRelativeKinematic(x_zad, y_zad, z_zad, rotx, roty, rotz, delta_angle))
			return;
		setLegs(delta_angle);
		sendAngles(speed_ref);
		this->sleepODE(SERVO_DELAY_RC*1000);

//	return true;
}
///computes stability margin
float CRobot_RC::computeStabilityMargin(CPunctum body, CPunctum * feet){
	if (!isStable(body, feet))
		return 0;
	CPunctum rob;
	for (int r=0;r<15;r++){
		for (int theta=0;theta<8;theta++){
			rob.createTRMatrix(0,0,0,r*0.01*cos(theta*0.785),r*0.01*sin(theta*0.785),0);
			rob=body*rob;
			if (!isStable(rob, feet))
				return r*0.01;
		}
	}
	return 0.15;
}

///move body closer to stability center
void CRobot_RC::increaseStabilityMargin(CPunctum *body, CPunctum * feet, float distance){
	float ** vertices;
	vertices = new float *[6];
	for (int i=0;i<6;i++)
		vertices[i]=new float[2];
	for (int i=0;i<6;i++){//wielokat podparcia na nieparzystych
		vertices[i][0]=feet[i].getElement(1,4); 
		vertices[i][1]=feet[i].getElement(2,4);
	}
	float Cx,Cy;
	computePolygonCentroid(vertices, 6, &Cx, &Cy);
	float movement[2];
	movement[0]=Cx-body->getElement(1,4);
	movement[1]=Cy-body->getElement(2,4);
	for (int i=0;i<2;i++){
		if ((abs(movement[i])>distance)&&(movement[i]>0)) movement[i]=distance;
		if ((abs(movement[i])>distance)&&(movement[i]<0)) movement[i]=-distance;
	}
	body->setElement(body->getElement(1,4)+movement[0],1,4);
	body->setElement(body->getElement(2,4)+movement[1],2,4);
}

///move body closer to stability center
void CRobot_RC::increaseStabilityMarginSwing(CPunctum *body, CPunctum * feet, float distance){
/*	float ** vertices;
	vertices = new float *[6];
	for (int i=0;i<6;i++)
		vertices[i]=new float[2];
	int ilokat=0;
	for (int i=0;i<6;i++){//wielokat podparcia na nieparzystych
		if (feet[i].isFoothold()){
			vertices[ilokat][0]=feet[i].getElement(1,4); 
			vertices[ilokat][1]=feet[i].getElement(2,4);
			ilokat++;
		}
	}
	float Cx,Cy;
	computePolygonCentroid(vertices, ilokat, &Cx, &Cy);*/
	CPunctum body_best, body_temp;
	body_best=*body;
	CPunctum C0;
	float fit;
	float best_marg = computeStabilityMargin(*body,feet);
	for (int i=-3;i<3;i++){
		C0.createTRMatrix(0,0,0,0.015*i,0,0);
		body_temp = (*body) * C0;
		fit = computeStabilityMargin(body_temp,feet);
		if (fit>best_marg){
			best_marg=fit;
			body_best=body_temp;
		}
	}
	
	//CPunctum C = body->inv(*body)*C0;
	//C.setElement(0.0,2,4);
	//* body = (* body)*C;
	*body = body_best;
}

///computes kinematic margin
float CRobot_RC::computeKinematicMargin(CPunctum body, CPunctum * feet){
	float margin=1e10;
	float marg;
	CPunctum foot_pos, tmp;
	int part;
	for (int i=0;i<6;i++){
		tmp = body * leg[i].start;
		foot_pos = tmp.inv()*feet[i];
		if (i<3) part=1;
		else part=-1;
		marg = leg[i].computeKinematicMarginPos(foot_pos.getElement(1,4),foot_pos.getElement(2,4),foot_pos.getElement(3,4),part,0);

		if (marg<margin)
			margin=marg;
	}
	return margin;
}

///computes kinematic margin
float CRobot_RC::computeKinematicMarginApprox(CPunctum body, CPunctum * feet, bool only_stance){
	float margin=1e10;
	float marg;
	for (int i=0;i<6;i++){
		if (!(only_stance&&!feet[i].isFoothold())){
			marg = computeKinematicMarginApprox(&body, &feet[i], i);
			if (marg<margin)
				margin=marg;
		}
	}
	if ((margin>1)||(margin<0))
		margin=0;
	return margin;
}

///computes kinematic margin for single leg
float CRobot_RC::computeKinematicMarginApprox(CPunctum * body, CPunctum * foot, int leg_no){
	float marg;
	CPunctum foot_pos, tmp;
	float angles[3];
	int part;
	tmp = (*body) * this->leg[leg_no].start;
	tmp.invThis();
	foot_pos = tmp*(*foot);
	if (leg_no<3) part=1;
	else part=-1;
	if (!leg[leg_no].inverse_kinematic(foot_pos.getElement(1,4),foot_pos.getElement(2,4),foot_pos.getElement(3,4),part,angles))
		return 0;
	return leg[leg_no].computeKinematicMarginApprox(angles[0], angles[1], angles[2],part,0);
}

/// zmienia katy w serwomechanizmach
bool CRobot_RC::changeAngles(double * angles, float speed){
	float max_delta_angle;
	float delta_angles[18];
	for (int i=0;i<18;i++){
		delta_angles[i]=angles[i];
		delta_angles[i]=delta_angles[i]-leg[int(i/3)].getAngle(i%3);
	}
	findAbsMax(delta_angles, 18, &max_delta_angle);//znajdujemy najwiekszy kat
	float delta_t; //czas w jakim zostanie wykonany ruch
	float real_speed = speed*MAX_SERVO_SPEED_RC; //rzeczywista, zmniejszona predkosc serwomechanizmu
	bool end=true;
	delta_t = max_delta_angle/real_speed;//czas spedzony na poruszanie sie z maksymalna predkoscia
	//!potencjalna przyczyna bledu - rzutowanie na int - robot moze pokonywac mniejsza odleglosc od zadanej
	int iterator =(int)(delta_t/SERVO_DELAY_RC)+1; // znajdujemy liczbe iteracji potrzebnych na wykonanie ruchu
	for (int i=0; i<iterator;i++){//
		for (int j=0; j<6;j++){
			for (int k=0; k<3;k++){
				leg[j].setAngle(k,leg[j].getAngle(k)+(delta_angles[j*3+k]/float(iterator)));
			}
		}
		sendAngles(speed);
		this->sleepODE(SERVO_DELAY_RC*300);
	}
	return true;
}

/// przesuwa stope o zadana odleglosc w ukladzie nogi
bool CRobot_RC::changeFoot(float x, float y, float z, unsigned char leg_no, float speed){
	int part;
	if (leg_no<3) part=1;
	else part=0;
	float delta_t; //czas w jakim zostanie wykonany ruch
	float delta_angle[3]; //zmiany katow w stawach konczyny
	float real_speed = speed*MAX_SERVO_SPEED_RC; //rzeczywista, zmniejszona predkosc serwomechanizmu
	if (!leg[leg_no].computeMoveDeltaAngle(x, y, z, part, delta_angle))
		return false; //obliczenie drogi
	float max_delta_angle;
	findAbsMax(delta_angle, 3, &max_delta_angle);//znajdujemy najwiekszy kat
	delta_t = max_delta_angle/real_speed;
	int iterator = (int)(delta_t/SERVO_DELAY_RC); // znajdujemsetServoy liczbe iteracji potrzebnych na wykonanie ruchu
	for (int i=1; i<=iterator;i++){
		if (!leg[leg_no].moveFoot(x/(float) iterator, y/(float) iterator, z/(float) iterator, part))
			return false;
		sendAngles(speed);
		this->sleepODE(SERVO_DELAY_RC*1000);
	}
	return true;
}

/// przesuwa wszystkie stopy o zadane odleglosci w ukladzie nogi
bool CRobot_RC::changeAllfeet(float * x, float * y, float * z, float speed){
	int part;
	float delta_t; //czas w jakim zostanie wykonany ruch
	float delta_angle[18]; //zmiany katow w stawach dla wszystkich konczyn
	float real_speed = speed*MAX_SERVO_SPEED_RC; //rzeczywista, zmniejszona predkosc serwomechanizmu
	for (int i=0; i<6;i++){//znalezienie maksymalnego delta angle
		if (i<3) part=1;
		else part=0;
		if (!leg[i].computeMoveDeltaAngle(x[i], y[i], z[i], part, &delta_angle[i*3])) //obliczenie drogi
			return false;
	}
	float max_delta_angle;
	findAbsMax(delta_angle, 18, &max_delta_angle);//znajdujemy najwiekszy kat
	delta_t = max_delta_angle/real_speed;
	int iterator = (int)(delta_t/SERVO_DELAY_RC); // znajdujemy liczbe iteracji potrzebnych na wykonanie ruchu
	if (iterator==0) iterator=1;
	for (int i=1; i<=iterator;i++){
		for (int j=0;j<6;j++){
			if (j<3) part=1;
			else part=-1;
			if (!leg[j].moveFoot(x[j]/(float) iterator, y[j]/(float) iterator, z[j]/(float) iterator, part))
				return false;
		}
		sendAngles(speed);
		this->sleepODE(SERVO_DELAY_RC*1000);
	//		this->sleepODE(5000);
	}
//	this->sleepODE(4);
	return true;
}

/// przesuwa stope o zadana odleglosc w ukladzie robota
bool CRobot_RC::changeFootRobot(float x, float y, float z, unsigned char leg_no, float speed){
	if (leg_no<3) {if (!changeFoot(x, y, z, leg_no, speed)) return false;}
	else {if (!changeFoot(-x, y, -z, leg_no, speed)) return false;}
	return true;
}

/// przesuwa wszystkie stopy o zadane odleglosci w ukladzie robota
bool CRobot_RC::changeAllfeetRobot(float * x, float * y, float * z, float speed){
    float x_leg[6],y_leg[6], z_leg[6];
    CPunctum movement;//ruch w ukladzie robota
    for (int i=0; i<3;i++){//obliczenie ruchu w ukladzie konczyny, pomijamy obliczenia na macierzach ze wzgledu na szybkosc dzialania
		x_leg[i]=x[i];
		y_leg[i]=y[i];
		z_leg[i]=z[i];
    }
    for (int i=3; i<6;i++){//obliczenie ruchu w ukladzie konczyny, pomijamy obliczenia na macierzach ze wzgledu na szybkosc dzialania
		x_leg[i]=-x[i];
		y_leg[i]=y[i];
		z_leg[i]=-z[i];
    }
    if (!changeAllfeet(x_leg, y_leg, z_leg, speed)) 
		return false;//wysterowanie wszystkich konczyn
	return true;
}

/// stawia stopy na podlodze - opuszcza do momentu uzyskania kontaktu
bool CRobot_RC::Placefeet(float dz, int legs, float speed){
	float x_leg[6]={0,0,0,0,0,0};
	float y_leg[6]={0,0,0,0,0,0};
	float z_leg[6]={0,0,0,0,0,0};
    CPunctum movement;//ruch w ukladzie robota
	float moove[3]={dz,dz,dz};
	do {
		for (int i=legs; i<6;i+=2){//obliczenie ruchu w ukladzie konczyny, pomijamy obliczenia na macierzach ze wzgledu na szybkosc dzialania
			if (dynamicWorld->robotODE.getContact(i)==1) moove[int(i/2.0)]=0;
			x_leg[i]=0;
			y_leg[i]=0;
			if (i<3)
				z_leg[i]=-moove[int(i/2.0)];
			else
				z_leg[i]=moove[int(i/2.0)];
		}
		if (!changeAllfeet(x_leg, y_leg, z_leg, speed)) 
			return false;//wysterowanie wszystkich konczyn
	} while (!((moove[0]==0)&&(moove[1]==0)&&(moove[2]==0)));
	return true;
}

/// stawia stopy na podlodze - opuszcza do momentu uzyskania kontaktu
bool CRobot_RC::Placefeet(float dz, float speed){
	float x_leg[6]={0,0,0,0,0,0};
	float y_leg[6]={0,0,0,0,0,0};
	float z_leg[6]={0,0,0,0,0,0};
    CPunctum movement;//ruch w ukladzie robota
	float moove[6]={dz,dz,dz,dz,dz,dz};
	do {
		for (int i=0; i<6;i++){//obliczenie ruchu w ukladzie konczyny, pomijamy obliczenia na macierzach ze wzgledu na szybkosc dzialania
			if (dynamicWorld->robotODE.getContact(i)==1) moove[i]=0;
			x_leg[i]=0;
			y_leg[i]=0;
			if (i<3)
				z_leg[i]=-moove[i];
			else
				z_leg[i]=moove[i];
		}
		if (!changeAllfeet(x_leg, y_leg, z_leg, speed)) 
			return false;//wysterowanie wszystkich konczyn
	} while (!((moove[0]==0)&&(moove[1]==0)&&(moove[2]==0)&&(moove[3]==0)&&(moove[4]==0)&&(moove[5]==0)));
	return true;
}

/// przesuwa platforme o zadana odleglosc liniowa i katowa w aktualnym ukladzie robota
bool CRobot_RC::changePlatformRobot(float x, float y, float z, float alpha, float beta, float gamma, float speed, int accel){
	float delta_t; //czas w jakim zostanie wykonany ruch
	float delta_angle[18]; //zmiany katow w stawach konczyny
	float real_speed = speed*MAX_SERVO_SPEED_RC; //rzeczywista, zmniejszona predkosc serwomechanizmu
	if (!computeRobotRelativeKinematicDeltaAngle(x, y, z, alpha, beta, gamma, delta_angle))
		return false;//obliczenie drogi katowej
	float max_delta_angle;
	findAbsMax(delta_angle, 18, &max_delta_angle);//znajdujemy najwiekszy kat
	bool end=true;
	do {
		float s_increase = (0.5*real_speed)*(accel-1)*SERVO_DELAY_RC;//droga pokonana podczas rozpedzania lub hamowania
		if (s_increase*2<max_delta_angle) {//jezeli droga pokonana podczas rozpedzania i chamowania jest mniejsza od calej drogi - profil trapezowy
			delta_t = (max_delta_angle-s_increase*2)/real_speed;//czas spedzony na poruszanie sie z maksymalna predkoscia
			end = false;
		}
		else
			accel-=1;
	} while(end);
	//!potencjalna przyczyna bledu - rzutowanie na int - robot moze pokonywac mniejsza odleglosc od zadanej
	int iterator = 2*(accel-1)+(int)(delta_t/SERVO_DELAY_RC); // znajdujemy liczbe iteracji potrzebnych na wykonanie ruchu
	float accel_speed=speed/float(accel);
	float s=0; float delta_s=0;
//	double deltay=0;
	for (int i=0; i<accel-1;i++){//rozpedzanie
		s=accel_speed*MAX_SERVO_SPEED_RC*SERVO_DELAY_RC;//droga pokanana przez serwo, ktore ma najdluzszy ruch do pokonania
		if (!robotRelativeKinematic(x*s/max_delta_angle, y*s/max_delta_angle, z*s/max_delta_angle, alpha*s/max_delta_angle, beta*s/max_delta_angle, gamma*s/max_delta_angle, delta_angle))
			return false;
		setLegs(delta_angle);
		sendAngles(accel_speed);
		this->sleepODE(SERVO_DELAY_RC*1000);
		accel_speed+=speed/float(accel);
		delta_s+=s;
//		deltay+=y*s/max_delta_angle;
	}
	if (iterator-2*(accel-1)==0) iterator++;//aby przesunac o brakujaca reszte z dzielenie (problem z rzutowaniem int)
	float dividor = (max_delta_angle-2*delta_s)/float(iterator-2*(accel-1));//rozwiazanie problemu rzutowania na int
	for (int i=0; i<iterator-2*(accel-1);i++){//MAX_SPEED_RC
		s=accel_speed*MAX_SERVO_SPEED_RC*SERVO_DELAY_RC;//droga pokanana przez serwo, ktore ma najdluzszy ruch do pokonania
		if (!robotRelativeKinematic(x*dividor/max_delta_angle, y*dividor/max_delta_angle, z*dividor/max_delta_angle, alpha*dividor/max_delta_angle, beta*dividor/max_delta_angle, gamma*dividor/max_delta_angle, delta_angle))
			return false;
		setLegs(delta_angle);
		sendAngles(accel_speed);
		this->sleepODE(SERVO_DELAY_RC*1000);
//		deltay+=y*dividor/max_delta_angle;
	}
	for (int i=0; i<accel-1;i++){//hamowanie
		accel_speed-=speed/float(accel);
		s=accel_speed*MAX_SERVO_SPEED_RC*SERVO_DELAY_RC;//droga pokanana przez serwo, ktore ma najdluzszy ruch do pokonania
		if (!robotRelativeKinematic(x*s/max_delta_angle, y*s/max_delta_angle, z*s/max_delta_angle, alpha*s/max_delta_angle, beta*s/max_delta_angle, gamma*s/max_delta_angle, delta_angle))
			return false;
		setLegs(delta_angle);
		sendAngles(accel_speed);
		this->sleepODE(SERVO_DELAY_RC*1000);
//		deltay+=y*s/max_delta_angle;
	}
	return true;
}

/// przesuwa platforme o zadana odleglosc liniowa i katowa w aktualnym ukladzie robota poslugujac sie nogami parzystymi lub nieparzystymi
bool CRobot_RC::changePlatformRobotTripod(float x, float y, float z, float alpha, float beta, float gamma, int even, float speed, int accel){
	float _x[6]={0,0,0,0,0,0};
	float _y[6]={0,0,0,0,0,0};
	float _z[6]={0,0,0,0,0,0};
	float _alpha[6]={0,0,0,0,0,0};
	float _beta[6]={0,0,0,0,0,0};
	float _gamma[6]={0,0,0,0,0,0};
	for (int i=even;i<6;i+=2){
		_x[i]=x; _y[i]=y; _z[i]=z;
		_alpha[i]=alpha; _beta[i]=beta; _gamma[i]=gamma;
	}
	if (!changePlatformRobot(_x, _y, _z, _alpha, _beta, _gamma, speed, accel))
		return false;
	return true;
}

/// przesuwa platforme o zadana odleglosc liniowa i katowa w aktualnym ukladzie robota poslugujac sie nogami ktore znajduja sie na podlozu
bool CRobot_RC::changePlatformRobotSense(float x, float y, float z, float alpha, float beta, float gamma, float speed, int accel){
	float _x[6]={0,0,0,0,0,0};
	float _y[6]={0,0,0,0,0,0};
	float _z[6]={0,0,0,0,0,0};
	float _alpha[6]={0,0,0,0,0,0};
	float _beta[6]={0,0,0,0,0,0};
	float _gamma[6]={0,0,0,0,0,0};
	for (int i=0;i<6;i++){
		if (dynamicWorld->robotODE.getContact(i)==1){
			_x[i]=x; _y[i]=y; _z[i]=z;
			_alpha[i]=alpha; _beta[i]=beta; _gamma[i]=gamma;
		}
	}
	if (!changePlatformRobot(_x, _y, _z, _alpha, _beta, _gamma, speed, accel))
		return false;
	return true;
}

/// przesuwa platforme o zadana odleglosc liniowa i katowa w aktualnym ukladzie robota (kazda noga moze zadawac inny kierunek)
bool CRobot_RC::changePlatformRobot(float * x, float * y, float * z, float * alpha, float * beta, float * gamma, float speed, int accel){
	float delta_t; //czas w jakim zostanie wykonany ruch
	float delta_angle[18]; //zmiany katow w stawach konczyny
	float real_speed = speed*MAX_SERVO_SPEED_RC; //rzeczywista, zmniejszona predkosc serwomechanizmu
	if (!computeRobotRelativeKinematicDeltaAngle(x, y, z, alpha, beta, gamma, delta_angle))
		return false;//obliczenie drogi katowej
	float max_delta_angle;
	findAbsMax(delta_angle, 18, &max_delta_angle);//znajdujemy najwiekszy kat
	bool end=true;
	do {
		float s_increase = (0.5*real_speed)*(accel-1)*SERVO_DELAY_RC;//droga pokonana podczas rozpedzania lub hamowania
		if (s_increase*2<max_delta_angle) {//jezeli droga pokonana podczas rozpedzania i chamowania jest mniejsza od calej drogi - profil trapezowy
			delta_t = (max_delta_angle-s_increase*2)/real_speed;//czas spedzony na poruszanie sie z maksymalna predkoscia
			end = false;
		}
		else
			accel-=1;
	} while(end);
	//!potencjalna przyczyna bledu - rzutowanie na int - robot moze pokonywac mniejsza odleglosc od zadanej
	int iterator = 2*(accel-1)+(int)(delta_t/SERVO_DELAY_RC); // znajdujemy liczbe iteracji potrzebnych na wykonanie ruchu
	float accel_speed=speed/float(accel);
	float s=0; float delta_s=0;
	float x_zad[6],y_zad[6],z_zad[6],alpha_zad[6],beta_zad[6],gamma_zad[6];
	for (int i=0; i<accel-1;i++){//rozpedzanie
		s=accel_speed*MAX_SERVO_SPEED_RC*SERVO_DELAY_RC;//droga pokanana przez serwo, ktore ma najdluzszy ruch do pokonania
		for (int j=0; j<6;j++){// krok ruchu dla kazdego wezla kinematycznego
			x_zad[j]=x[j]*s/max_delta_angle; y_zad[j]=y[j]*s/max_delta_angle; z_zad[j]=z[j]*s/max_delta_angle;
			alpha_zad[j]=alpha[j]*s/max_delta_angle; beta_zad[j]=beta[j]*s/max_delta_angle; gamma_zad[j]=gamma[j]*s/max_delta_angle;
		}
		if (!robotRelativeKinematic(x_zad, y_zad, z_zad, alpha_zad, beta_zad, gamma_zad, delta_angle))
			return false;
		setLegs(delta_angle);
		sendAngles(accel_speed);
		this->sleepODE(SERVO_DELAY_RC*1000);
		accel_speed+=speed/float(accel);
		delta_s+=s;
	}
	if (iterator-2*(accel-1)==0) iterator++;//aby przesunac o brakujaca reszte z dzielenie (problem z rzutowaniem int)
	float dividor = (max_delta_angle-2*delta_s)/float(iterator-2*(accel-1));//rozwiazanie problemu rzutowania na int
	for (int i=0; i<iterator-2*(accel-1);i++){//MAX_SPEED_RC
		for (int j=0; j<6;j++){// krok ruchu dla kazdego wezla kinematycznego
			x_zad[j]=x[j]*dividor/max_delta_angle; y_zad[j]=y[j]*dividor/max_delta_angle; z_zad[j]=z[j]*dividor/max_delta_angle;
			alpha_zad[j]=alpha[j]*dividor/max_delta_angle; beta_zad[j]=beta[j]*dividor/max_delta_angle; gamma_zad[j]=gamma[j]*dividor/max_delta_angle;
		}
		if (!robotRelativeKinematic(x_zad, y_zad, z_zad, alpha_zad, beta_zad, gamma_zad, delta_angle))
			return false;
		setLegs(delta_angle);
		sendAngles(accel_speed);
		this->sleepODE(SERVO_DELAY_RC*1000);
	}
	for (int i=0; i<accel-1;i++){//hamowanie
		accel_speed-=speed/float(accel);
		s=accel_speed*MAX_SERVO_SPEED_RC*SERVO_DELAY_RC;//droga pokanana przez serwo, ktore ma najdluzszy ruch do pokonania
		for (int j=0; j<6;j++){// krok ruchu dla kazdego wezla kinematycznego
			x_zad[j]=x[j]*s/max_delta_angle; y_zad[j]=y[j]*s/max_delta_angle; z_zad[j]=z[j]*s/max_delta_angle;
			alpha_zad[j]=alpha[j]*s/max_delta_angle; beta_zad[j]=beta[j]*s/max_delta_angle; gamma_zad[j]=gamma[j]*s/max_delta_angle;
		}
		if (!robotRelativeKinematic(x_zad, y_zad, z_zad, alpha_zad, beta_zad, gamma_zad, delta_angle))
			return false;
		setLegs(delta_angle);
		sendAngles(accel_speed);
		this->sleepODE(SERVO_DELAY_RC*1000);
	}	return true;
}

/// przesuwa platforme o zadana odleglosc liniowa i katowa w aktualnym ukladzie robota (kazda noga moze zadawac inny kierunek)
/// konczyny, ktore sa w powietrzu poruszaja sie wzgledem pozycji neutralnej
bool CRobot_RC::changePlatformRobotNeutral(float * x, float * y, float * z, float * alpha, float * beta, float * gamma,float foot_up, int legs, float speed, int accel){
	float delta_t; //czas w jakim zostanie wykonany ruch
	float delta_angle[18]; //zmiany katow w stawach konczyny
	float real_speed = speed*MAX_SERVO_SPEED_RC; //rzeczywista, zmniejszona predkosc serwomechanizmu
	if (!computeRobotRelativeKinematicDeltaAngleNeutral(x, y, z, alpha, beta, gamma, delta_angle,foot_up,legs)) //obliczenie drogi katowej
		return false;
	float max_delta_angle;
	findAbsMax(delta_angle, 18, &max_delta_angle);//znajdujemy najwiekszy kat
	bool end=true;
	do {
		float s_increase = (0.5*real_speed)*(accel-1)*SERVO_DELAY_RC;//droga pokonana podczas rozpedzania lub hamowania
		if (s_increase*2<max_delta_angle) {//jezeli droga pokonana podczas rozpedzania i chamowania jest mniejsza od calej drogi - profil trapezowy
			delta_t = (max_delta_angle-s_increase*2)/real_speed;//czas spedzony na poruszanie sie z maksymalna predkoscia
			end = false;
		}
		else
			accel-=1;
	} while(end);
	//!potencjalna przyczyna bledu - rzutowanie na int - robot moze pokonywac mniejsza odleglosc od zadanej
	int iterator = 2*(accel-1)+(int)(delta_t/SERVO_DELAY_RC); // znajdujemy liczbe iteracji potrzebnych na wykonanie ruchu
	float accel_speed=speed/float(accel);
	float s=0; float delta_s=0;
	float x_zad[6],y_zad[6],z_zad[6],alpha_zad[6],beta_zad[6],gamma_zad[6];
	for (int i=0; i<accel-1;i++){//rozpedzanie
		s=accel_speed*MAX_SERVO_SPEED_RC*SERVO_DELAY_RC;//droga pokanana przez serwo, ktore ma najdluzszy ruch do pokonania
		for (int j=0; j<6;j++){
			for (int k=0; k<3;k++){
				leg[j].setAngle(k,leg[j].getAngle(k)+(delta_angle[j*3+k]*s/max_delta_angle));
			}
		}
		/*for (int j=0; j<6;j++){// krok ruchu dla kazdego wezla kinematycznego
			x_zad[j]=x[j]*s/max_delta_angle; y_zad[j]=y[j]*s/max_delta_angle; z_zad[j]=z[j]*s/max_delta_angle;
			alpha_zad[j]=alpha[j]*s/max_delta_angle; beta_zad[j]=beta[j]*s/max_delta_angle; gamma_zad[j]=gamma[j]*s/max_delta_angle;
		}
		if (!robotRelativeKinematic(x_zad, y_zad, z_zad, alpha_zad, beta_zad, gamma_zad,delta_angle))
			return false;*/
//		setLegs(delta_angle);
		sendAngles(accel_speed);
		this->sleepODE(SERVO_DELAY_RC*1000);
		accel_speed+=speed/float(accel);
		delta_s+=s;
	}
	if (iterator-2*(accel-1)==0) iterator++;//aby przesunac o brakujaca reszte z dzielenie (problem z rzutowaniem int)
	float dividor = (max_delta_angle-2*delta_s)/float(iterator-2*(accel-1));//rozwiazanie problemu rzutowania na int
	for (int i=0; i<iterator-2*(accel-1);i++){//MAX_SPEED_RC
		for (int j=0; j<6;j++){
			for (int k=0; k<3;k++){
				leg[j].setAngle(k,leg[j].getAngle(k)+delta_angle[j*3+k]*dividor/max_delta_angle);
			}
		}
	/*	for (int i=0; i<6;i++){// krok ruchu dla kazdego wezla kinematycznego
			x_zad[i]=x[i]*dividor/max_delta_angle; y_zad[i]=y[i]*dividor/max_delta_angle; z_zad[i]=z[1]*dividor/max_delta_angle;
			alpha_zad[i]=alpha[i]*dividor/max_delta_angle; beta_zad[i]=beta[i]*dividor/max_delta_angle; gamma_zad[i]=gamma[i]*dividor/max_delta_angle;
		}
		if (!robotRelativeKinematic(x_zad, y_zad, z_zad, alpha_zad, beta_zad, gamma_zad, delta_angle))
			return false;
		setLegs(delta_angle);*/
		sendAngles(accel_speed);
		this->sleepODE(SERVO_DELAY_RC*1000);
	}
	for (int i=0; i<accel-1;i++){//hamowanie
		accel_speed-=speed/float(accel);
		s=accel_speed*MAX_SERVO_SPEED_RC*SERVO_DELAY_RC;//droga pokanana przez serwo, ktore ma najdluzszy ruch do pokonania
		for (int j=0; j<6;j++){
			for (int k=0; k<3;k++){
				leg[j].setAngle(k,leg[j].getAngle(k)+(delta_angle[j*3+k]*s/max_delta_angle));
			}
		}
	/*	for (int i=0; i<6;i++){// krok ruchu dla kazdego wezla kinematycznego
			x_zad[i]=x[i]*s/max_delta_angle; y_zad[i]=y[i]*s/max_delta_angle; z_zad[i]=z[1]*s/max_delta_angle;
			alpha_zad[i]=alpha[i]*s/max_delta_angle; beta_zad[i]=beta[i]*s/max_delta_angle; gamma_zad[i]=gamma[i]*s/max_delta_angle;
		}
		if (!robotRelativeKinematic(x_zad, y_zad, z_zad, alpha_zad, beta_zad, gamma_zad, delta_angle))
			return false;
		setLegs(delta_angle);*/
		sendAngles(accel_speed);
		this->sleepODE(SERVO_DELAY_RC*1000);
	}
	this->sleepODE(20);
	return true;
}

//przemieszcza robota do zadanych pozycji w ukladzie globalnym
bool CRobot_RC::move2GlobalPosition(CPunctum body_prev, CPunctum body, CPunctum * feet_prev, CPunctum * feet, float speed, int soft){
	CPunctum pos[6];
	double angles[18];
	int part;
	getFullRobotState(&body_prev, feet_prev);
	for (int i=0;i<6;i++) {
	    if (i<3) part=1;
	    else part=-1;
		CPunctum delta,delta1, tmp;
		delta1 = leg[i].forward_kinematic(getAngle(i,0),getAngle(i,1),getAngle(i,2),part);
		tmp = body*leg[i].start;
		tmp.invThis();
		if (feet[i].isFoothold())
			pos[i]=tmp*feet_prev[i];
		else
			pos[i]=tmp*feet[i];
		for (int j=0;j<5;j++){
			int success=0;
			if ((soft)&&(!leg[i].inverse_kinematic(pos[i].getElement(1,4),pos[i].getElement(2,4),pos[i].getElement(3,4),part,&angles[i*3])))	{
				if (!feet[i].isFoothold()){
					for (int r=1;r<5;r++){
						for (int theta=0;theta<8;theta++){
							for (int phi=0;phi<8;phi++){
								if (leg[i].inverse_kinematic(pos[i].getElement(1,4)+r*0.01*sin(theta*0.785)*cos(phi*0.785),pos[i].getElement(2,4)+r*0.01*sin(theta*0.785)*sin(phi*0.785),pos[i].getElement(3,4)+r*0.005*cos(theta*0.785),part,&angles[i*3])){
									success = 1;				
									break;
								}
							}
							if (success==1)
								break;
						}
						if (success==1)
							break;
					}
					if (success==0)return false;
				}
				else {
					body.setElement(body.getElement(3,4)-0.01,3,4);
					tmp = body*leg[i].start;
					tmp.invThis();
					pos[i]=tmp*feet_prev[i];
				}
			}
			else {
				break;
			}
			if (j==4) return false;
			if (success==1) break;
		}
	}
	changeAngles(angles,speed);
}

/// przesuwa platforme o zadana odleglosc liniowa i katowa wzgledem konfiguracji neutralnej
/// uwaga nie używać naprzemiennie wersji jedno i wielowymiarowej - nie będzie dzialalo poprawnie
/// ze wzgledu na brak znajomosci poprzedniej pozycji platformy (TODO) lub nalezy konczyc ruch w pozycji neutralnej
bool CRobot_RC::changePlatform(float x, float y, float z, float alpha, float beta, float gamma, float speed, int accel){
	float delta_t; //czas w jakim zostanie wykonany ruch
	float delta_angle[18]; //zmiany katow w stawach konczyny
	float real_speed = speed*MAX_SERVO_SPEED_RC; //rzeczywista, zmniejszona predkosc serwomechanizmu
	if (!computeRobotKinematicDeltaAngle(x, y, z, alpha, beta, gamma, delta_angle))
		return false;//obliczenie drogi katowej
	float max_delta_angle;
	findAbsMax(delta_angle, 18, &max_delta_angle);//znajdujemy najwiekszy kat
	bool end=true;
	do {
		float s_increase = (0.5*real_speed)*(accel-1)*SERVO_DELAY_RC;//droga pokonana podczas rozpedzania lub hamowania
		if (s_increase*2<max_delta_angle) {//jezeli droga pokonana podczas rozpedzania i chamowania jest mniejsza od calej drogi - profil trapezowy
			delta_t = (max_delta_angle-s_increase*2)/real_speed;//czas spedzony na poruszanie sie z maksymalna predkoscia
			end = false;
		}
		else
			accel-=1;
	} while(end);
	//!potencjalna przyczyna bledu - rzutowanie na int - robot moze pokonywac mniejsza odleglosc od zadanej
	int iterator = 2*(accel-1)+(int)(delta_t/SERVO_DELAY_RC); // znajdujemy liczbe iteracji potrzebnych na wykonanie ruchu
	float accel_speed=speed/float(accel);
	float s=0; float delta_s=0;
	for (int i=0; i<accel-1;i++){//rozpedzanie
		s+=accel_speed*MAX_SERVO_SPEED_RC*SERVO_DELAY_RC;//droga pokanana przez serwo, ktore ma najdluzszy ruch do pokonania
		if (!robotKinematic((x*s/max_delta_angle)+(1-(s/max_delta_angle))*robot_platform_pos[0], (y*s/max_delta_angle)+(1-(s/max_delta_angle))*robot_platform_pos[1], (z*s/max_delta_angle)+(1-(s/max_delta_angle))*robot_platform_pos[2], (alpha*s/max_delta_angle)+(1-(s/max_delta_angle))*robot_platform_pos[3], (beta*s/max_delta_angle)+(1-(s/max_delta_angle))*robot_platform_pos[4], (gamma*s/max_delta_angle)+(1-(s/max_delta_angle))*robot_platform_pos[5], delta_angle))
			return false;
		setLegs(delta_angle);
		sendAngles(accel_speed);
		this->sleepODE(SERVO_DELAY_RC*1000);
		accel_speed+=speed/float(accel);
	}
	if (iterator-2*(accel-1)==0) iterator++;//aby przesunac o brakujaca reszte z dzielenie (problem z rzutowaniem int)
	float dividor = (max_delta_angle-2*s)/float(iterator-2*(accel-1));//rozwiazanie problemu rzutowania na int
	for (int i=0; i<iterator-2*(accel-1);i++){//MAX_SPEED_RC
		s+=dividor;//droga pokanana przez serwo, ktore ma najdluzszy ruch do pokonania
		if (!robotKinematic(x*s/max_delta_angle+(1-(s/max_delta_angle))*robot_platform_pos[0], (y*s/max_delta_angle)+(1-(s/max_delta_angle))*robot_platform_pos[1], (z*s/max_delta_angle)+(1-(s/max_delta_angle))*robot_platform_pos[2], (alpha*s/max_delta_angle)+(1-(s/max_delta_angle))*robot_platform_pos[3], (beta*s/max_delta_angle)+(1-(s/max_delta_angle))*robot_platform_pos[4], (gamma*s/max_delta_angle)+(1-(s/max_delta_angle))*robot_platform_pos[5], delta_angle))
			return false;
		setLegs(delta_angle);
		sendAngles(accel_speed);
		this->sleepODE(SERVO_DELAY_RC*1000);
	}
	for (int i=0; i<accel-1;i++){//hamowanie
		accel_speed-=speed/float(accel);
		s+=accel_speed*MAX_SERVO_SPEED_RC*SERVO_DELAY_RC;//droga pokanana przez serwo, ktore ma najdluzszy ruch do pokonania
		if (!robotKinematic(x*s/max_delta_angle+(1-(s/max_delta_angle))*robot_platform_pos[0], (y*s/max_delta_angle)+(1-(s/max_delta_angle))*robot_platform_pos[1], (z*s/max_delta_angle)+(1-(s/max_delta_angle))*robot_platform_pos[2], (alpha*s/max_delta_angle)+(1-(s/max_delta_angle))*robot_platform_pos[3], (beta*s/max_delta_angle)+(1-(s/max_delta_angle))*robot_platform_pos[4], (gamma*s/max_delta_angle)+(1-(s/max_delta_angle))*robot_platform_pos[5], delta_angle))
			return false;
		setLegs(delta_angle);
		sendAngles(accel_speed);
		this->sleepODE(SERVO_DELAY_RC*1000);
	}
	robot_platform_pos[0]=x; robot_platform_pos[1]=y; robot_platform_pos[2]=z;
	robot_platform_pos[3]=alpha; robot_platform_pos[4]=beta; robot_platform_pos[5]=gamma;
	return true;
}

/// stabilizuje platforme na podstawie czujnikow z imu
bool CRobot_RC::stabilizePlatform(float speed, int accel){
	float imu_rot[3];
	getRotAngles(imu_rot);
	//printf("stab %f %f\n", imu_rot[0],imu_rot[1]);
	if (!changePlatformRobot(0,0,0, -imu_rot[0],-imu_rot[1],0, speed, accel))
	  return false;
	return true;
}

/// przesuwa platforme o zadana odleglosc liniowa i katowa wzgledem konfiguracji neutralnej
/// uwaga nie używać naprzemiennie wersji jedno i wielowymiarowej - nie będzie dzialalo poprawnie
/// ze wzgledu na brak znajomosci poprzedniej pozycji platformy (TODO) lub nalezy konczyc ruch w pozycji neutralnej
bool CRobot_RC::changePlatform(float * x, float * y, float * z, float * alpha, float * beta, float * gamma, float speed, int accel){
	float delta_t; //czas w jakim zostanie wykonany ruch
	float delta_angle[18]; //zmiany katow w stawach konczyny
	float real_speed = speed*MAX_SERVO_SPEED_RC; //rzeczywista, zmniejszona predkosc serwomechanizmu
	if (!computeRobotKinematicDeltaAngle(x, y, z, alpha, beta, gamma, delta_angle))
		return false;//obliczenie drogi katowej
	float max_delta_angle;
	findAbsMax(delta_angle, 18, &max_delta_angle);//znajdujemy najwiekszy kat
	bool end=true;
	do {
		float s_increase = (0.5*real_speed)*(accel-1)*SERVO_DELAY_RC;//droga pokonana podczas rozpedzania lub hamowania
		if (s_increase*2<max_delta_angle) {//jezeli droga pokonana podczas rozpedzania i chamowania jest mniejsza od calej drogi - profil trapezowy
			delta_t = (max_delta_angle-s_increase*2)/real_speed;//czas spedzony na poruszanie sie z maksymalna predkoscia
			end = false;
		}
		else
			accel-=1;
	} while(end);
	//!potencjalna przyczyna bledu - rzutowanie na int - robot moze pokonywac mniejsza odleglosc od zadanej
	int iterator = 2*(accel-1)+(int)(delta_t/SERVO_DELAY_RC); // znajdujemy liczbe iteracji potrzebnych na wykonanie ruchu
	float accel_speed=speed/float(accel);
	float s=0; float delta_s=0;
	float x_zad[6],y_zad[6],z_zad[6],alpha_zad[6],beta_zad[6],gamma_zad[6];
	for (int i=0; i<accel-1;i++){//rozpedzanie
		s+=accel_speed*MAX_SERVO_SPEED_RC*SERVO_DELAY_RC;//droga pokanana przez serwo, ktore ma najdluzszy ruch do pokonania
		for (int j=0; j<6;j++){// krok ruchu dla kazdego wezla kinematycznego
			x_zad[j]=(x[j]*s/max_delta_angle)+(1-(s/max_delta_angle))*x_ref_prev[j]; y_zad[j]=(y[j]*s/max_delta_angle)+(1-(s/max_delta_angle))*y_ref_prev[j]; z_zad[j]=(z[j]*s/max_delta_angle)+(1-(s/max_delta_angle))*z_ref_prev[j];
			alpha_zad[j]=(alpha[j]*s/max_delta_angle)+(1-(s/max_delta_angle))*alpha_ref_prev[j]; beta_zad[j]=(beta[j]*s/max_delta_angle)+(1-(s/max_delta_angle))*beta_ref_prev[j]; gamma_zad[j]=(gamma[j]*s/max_delta_angle)+(1-(s/max_delta_angle))*gamma_ref_prev[j];
		}
		if (!robotKinematic(x_zad, y_zad, z_zad, alpha_zad, beta_zad, gamma_zad, delta_angle))
			return false;
		setLegs(delta_angle);
		sendAngles(accel_speed);
		this->sleepODE(SERVO_DELAY_RC*1000);
		accel_speed+=speed/float(accel);
	}
	if (iterator-2*(accel-1)==0) iterator++;//aby przesunac o brakujaca reszte z dzielenie (problem z rzutowaniem int)
	float dividor = (max_delta_angle-2*s)/float(iterator-2*(accel-1));//rozwiazanie problemu rzutowania na int
	for (int i=0; i<iterator-2*(accel-1);i++){//MAX_SPEED_RC
		s+=dividor;//droga pokanana przez serwo, ktore ma najdluzszy ruch do pokonania
		for (int j=0; j<6;j++){// krok ruchu dla kazdego wezla kinematycznego
			x_zad[j]=(x[j]*s/max_delta_angle)+(1-(s/max_delta_angle))*x_ref_prev[j]; y_zad[j]=(y[j]*s/max_delta_angle)+(1-(s/max_delta_angle))*y_ref_prev[j]; z_zad[j]=(z[j]*s/max_delta_angle)+(1-(s/max_delta_angle))*z_ref_prev[j];
			alpha_zad[j]=(alpha[j]*s/max_delta_angle)+(1-(s/max_delta_angle))*alpha_ref_prev[j]; beta_zad[j]=(beta[j]*s/max_delta_angle)+(1-(s/max_delta_angle))*beta_ref_prev[j]; gamma_zad[j]=(gamma[j]*s/max_delta_angle)+(1-(s/max_delta_angle))*gamma_ref_prev[j];
		}
		if (!robotKinematic(x_zad, y_zad, z_zad, alpha_zad, beta_zad, gamma_zad, delta_angle))
			return false;
		setLegs(delta_angle);
		sendAngles(accel_speed);
		this->sleepODE(SERVO_DELAY_RC*1000);
	}
	for (int i=0; i<accel-1;i++){//hamowanie
		accel_speed-=speed/float(accel);
		s+=accel_speed*MAX_SERVO_SPEED_RC*SERVO_DELAY_RC;//droga pokanana przez serwo, ktore ma najdluzszy ruch do pokonania
		for (int j=0; j<6;j++){// krok ruchu dla kazdego wezla kinematycznego
			x_zad[j]=(x[j]*s/max_delta_angle)+(1-(s/max_delta_angle))*x_ref_prev[j]; y_zad[j]=(y[j]*s/max_delta_angle)+(1-(s/max_delta_angle))*y_ref_prev[j]; z_zad[j]=(z[j]*s/max_delta_angle)+(1-(s/max_delta_angle))*z_ref_prev[j];
			alpha_zad[j]=(alpha[j]*s/max_delta_angle)+(1-(s/max_delta_angle))*alpha_ref_prev[j]; beta_zad[j]=(beta[j]*s/max_delta_angle)+(1-(s/max_delta_angle))*beta_ref_prev[j]; gamma_zad[j]=(gamma[j]*s/max_delta_angle)+(1-(s/max_delta_angle))*gamma_ref_prev[j];
		}
		if (!robotKinematic(x_zad, y_zad, z_zad, alpha_zad, beta_zad, gamma_zad, delta_angle))
			return false;
		setLegs(delta_angle);
		sendAngles(accel_speed);
		this->sleepODE(SERVO_DELAY_RC*1000);
	}
	for (int i=0;i<6;i++){
		x_ref_prev[i]=x[i]; y_ref_prev[i]=y[i]; z_ref_prev[i]=z[i];
		alpha_ref_prev[i]=alpha[i]; beta_ref_prev[i]=beta[i]; gamma_ref_prev[i]=gamma[i];
	}
	return true;
}

///modifies global robot position
void CRobot_RC::modifyRobotPosition(float x, float y, float z, float alpha, float beta, float gamma){
  CPunctum movement;
  movement.createTRMatrix(alpha, beta, gamma, x, y, z);
  robot_global_pos = robot_global_pos * movement;
  robot_platform_pos[0]=x; robot_platform_pos[1]=y; robot_platform_pos[2]=z;
  robot_platform_pos[3]=alpha; robot_platform_pos[4]=beta; robot_platform_pos[5]=gamma;
}

//Simulate ODE for x miliseconds
void CRobot_RC::sleepODE(int miliseconds){
	int step_count=(int)(miliseconds/(dynamicWorld->stepDT/0.001));
	for (int i=1;i<step_count;i++) {
		dynamicWorld->SimStep();
	}
}

/// zwraca stan robota w postaci macierzy odpowiedzialnych z polozenie stop i korpusu
void CRobot_RC::getFullRobotState(CPunctum *body, CPunctum * feet){
	* body = this->getRobotState();
	int part;
	for (int i=0;i<6;i++){
		if (i<3) part=1; else part=-1;
		feet[i] = (*body)*leg[i].start*leg[i].getPosition(part);
	}
}

/// check collisions
bool CRobot_RC::checkCollisions(CPunctum body, CPunctum * feet){
	float Q_ref[18];
	int part;
	CPunctum pos;
	for (int i=0;i<6;i++) {
	    if (i<3) part=1;
	    else part=-1;
		pos = body*leg[i].start;
		pos.invThis();
		pos=pos*feet[i];
		if (!leg[i].inverse_kinematic(pos.getElement(1,4),pos.getElement(2,4),pos.getElement(3,4),part,&Q_ref[i*3]))
			return true;
	}
	bool collision_table[44];
	if (robot_structure->checkCollision(body.getElement(1,4),body.getElement(2,4),body.getElement(3,4),body.orientation[0],body.orientation[1],body.orientation[2], Q_ref,collision_table,false,dynamicWorld->ground->points,-1,-1)) 
		return true; //collision exist
	else
		return false;
}
