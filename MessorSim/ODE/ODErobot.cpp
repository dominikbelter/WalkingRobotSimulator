#include "ODErobot.h"
#include <math.h>
#include "../math/matrix.h"


CODERobot::CODERobot(void)
{
	imu.setInitialPosition(0.023,0,0);
	for (int i=0;i<6;i++) {
		filtered_contact[i]=0;
		ref_angles[i*3]=0;
		ref_angles[i*3+1]=(float)deg2rad((float)24);
		ref_angles[i*3+2]=(float)deg2rad((float)-114);
	}

	for (int i=0; i<18; i++)
	{
		ref_speed[i]=1;
	}

	ref_angles[0]=(float)deg2rad((float)45);
	ref_angles[6]=(float)deg2rad((float)-45);
	ref_angles[9]=(float)deg2rad((float)-45);
	ref_angles[15]=(float)deg2rad((float)45);
}

CODERobot::~CODERobot(void)
{
}

///ustawia poczatkowa pozycje i orientacje robota
void CODERobot::setInitialPosition(float x, float y, float z, float alpha, float beta, float gamma){
	imu.setInitialPosition(x,y,z,alpha,beta,gamma);
}

/// odczytuje wartosci zadane dla serwomechanizmow
float CODERobot::getAngle(int leg, int joint){
	return ref_angles[leg*3+joint];
}

/// ustawia predkosc zadana dla konczyny
void CODERobot::setLegSpeed(int leg_no, short * leg_speed){
	ref_speed[leg_no*3]=((float)leg_speed[0])/114.0;
	ref_speed[leg_no*3+1]=((float)leg_speed[1])/114.0;
	ref_speed[leg_no*3+2]=((float)leg_speed[2])/114.0;
}

/// ustawia wartosci zadane W STOPNIACH!!!
void CODERobot::setLeg(int leg_no, float * leg_deg){
	ref_angles[leg_no*3]=(float)leg_deg[0];
	ref_angles[leg_no*3+1]=(float)leg_deg[1];
	ref_angles[leg_no*3+2]=(float)leg_deg[2];
}

/// odczytuje rzeczywiste wartosci katów w stawach
void CODERobot::readLegPosition(int leg_no, short * read_angles){
	if (leg_no==4){
		read_angles[0] = (short)rad2deg((float)dJointGetHingeAngle(Joints[15])+ref_angles[15]);
		read_angles[1] = (short)rad2deg((float)dJointGetHingeAngle(Joints[16])+ref_angles[16]);
		read_angles[2] = (short)rad2deg((float)dJointGetHingeAngle(Joints[17])+ref_angles[17]);
	}
	else if (leg_no==5){
		read_angles[0] = (short)rad2deg((float)dJointGetHingeAngle(Joints[12])+ref_angles[12]);
		read_angles[1] = (short)rad2deg((float)dJointGetHingeAngle(Joints[13])+ref_angles[13]);
		read_angles[2] = (short)rad2deg((float)dJointGetHingeAngle(Joints[14])+ref_angles[14]);
	}
	else if (leg_no==6){
		read_angles[0] = (short)rad2deg((float)dJointGetHingeAngle(Joints[9])+ref_angles[9]);
		read_angles[1] = (short)rad2deg((float)dJointGetHingeAngle(Joints[10])+ref_angles[10]);
		read_angles[2] = (short)rad2deg((float)dJointGetHingeAngle(Joints[11])+ref_angles[11]);
	}
	else {
		read_angles[0] = (short)rad2deg((float)dJointGetHingeAngle(Joints[leg_no*3])+ref_angles[leg_no*3]);
		read_angles[1] = (short)rad2deg((float)dJointGetHingeAngle(Joints[leg_no*3+1])+ref_angles[leg_no*3+1]);
		read_angles[2] = (short)rad2deg((float)dJointGetHingeAngle(Joints[leg_no*3+2])+ref_angles[leg_no*3+2]);
	}
}

/// ODE - symulacja regulatora w serwomechanizmie
void CODERobot::setServo(int servo_nr, double value) {
	dReal Gain = (dReal) 55.1465;
	dReal v_max = (dReal) ref_speed[servo_nr]*max_servo_speed;
	dReal MaxForce = (dReal)13.0;

	dReal TruePosition = dJointGetHingeAngle(Joints[servo_nr]);
	dReal DesiredPosition = (dReal)value;
	dReal Error = TruePosition - DesiredPosition;

	dReal DesiredVelocity = -Error * Gain;
	if (DesiredVelocity>v_max) 
		DesiredVelocity=v_max;
	if (DesiredVelocity<-v_max) 
		DesiredVelocity=-v_max;

	dJointSetHingeParam(Joints[servo_nr], dParamVel, DesiredVelocity);

	/*
	float b=1.0;//tlumienie
	float k=10;//tlumienie
	for (int i=0;i<6;i++){
		dReal TrueVel1 = dJointGetUniversalAngle1Rate(Joints[18+i]);
		dReal TrueVel2 = dJointGetUniversalAngle2Rate(Joints[18+i]);
		dReal TruePos1 = dJointGetUniversalAngle1(Joints[18+i]);
		dReal TruePos2 = dJointGetUniversalAngle2(Joints[18+i]);
		dJointSetUniversalParam(Joints[i+18], dParamVel, -(k*TruePos1)-b*TrueVel1);
		dJointSetUniversalParam(Joints[i+18], dParamVel2,-(k*TruePos2)-b*TrueVel2);
	}*/
}

/// ODE - ustawia wartosci katow we wszystkich stawach
void CODERobot::setAllServos()
{
	setServo(0,deg2rad(45)-getAngle(0, 0));
	setServo(1,deg2rad(24)-getAngle(0, 1));
	setServo(2,deg2rad(-114)-getAngle(0, 2));
	setServo(3,deg2rad(0)-getAngle(1, 0));
	setServo(4,deg2rad(24)-getAngle(1, 1));
	setServo(5,deg2rad(-114)-getAngle(1, 2));
	setServo(6,deg2rad(-45)-getAngle(2, 0));
	setServo(7,deg2rad(24)-getAngle(2, 1));
	setServo(8,deg2rad(-114)-getAngle(2, 2));
	setServo(9,-(deg2rad(-45)-getAngle(3, 0)));
	setServo(10,-(deg2rad(24)-getAngle(3, 1)));
	setServo(11,-(deg2rad(-114)-getAngle(3, 2)));
	setServo(12,-(deg2rad(0)-getAngle(4, 0)));
	setServo(13,-(deg2rad(24)-getAngle(4, 1)));
	setServo(14,-(deg2rad(-114)-getAngle(4, 2)));
	setServo(15,-(deg2rad(45)-getAngle(5, 0)));
	setServo(16,-(deg2rad(24)-getAngle(5, 1)));
	setServo(17,-(deg2rad(-114)-getAngle(5, 2)));
}

/// ODE - pobiera rzeczywiste wartosci katow we wszystkich stawach
void CODERobot::getRealServoValues(float angles[])
{
	for (int i=0;i<18;i++) {
		if (i<9){
			if (i%3==1) angles[i]=-dJointGetHingeAngle(Joints[i])+24*3.14/180;
			else if (i%3==2) angles[i]=-dJointGetHingeAngle(Joints[i])-114*3.14/180;
			else if (i%3==0) angles[i]=-dJointGetHingeAngle(Joints[i]);
		}
		else {
			if (i%3==1) angles[i]=dJointGetHingeAngle(Joints[i])+24*3.14/180;
			else if (i%3==2) angles[i]=dJointGetHingeAngle(Joints[i])-114*3.14/180;
			else if (i%3==0) angles[i]=dJointGetHingeAngle(Joints[i]);
		}
	}
	angles[0]+=45*3.14/180;
	angles[6]-=45*3.14/180;
	angles[9]-=45*3.14/180;
	angles[15]+=45*3.14/180;
}

/// definicja robota w srodowisku ODE
void CODERobot::ODEcreateRobot(dWorldID World, dSpaceID Space, dJointGroupID jointgroup) {

	#pragma warning( disable : 4244 4305 )

	double mass_corp=0.908;
	double mass_tibia=0.163;
	double mass_femur=0.108;
	double mass_coxa=0.134;
	double mass_foot=0.05;
	
    dReal sides[3];
    dMass m;
    dMatrix3 R; // macierz rotacji
    VECTOR tempVect(0.0, 0.0, 0.0);
	double mass=mass_corp;
	double leg_ref[3]={45*PI/180,(24)*PI/180,0};
	leg_ref[2]=-(-114*PI/180)-leg_ref[1];//-(pi/2-leg_ref[1]);
	double c = leg_ref[2]*180/PI;
	float robot_position[3];
	imu.getIMUposition(robot_position);
	double offset_z=robot_position[2]; //przesuniecie pozycji robota w osi z
	double offset_x=robot_position[0]; //przesuniecie pozycji robota w osi x
	double offset_y=-robot_position[1]; //przesuniecie pozycji robota w osi y

    double promien = 0.005; //promien piguly - 3 segment nogi
    double dlugosc = segment3-2*promien; //dlugosc piguly - 2*3 segment nogi

	for (int bodies = 0;bodies<25;bodies++)
		Object[bodies].Body = dBodyCreate(World);

    // Set up for static object - rama glowna
    sides[0] = 0.03;
    sides[1] = 0.03;
    sides[2] = 0.03;
    dBodySetPosition(Object[0].Body, offset_x+0, offset_z+0, 0+offset_y);
    dBodySetLinearVel(Object[0].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 1, 0, 0, 0);
	dBodySetRotation(Object[0].Body, R);
    dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]); //ustawia gestosc dla masy dMassSetBoxTotal
    Object[0].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[0].Geom[0], Object[0].Body);
    dBodySetMass(Object[0].Body, &m);

	imu.setIMUBody(Object[0].Body, Object[0].Geom[0]);//ustawienie imu

	mass=mass_coxa;
	// Set up for static object - pierwszy czlon nogi 1
    sides[0] = 0.03;
    sides[1] = 0.03;
    sides[2] = 0.03;
    dBodySetPosition(Object[1].Body, offset_x+width_min+(segment1*cos(leg_ref[0])/2.0), offset_z, -(lenght)+offset_y-segment1*sin(leg_ref[0])/2.0);
    dBodySetLinearVel(Object[1].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 0, 1, 0, leg_ref[0]);
	dBodySetRotation(Object[1].Body, R);
    dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
    Object[1].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[1].Geom[0], Object[1].Body);
    dBodySetMass(Object[1].Body, &m);

	mass=mass_femur;
	// Set up for static object - drugi czlon nogi 1
    sides[0] = 0.03;
    sides[1] = 0.03;
    sides[2] = 0.03;
    dBodySetLinearVel(Object[2].Body, 0, 0, 0);
	dMatrix3 R1, R2;
    dRFromAxisAndAngle(R1, 0, 0, 1, leg_ref[1]);
	dRFromAxisAndAngle(R2, 0, 1, 0, leg_ref[0]);
	dMultiply0 (R, R2, R1, 3, 3, 3);
	dBodySetRotation(Object[2].Body, R);
    dBodySetPosition(Object[2].Body, offset_x+width_min+segment1*cos(leg_ref[0])+(segment2*cos(leg_ref[1])/2)*cos(leg_ref[0]), segment2*sin(leg_ref[1])/2+offset_z, -(lenght)+offset_y-segment1*sin(leg_ref[0])-(segment2*cos(leg_ref[1])/2.0)*sin(leg_ref[0]));
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
    Object[2].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[2].Geom[0], Object[2].Body);
    dBodySetMass(Object[2].Body, &m);

	mass=mass_tibia;
	// Set up for static object - trzeci czlon nogi 1
	sides[0] = 0.02;
    sides[1] = 0.02;
    sides[2] = 0.02;
    dBodySetLinearVel(Object[3].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 0, 0, 1, -leg_ref[2]);
	dBodySetRotation(Object[3].Body, R);
    dBodySetPosition(Object[3].Body, offset_x+width_min+segment1*cos(leg_ref[0])+(segment2*cos(leg_ref[1]))*cos(leg_ref[0])+((segment3-foot_length)*cos(leg_ref[2])/2.0)*cos(leg_ref[0]), segment2*sin(leg_ref[1])-(segment3-foot_length)*sin(leg_ref[2])/2+offset_z, -(lenght)+offset_y-segment1*sin(leg_ref[0])-(segment2*cos(leg_ref[1]))*sin(leg_ref[0])-((segment3-foot_length)*cos(leg_ref[2])/2.0)*sin(leg_ref[0]));
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
	Object[3].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[3].Geom[0], Object[3].Body);
    dBodySetMass(Object[3].Body, &m);

	mass=mass_coxa;
	// Set up for static object - pierwszy czlon nogi 2
    sides[0] = 0.03;
    sides[1] = 0.03;
    sides[2] = 0.03;
    dBodySetPosition(Object[4].Body, offset_x+width_max+(segment1/2.0), offset_z, offset_y);
    dBodySetLinearVel(Object[4].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 1, 0, 0, 0);
	dBodySetRotation(Object[4].Body, R);
    dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
    Object[4].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[4].Geom[0], Object[4].Body);
    dBodySetMass(Object[4].Body, &m);

	mass=mass_femur;
	// Set up for static object - drugi czlon nogi 2
    sides[0] = 0.03;
    sides[1] = 0.03;
    sides[2] = 0.03;
    dBodySetLinearVel(Object[5].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 0, 0, 1, leg_ref[1]);
	dBodySetRotation(Object[5].Body, R);
    dBodySetPosition(Object[5].Body, offset_x+width_max+(segment1)+segment2*cos(leg_ref[1])/2, segment2*sin(leg_ref[1])/2+offset_z,offset_y);
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
    Object[5].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[5].Geom[0], Object[5].Body);
    dBodySetMass(Object[5].Body, &m);

	mass=mass_tibia;
	// Set up for static object - trzeci czlon nogi 2
    sides[0] = 0.02;
    sides[1] = 0.02;
    sides[2] = 0.02;
    dBodySetLinearVel(Object[6].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 0, 0, 1, -leg_ref[2]);
	dBodySetRotation(Object[6].Body, R);
    dBodySetPosition(Object[6].Body, offset_x+width_max+(segment1)+segment2*cos(leg_ref[1])+(segment3-foot_length)*cos(leg_ref[2])/2, segment2*sin(leg_ref[1])-(segment3-foot_length)*sin(leg_ref[2])/2+offset_z, offset_y);
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
	Object[6].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[6].Geom[0], Object[6].Body);
    dBodySetMass(Object[6].Body, &m);

	mass=mass_coxa;
	// Set up for static object - pierwszy czlon nogi 3
    sides[0] = 0.03;
    sides[1] = 0.03;
    sides[2] = 0.03;
    dBodySetPosition(Object[7].Body, offset_x+width_min+(segment1*cos(-leg_ref[0])/2.0), offset_z, (lenght)+offset_y-segment1*sin(-leg_ref[0])/2.0);
    dBodySetLinearVel(Object[7].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 0, 1, 0, -leg_ref[0]);
	dBodySetRotation(Object[7].Body, R);
    dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
    Object[7].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[7].Geom[0], Object[7].Body);
    dBodySetMass(Object[7].Body, &m);

	mass=mass_femur;
	// Set up for static object - drugi czlon nogi 3
    sides[0] = 0.03;
    sides[1] = 0.03;
    sides[2] = 0.03;
    dBodySetLinearVel(Object[8].Body, 0, 0, 0);
    dRFromAxisAndAngle(R1, 0, 0, 1, leg_ref[1]);
	dRFromAxisAndAngle(R2, 0, 1, 0, -leg_ref[0]);
	dMultiply0(R, R2, R1, 3, 3, 3);
	dBodySetRotation(Object[8].Body, R);
    dBodySetPosition(Object[8].Body, offset_x+width_min+segment1*cos(-leg_ref[0])+((segment2/2.0)*cos(leg_ref[1]))*cos(leg_ref[0]), segment2*sin(leg_ref[1])/2+offset_z, (lenght)+offset_y-segment1*sin(-leg_ref[0])-((segment2/2.0)*cos(-leg_ref[1]))*sin(-leg_ref[0]));
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
    Object[8].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[8].Geom[0], Object[8].Body);
    dBodySetMass(Object[8].Body, &m);

	mass=mass_tibia;
	// Set up for static object - trzeci czlon nogi 3
    sides[0] = 0.02;
    sides[1] = 0.02;
    sides[2] = 0.02;
    dBodySetLinearVel(Object[9].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 0, 0,1, -leg_ref[2]);
	dBodySetRotation(Object[9].Body, R);
    dBodySetPosition(Object[9].Body, offset_x+width_min+segment1*cos(-leg_ref[0])+(segment2*cos(leg_ref[1]))*cos(-leg_ref[0])+((segment3-foot_length)*cos(leg_ref[2])/2.0)*cos(-leg_ref[0]), segment2*sin(leg_ref[1])-(segment3-foot_length)*sin(leg_ref[2])/2+offset_z, (lenght)+offset_y-segment1*sin(-leg_ref[0])-(segment2*cos(leg_ref[1]))*sin(-leg_ref[0])-((segment3-foot_length)*cos(leg_ref[2])/2.0)*sin(-leg_ref[0]));
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
	Object[9].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[9].Geom[0], Object[9].Body);
    dBodySetMass(Object[9].Body, &m);

	mass=mass_coxa;
	// Set up for static object - pierwszy czlon nogi 4
    sides[0] = 0.03;
    sides[1] = 0.03;
    sides[2] = 0.03;
    dBodySetPosition(Object[10].Body, offset_x-(width_min+(segment1*cos(-leg_ref[0])/2.0)), offset_z, -(lenght)+offset_y+(segment1/2.0)*sin(-leg_ref[0]));
    dBodySetLinearVel(Object[10].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 0, 1, 0, -leg_ref[0]);
	dBodySetRotation(Object[10].Body, R);
    dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
    Object[10].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[10].Geom[0], Object[10].Body);
    dBodySetMass(Object[10].Body, &m);

	mass=mass_femur;
	// Set up for static object - drugi czlon nogi 4
    sides[0] = 0.03;
    sides[1] = 0.03;
    sides[2] = 0.03;
    dBodySetLinearVel(Object[11].Body, 0, 0, 0);
	dRFromAxisAndAngle(R1, 0, 0, 1, -leg_ref[1]);
	dRFromAxisAndAngle(R2, 0, 1, 0, -leg_ref[0]);
	dMultiply0(R, R2, R1, 3, 3, 3);
	dBodySetRotation(Object[11].Body, R);
    dBodySetPosition(Object[11].Body, offset_x-(width_min+segment1*cos(-leg_ref[0])+((segment2/2.0)*cos(leg_ref[1]))*cos(-leg_ref[0])), segment2*sin(leg_ref[1])/2+offset_z, -(lenght)+offset_y+segment1*sin(-leg_ref[0])+((segment2/2.0)*cos(leg_ref[1]))*sin(-leg_ref[0]));
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
    Object[11].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[11].Geom[0], Object[11].Body);
    dBodySetMass(Object[11].Body, &m);

	mass=mass_tibia;
	// Set up for static object - trzeci czlon nogi 4
    sides[0] = 0.02;
    sides[1] = 0.02;
    sides[2] = 0.02;
    dBodySetLinearVel(Object[12].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 0, 0, 1, leg_ref[2]);
	dBodySetRotation(Object[12].Body, R);
    dBodySetPosition(Object[12].Body, offset_x-(width_min+segment1*cos(-leg_ref[0])+(segment2*cos(leg_ref[1]))*cos(-leg_ref[0])+((segment3-foot_length)*cos(leg_ref[2])/2.0)*cos(-leg_ref[0])), segment2*sin(leg_ref[1])-(segment3-foot_length)*sin(leg_ref[2])/2+offset_z, -(lenght)+offset_y+segment1*sin(-leg_ref[0])+(segment2*cos(leg_ref[1]))*sin(-leg_ref[0])+((segment3-foot_length)*cos(leg_ref[2])/2.0)*sin(-leg_ref[0]));
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
	Object[12].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[12].Geom[0], Object[12].Body);
    dBodySetMass(Object[12].Body, &m);

	mass=mass_coxa;
	// Set up for static object - pierwszy czlon nogi 5
    sides[0] = 0.03;
    sides[1] = 0.03;
    sides[2] = 0.03;
    dBodySetPosition(Object[13].Body, offset_x-(width_max+(segment1/2.0)), offset_z, offset_y);
    dBodySetLinearVel(Object[13].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 1, 0, 0, 0);
	dBodySetRotation(Object[13].Body, R);
    dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
    Object[13].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[13].Geom[0], Object[13].Body);
    dBodySetMass(Object[13].Body, &m);

	mass=mass_femur;
	// Set up for static object - drugi czlon nogi 5
    sides[0] = 0.03;
    sides[1] = 0.03;
    sides[2] = 0.03;
    dBodySetLinearVel(Object[14].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 0, 0, 1, -leg_ref[1]);
	dBodySetRotation(Object[14].Body, R);
    dBodySetPosition(Object[14].Body, offset_x-(width_max+(segment1))-segment2*cos(leg_ref[1])/2, segment2*sin(leg_ref[1])/2+offset_z, offset_y);
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
    Object[14].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[14].Geom[0], Object[14].Body);
    dBodySetMass(Object[14].Body, &m);

	mass=mass_tibia;
	// Set up for static object - trzeci czlon nogi 5
    sides[0] = 0.02;
    sides[1] = 0.02;
    sides[2] = 0.02;
    dBodySetLinearVel(Object[15].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 0, 0, 1, leg_ref[2]);
	dBodySetRotation(Object[15].Body, R);
    dBodySetPosition(Object[15].Body, offset_x-(width_max+(segment1))-segment2*cos(leg_ref[1])-(segment3-foot_length)*cos(leg_ref[2])/2, segment2*sin(leg_ref[1])-(segment3-foot_length)*sin(leg_ref[2])/2+offset_z, offset_y);
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
	Object[15].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[15].Geom[0], Object[15].Body);
    dBodySetMass(Object[15].Body, &m);

	mass=mass_coxa;
	// Set up for static object - pierwszy czlon nogi 6
    sides[0] = 0.03;
    sides[1] = 0.03;
    sides[2] = 0.03;
    dBodySetPosition(Object[16].Body, offset_x-(width_min+(segment1/2.0)*cos(leg_ref[0])), offset_z, (lenght)+offset_y+(segment1/2.0)*sin(leg_ref[0]));
    dBodySetLinearVel(Object[16].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 0, 1, 0, leg_ref[0]);
	dBodySetRotation(Object[16].Body, R);
    dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
    Object[16].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[16].Geom[0], Object[16].Body);
    dBodySetMass(Object[16].Body, &m);

	mass=mass_femur;
	// Set up for static object - drugi czlon nogi 6
    sides[0] = 0.03;
    sides[1] = 0.03;
    sides[2] = 0.03;
    dBodySetLinearVel(Object[17].Body, 0, 0, 0);
	dRFromAxisAndAngle(R1, 0, 0, 1, -leg_ref[1]);
	dRFromAxisAndAngle(R2, 0, 1, 0, leg_ref[0]);
	dMultiply0(R, R2, R1, 3, 3, 3);
	dBodySetRotation(Object[17].Body, R);
    dBodySetPosition(Object[17].Body, offset_x-(width_min+segment1*cos(leg_ref[0])+((segment2/2.0)*cos(leg_ref[1]))*cos(leg_ref[0])), segment2*sin(leg_ref[1])/2+offset_z, (lenght)+offset_y+segment1*sin(leg_ref[0])+((segment2/2.0)*cos(leg_ref[1]))*sin(leg_ref[0]));
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
    Object[17].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[17].Geom[0], Object[17].Body);
    dBodySetMass(Object[17].Body, &m);

	mass=mass_tibia;
	// Set up for static object - trzeci czlon nogi 6
    sides[0] = 0.02;
    sides[1] = 0.02;
    sides[2] = 0.02;
    dBodySetLinearVel(Object[18].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 0, 0, 1, leg_ref[2]);
	dBodySetRotation(Object[18].Body, R);
    dBodySetPosition(Object[18].Body, offset_x-(width_min+segment1*cos(leg_ref[0])+(segment2*cos(leg_ref[1]))*cos(leg_ref[0])+((segment3/2.0)*cos(leg_ref[2]))*cos(leg_ref[0])), segment2*sin(leg_ref[1])-(segment3-foot_length)*sin(leg_ref[2])/2+offset_z, (lenght)+offset_y+segment1*sin(leg_ref[0])+(segment2*cos(leg_ref[1]))*sin(leg_ref[0])+((segment3/2.0)*cos(leg_ref[2]))*sin(leg_ref[0]));
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
	Object[18].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[18].Geom[0], Object[18].Body);
    dBodySetMass(Object[18].Body, &m);

	mass=mass_foot;
	// Set up for static object - stopa nogi 1
    sides[0] = 0.015;
    sides[1] = 0.015;
    sides[2] = 0.005;
    dBodySetLinearVel(Object[19].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 1, 0, 0, -leg_ref[2]);
	dBodySetRotation(Object[19].Body, R);
    dBodySetPosition(Object[19].Body, offset_x+width_min+segment1*cos(leg_ref[0])+(segment2*cos(leg_ref[1]))*cos(leg_ref[0])+(((segment3)-foot_length)*cos(leg_ref[2])/2.0)*cos(leg_ref[0]), segment2*sin(leg_ref[1])-((segment3)-(sides[2]/2))*sin(leg_ref[2])+offset_z, -(lenght)+offset_y-segment1*sin(leg_ref[0])-(segment2*cos(leg_ref[1]))*sin(leg_ref[0])-((segment3)*cos(leg_ref[2])/2.0)*sin(leg_ref[0]));
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
	Object[19].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[19].Geom[0], Object[19].Body);
    dBodySetMass(Object[19].Body, &m);

	//stopa nogi 2
    dBodySetLinearVel(Object[20].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 1, 0, 0, -leg_ref[2]);
	dBodySetRotation(Object[20].Body, R);
    dBodySetPosition(Object[20].Body, offset_x+width_max+(segment1)+segment2*cos(leg_ref[1])+segment3*cos(leg_ref[2])/2, segment2*sin(leg_ref[1])-(segment3-(sides[2]/2))*sin(leg_ref[2])+offset_z, offset_y);
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
	Object[20].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[20].Geom[0], Object[20].Body);
    dBodySetMass(Object[20].Body, &m);

	//stopa nogi 3
	dBodySetLinearVel(Object[21].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 1, 0, 0, -leg_ref[2]);
	dBodySetRotation(Object[21].Body, R);
    dBodySetPosition(Object[21].Body, offset_x+width_min+segment1*cos(-leg_ref[0])+(segment2*cos(leg_ref[1]))*cos(-leg_ref[0])+(segment3*cos(leg_ref[2])/2.0)*cos(-leg_ref[0]), segment2*sin(leg_ref[1])-(segment3-(sides[2]/2))*sin(leg_ref[2])+offset_z, (lenght)+offset_y-segment1*sin(-leg_ref[0])-(segment2*cos(leg_ref[1]))*sin(-leg_ref[0])-(segment3*cos(leg_ref[2])/2.0)*sin(-leg_ref[0]));
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
	Object[21].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[21].Geom[0], Object[21].Body);
    dBodySetMass(Object[21].Body, &m);

	//stopa nogi 4
    dBodySetLinearVel(Object[22].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 1, 0, 0, -leg_ref[2]);
	dBodySetRotation(Object[22].Body, R);
    dBodySetPosition(Object[22].Body, offset_x-(width_min+segment1*cos(-leg_ref[0])+(segment2*cos(leg_ref[1]))*cos(-leg_ref[0])+(segment3*cos(leg_ref[2]))*cos(-leg_ref[0])), segment2*sin(leg_ref[1])-(segment3-(sides[2]/2))*sin(leg_ref[2])+offset_z, -(lenght)+offset_y+segment1*sin(-leg_ref[0])+(segment2*cos(leg_ref[1]))*sin(-leg_ref[0])+(segment3*cos(leg_ref[2]))*sin(-leg_ref[0]));
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
	Object[22].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[22].Geom[0], Object[22].Body);
    dBodySetMass(Object[22].Body, &m);

	// stopa nogi 5
    dBodySetLinearVel(Object[23].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 1, 0, 0, -leg_ref[2]);
	dBodySetRotation(Object[23].Body, R);
    dBodySetPosition(Object[23].Body, offset_x-(width_max+(segment1))-segment2*cos(leg_ref[1])-segment3*cos(leg_ref[2])/2, segment2*sin(leg_ref[1])-(segment3-(sides[2]/2))*sin(leg_ref[2])+offset_z, offset_y);
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
	Object[23].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[23].Geom[0], Object[23].Body);
    dBodySetMass(Object[23].Body, &m);

	// stopa nogi 6
    dBodySetLinearVel(Object[24].Body, 0, 0, 0);
    dRFromAxisAndAngle(R, 1, 0, 0, -leg_ref[2]);
	dBodySetRotation(Object[24].Body, R);
    dBodySetPosition(Object[24].Body, offset_x-(width_min+segment1*cos(leg_ref[0])+(segment2*cos(leg_ref[1]))*cos(leg_ref[0])+((segment3)*cos(leg_ref[2]))*cos(leg_ref[0])), segment2*sin(leg_ref[1])-(segment3-(sides[2]/2))*sin(leg_ref[2])+offset_z, (lenght)+offset_y+segment1*sin(leg_ref[0])+(segment2*cos(leg_ref[1]))*sin(leg_ref[0])+((segment3)*cos(leg_ref[2]))*sin(leg_ref[0]));
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
	Object[24].Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[24].Geom[0], Object[24].Body);
    dBodySetMass(Object[24].Body, &m);

	// zlacze pomiedzy glownym elementem a pierwszym czlonem nogi 1
    Joints[0] = dJointCreateHinge(World, jointgroup);
    dJointAttach(Joints[0], Object[0].Body, Object[1].Body);
    dJointSetHingeAnchor(Joints[0], offset_x+width_min, offset_z, -lenght+offset_y);
    dJointSetHingeAxis(Joints[0], 0, 1, 0);
    dJointSetHingeParam(Joints[0], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[0], dParamHiStop, PI/2);

	// zlacze pomiedzy 1, a 2 elementem nogi 1 
    Joints[1] = dJointCreateHinge(World, jointgroup);
    dJointAttach(Joints[1], Object[1].Body, Object[2].Body);
    dJointSetHingeAnchor(Joints[1], offset_x+width_min+segment1*cos(leg_ref[0]), offset_z, -lenght+offset_y-segment1*sin(leg_ref[0]));
    dJointSetHingeAxis(Joints[1], 0.5, 0, 0.5);
    dJointSetHingeParam(Joints[1], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[1], dParamHiStop, PI/2);

	// zlacze pomiedzy 2, a 3 elementem nogi 1 
    Joints[2] = dJointCreateHinge(World, jointgroup);
    dJointAttach(Joints[2], Object[2].Body, Object[3].Body);
    dJointSetHingeAnchor(Joints[2], offset_x+width_min+segment1*cos(leg_ref[0])+(segment2*cos(leg_ref[1]))*cos(leg_ref[0]), segment2*sin(leg_ref[1])+offset_z, -(lenght)+offset_y-segment1*sin(leg_ref[0])-(segment2*cos(leg_ref[1]))*sin(leg_ref[0]));
    dJointSetHingeAxis(Joints[2], 0.5, 0, 0.5);
    dJointSetHingeParam(Joints[2], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[2], dParamHiStop, PI/2);

	// zlacze pomiedzy prawym wypustkiem a pierwszym czlonem nogi 2
	Joints[3] = dJointCreateHinge(World, jointgroup);
    dJointAttach(Joints[3], Object[0].Body, Object[4].Body);
	dJointSetHingeAnchor(Joints[3], offset_x+width_max, offset_z, 0+offset_y);
    dJointSetHingeAxis(Joints[3], 0, 1, 0);
    dJointSetHingeParam(Joints[3], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[3], dParamHiStop, PI/2);

	// zlacze pomiedzy 1, a 2 elementem nogi 2
	Joints[4] = dJointCreateHinge(World, jointgroup);
    dJointAttach(Joints[4], Object[4].Body, Object[5].Body);
    dJointSetHingeAnchor(Joints[4], offset_x+width_max+segment1, offset_z, offset_y);
    dJointSetHingeAxis(Joints[4], 0, 0, 1);
    dJointSetHingeParam(Joints[4], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[4], dParamHiStop, PI/2);

	// zlacze pomiedzy 2, a 3 elementem nogi 2
    Joints[5] = dJointCreateHinge(World, jointgroup);
    dJointAttach(Joints[5], Object[5].Body, Object[6].Body);
    dJointSetHingeAnchor(Joints[5], offset_x+width_max+segment1+segment2*cos(leg_ref[1]), segment2*sin(leg_ref[1])+offset_z, offset_y);
    dJointSetHingeAxis(Joints[5], 0, 0, 1);
    dJointSetHingeParam(Joints[5], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[5], dParamHiStop, PI/2);

	// zlacze pomiedzy glownym elementem a pierwszym czlonem nogi 3
    Joints[6] = dJointCreateHinge(World, jointgroup);
    dJointAttach(Joints[6], Object[0].Body, Object[7].Body);
    dJointSetHingeAnchor(Joints[6], offset_x+width_min, offset_z, lenght+offset_y);
    dJointSetHingeAxis(Joints[6], 0, 1, 0);
    dJointSetHingeParam(Joints[6], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[6], dParamHiStop, PI/2);

	// zlacze pomiedzy 1, a 2 elementem nogi 3 
    Joints[7] = dJointCreateHinge(World, jointgroup);
    dJointAttach(Joints[7], Object[7].Body, Object[8].Body);
    dJointSetHingeAnchor(Joints[7], offset_x+width_min+segment1*cos(-leg_ref[0]), offset_z, lenght+offset_y-segment1*sin(-leg_ref[0]));
    dJointSetHingeAxis(Joints[7], -0.5, 0, 0.5);
    dJointSetHingeParam(Joints[7], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[7], dParamHiStop, PI/2);

	// zlacze pomiedzy 2, a 3 elementem nogi 3 
    Joints[8] = dJointCreateHinge(World, jointgroup);
    dJointAttach(Joints[8], Object[8].Body, Object[9].Body);
    dJointSetHingeAnchor(Joints[8], offset_x+width_min+segment1*cos(-leg_ref[0])+(segment2*cos(leg_ref[1]))*cos(-leg_ref[0]), segment2*sin(leg_ref[1])+offset_z, lenght+offset_y-segment1*sin(-leg_ref[0])-(segment2*cos(leg_ref[1]))*sin(-leg_ref[0]));
    dJointSetHingeAxis(Joints[8], -0.5, 0, 0.5);
    dJointSetHingeParam(Joints[8], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[8], dParamHiStop, PI/2);

	// zlacze pomiedzy glownym elementem a pierwszym czlonem nogi 4
    Joints[15] = dJointCreateHinge(World, jointgroup);
    dJointAttach(Joints[15], Object[0].Body, Object[10].Body);
    dJointSetHingeAnchor(Joints[15], offset_x-width_min, offset_z, -lenght+offset_y);
    dJointSetHingeAxis(Joints[15], 0, 1, 0);
    dJointSetHingeParam(Joints[15], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[15], dParamHiStop, PI/2);

	// zlacze pomiedzy 1, a 2 elementem nogi 4
	Joints[16] = dJointCreateHinge(World, jointgroup);
    dJointAttach(Joints[16], Object[10].Body, Object[11].Body);
    dJointSetHingeAnchor(Joints[16], offset_x-(width_min+segment1*cos(-leg_ref[0])), offset_z, -lenght+offset_y+segment1*sin(-leg_ref[0]));
    dJointSetHingeAxis(Joints[16], -0.5, 0, 0.5);
    dJointSetHingeParam(Joints[16], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[16], dParamHiStop, PI/2);

	// zlacze pomiedzy 2, a 3 elementem nogi 4 
    Joints[17] = dJointCreateHinge(World, jointgroup);
    dJointAttach(Joints[17], Object[11].Body, Object[12].Body);
    dJointSetHingeAnchor(Joints[17], offset_x-(width_min+segment1*cos(-leg_ref[0])+(segment2*cos(leg_ref[1]))*cos(-leg_ref[0])), segment2*sin(leg_ref[1])+offset_z, -(lenght)+offset_y+segment1*sin(-leg_ref[0])+(segment2*cos(leg_ref[1]))*sin(-leg_ref[0]));
    dJointSetHingeAxis(Joints[17], -0.5, 0, 0.5);
    dJointSetHingeParam(Joints[17], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[17], dParamHiStop, PI/2);

	// zlacze pomiedzy prawym wypustkiem a pierwszym czlonem nogi 5
	Joints[12] = dJointCreateHinge(World, jointgroup);
    dJointAttach(Joints[12], Object[0].Body, Object[13].Body);
	dJointSetHingeAnchor(Joints[12], offset_x-width_max, offset_z, offset_y);
    dJointSetHingeAxis(Joints[12], 0, 1, 0);
    dJointSetHingeParam(Joints[12], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[12], dParamHiStop, PI/2);

	// zlacze pomiedzy 1, a 2 elementem nogi 5
	Joints[13] = dJointCreateHinge(World, jointgroup);
    dJointAttach(Joints[13], Object[13].Body, Object[14].Body);
    dJointSetHingeAnchor(Joints[13], offset_x-(width_max+segment1), offset_z, offset_y);
    dJointSetHingeAxis(Joints[13], 0, 0, 1);
    dJointSetHingeParam(Joints[13], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[13], dParamHiStop, PI/2);

	// zlacze pomiedzy 2, a 3 elementem nogi 5
    Joints[14] = dJointCreateHinge(World, jointgroup);
    dJointAttach(Joints[14], Object[14].Body, Object[15].Body);
    dJointSetHingeAnchor(Joints[14], offset_x-(width_max+segment1)-segment2*cos(leg_ref[1]), segment2*sin(leg_ref[1])+offset_z, offset_y);
    dJointSetHingeAxis(Joints[14], 0, 0, 1);
    dJointSetHingeParam(Joints[14], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[14], dParamHiStop, PI/2);

	// zlacze pomiedzy glownym elementem a pierwszym czlonem nogi 6
    Joints[9] = dJointCreateHinge(World, jointgroup);
    dJointAttach(Joints[9], Object[0].Body, Object[16].Body);
    dJointSetHingeAnchor(Joints[9], offset_x-width_min, offset_z, lenght+offset_y);
    dJointSetHingeAxis(Joints[9], 0, 1, 0);
    dJointSetHingeParam(Joints[9], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[9], dParamHiStop, PI/2);

	// zlacze pomiedzy 1, a 2 elementem nogi 6
	Joints[10] = dJointCreateHinge(World, jointgroup);
    dJointAttach(Joints[10], Object[16].Body, Object[17].Body);
    dJointSetHingeAnchor(Joints[10], offset_x-(width_min+segment1*cos(leg_ref[0])), offset_z, lenght+offset_y+segment1*sin(leg_ref[0]));
    dJointSetHingeAxis(Joints[10], 0.5, 0, 0.5);
    dJointSetHingeParam(Joints[10], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[10], dParamHiStop, PI/2);

	// zlacze pomiedzy 2, a 3 elementem nogi 6 
    Joints[11] = dJointCreateHinge(World, jointgroup);
    dJointAttach(Joints[11], Object[17].Body, Object[18].Body);
    dJointSetHingeAnchor(Joints[11], offset_x-(width_min+segment1*cos(leg_ref[0])+(segment2*cos(leg_ref[1]))*cos(leg_ref[0])), segment2*sin(leg_ref[1])+offset_z, lenght+offset_y+segment1*sin(leg_ref[0])+(segment2*cos(leg_ref[1]))*sin(leg_ref[0]));
    dJointSetHingeAxis(Joints[11], 0.5, 0, 0.5);
    dJointSetHingeParam(Joints[11], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[11], dParamHiStop, PI/2);

	// zlacze pomiedzy noga 1, a stopa
    Joints[18] = dJointCreateUniversal(World, jointgroup);
    dJointAttach(Joints[18], Object[3].Body, Object[19].Body);
    dJointSetUniversalAnchor(Joints[18], offset_x+width_min+segment1*cos(leg_ref[0])+(segment2*cos(leg_ref[1]))*cos(leg_ref[0])+(segment3*cos(leg_ref[2])/2.0)*cos(leg_ref[0]), segment2*sin(leg_ref[1])-(segment3-foot_length)*sin(leg_ref[2])+offset_z, -(lenght)+offset_y-segment1*sin(leg_ref[0])-(segment2*cos(leg_ref[1]))*sin(leg_ref[0])-(segment3*cos(leg_ref[2])/2.0)*sin(leg_ref[0]));
	dJointSetUniversalAxis1 (Joints[18], 1,0,0);
	dJointSetUniversalAxis2 (Joints[18], 0,0,1);

	// zlacze pomiedzy noga 2, a stopa
    Joints[19] = dJointCreateUniversal(World, jointgroup);
    dJointAttach(Joints[19], Object[6].Body, Object[20].Body);
    dJointSetUniversalAnchor(Joints[19], offset_x+width_max+(segment1)+segment2*cos(leg_ref[1])+segment3*cos(leg_ref[2])/2, segment2*sin(leg_ref[1])-(segment3-foot_length)*sin(leg_ref[2])+offset_z, offset_y);
	dJointSetUniversalAxis1(Joints[19], 1,0,0);
	dJointSetUniversalAxis2(Joints[19], 0,0,1);

	// zlacze pomiedzy noga 3, a stopa
    Joints[20] = dJointCreateUniversal(World, jointgroup);
    dJointAttach(Joints[20], Object[9].Body, Object[21].Body);
    dJointSetUniversalAnchor(Joints[20], offset_x+width_min+segment1*cos(-leg_ref[0])+(segment2*cos(leg_ref[1]))*cos(-leg_ref[0])+(segment3*cos(leg_ref[2])/2.0)*cos(-leg_ref[0]), segment2*sin(leg_ref[1])-(segment3-foot_length)*sin(leg_ref[2])+offset_z, (lenght)+offset_y-segment1*sin(-leg_ref[0])-(segment2*cos(leg_ref[1]))*sin(-leg_ref[0])-(segment3*cos(leg_ref[2])/2.0)*sin(-leg_ref[0]));
	dJointSetUniversalAxis1(Joints[20], 1,0,0);
	dJointSetUniversalAxis2(Joints[20], 0,0,1);

	// zlacze pomiedzy noga 4, a stopa
    Joints[21] = dJointCreateUniversal(World, jointgroup);
    dJointAttach(Joints[21], Object[12].Body, Object[22].Body);
    dJointSetUniversalAnchor(Joints[21], offset_x-(width_min+segment1*cos(-leg_ref[0])+(segment2*cos(leg_ref[1]))*cos(-leg_ref[0])+(segment3*cos(leg_ref[2]))*cos(-leg_ref[0])), segment2*sin(leg_ref[1])-(segment3-foot_length)*sin(leg_ref[2])+offset_z, -(lenght)+offset_y+segment1*sin(-leg_ref[0])+(segment2*cos(leg_ref[1]))*sin(-leg_ref[0])+(segment3*cos(leg_ref[2]))*sin(-leg_ref[0]));
	dJointSetUniversalAxis1 (Joints[21], 1,0,0);
	dJointSetUniversalAxis2 (Joints[21], 0,0,1);

	// zlacze pomiedzy noga 5, a stopa
    Joints[22] = dJointCreateUniversal(World, jointgroup);
    dJointAttach(Joints[22], Object[15].Body, Object[23].Body);
    dJointSetUniversalAnchor(Joints[22], offset_x-(width_max+(segment1))-segment2*cos(leg_ref[1])-segment3*cos(leg_ref[2])/2, segment2*sin(leg_ref[1])-(segment3-foot_length)*sin(leg_ref[2])+offset_z, offset_y);
	dJointSetUniversalAxis1(Joints[22], 1,0,0);
	dJointSetUniversalAxis2(Joints[22], 0,0,1);

	// zlacze pomiedzy noga 6, a stopa
    Joints[23] = dJointCreateUniversal(World, jointgroup);
    dJointAttach(Joints[23], Object[18].Body, Object[24].Body);
    dJointSetUniversalAnchor(Joints[23], offset_x-(width_min+segment1*cos(leg_ref[0])+(segment2*cos(leg_ref[1]))*cos(leg_ref[0])+((segment3)*cos(leg_ref[2]))*cos(leg_ref[0])), segment2*sin(leg_ref[1])-(segment3-foot_length)*sin(leg_ref[2])+offset_z, (lenght)+offset_y+segment1*sin(leg_ref[0])+(segment2*cos(leg_ref[1]))*sin(leg_ref[0])+((segment3)*cos(leg_ref[2]))*sin(leg_ref[0]));
	dJointSetUniversalAxis1(Joints[23], 1,0,0);
	dJointSetUniversalAxis2(Joints[23], 0,0,1);

	dReal MaxForce = (dReal)3.0;

	for (int i=0;i<18;i++){
		dJointSetHingeParam(Joints[i], dParamFMax, MaxForce);
	}

	for (int i=0;i<6;i++){
		dJointSetUniversalParam(Joints[i+18], dParamLoStop, -0.00001*PI/180);
		dJointSetUniversalParam(Joints[i+18], dParamHiStop, 0.00001*PI/180);
		dJointSetUniversalParam(Joints[i+18], dParamLoStop2, -0.00001*PI/180);
		dJointSetUniversalParam(Joints[i+18], dParamHiStop2, 0.00001*PI/180);
		dJointSetUniversalParam(Joints[i+18], dParamFudgeFactor, 0.1);
		dJointSetUniversalParam(Joints[i+18], dParamFudgeFactor2, 0.1);
		dJointSetUniversalParam(Joints[i+18], dParamFMax, 30.0);
	}

}

void CODERobot::setPositionSensors() {
	imu.measure();
}

/// ODE pobiera pozycje stopy numeracja stop 1 do 6
void CODERobot::getFootPosition(int foot, dReal position[]){
	dVector3 result;
	double pozycja; // pozycja stopy w ukladzie srdoka ciezkosci ostatniego czlonu nogi
	if (foot<3) pozycja=0.005/2.0;
	else { 
		pozycja=0.005/2.0;
		if (foot==3) foot=5;
		else if (foot==5) foot=3;
	} 
	dBodyGetRelPointPos (Object[19+foot].Body, 0, 0, -pozycja, result);
	position[0]=result[0];
	position[1]=-result[2];
	position[2]=result[1];
}

/*kroczenie po trudnym terenie*/

///pobiera dane o pozycji i orientacji robota
CPunctum CODERobot::getRobotState()
{
	CPunctum robot_pos;
	float robot_orientation[3];
	imu.getIMUorientation(robot_orientation);
	float robot_position[3];
	imu.getIMUorientation(robot_position);
	robot_pos.createTRMatrix(robot_orientation[0],robot_orientation[1],robot_orientation[2],robot_position[0],robot_position[1],robot_position[2]);
	return robot_pos;

}

void CODERobot::getRotAngles(float *angles)
{
	imu.getIMUorientation(angles);
}

/// odczytuje stan przycisku w stopie
bool CODERobot::getContact(int leg){
	return filtered_contact[leg];
}