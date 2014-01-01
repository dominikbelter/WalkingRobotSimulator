#include "StarlETH.h"
#include <math.h>

/// A single instance of Kinect grabber
StarlETH::Ptr robot;

StarlETH::StarlETH(void) : SimRobot("StarlETH Robot") {
	refSpeed.resize(STARLETH_LINKS_NO); //12 servomotors
	refAngles.resize(STARLETH_LINKS_NO); //12 servomotors
	refLoad.resize(STARLETH_LINKS_NO); //12 servomotors
	groundContact.resize(STARLETH_LEGS);
	ODEgroundContact.resize(STARLETH_LEGS);
	imu.setInitialPosition(0.023,0,0);
	for (int i=0;i<STARLETH_LEGS;i++) {
		groundContact[i]=0;
		for (int j=0;j<STARLETH_SERVOS_PER_LEG;j++) {
			refAngles[i*STARLETH_SERVOS_PER_LEG + j]=0;
		}
	}

	for (int i=0; i<STARLETH_LINKS_NO; i++)	{
		refSpeed[i] = 1;
		refLoad[i] = 0;
	}

	refAngles[1]=(robsim::float_type)deg2rad((float)25);
	refAngles[2]=(robsim::float_type)deg2rad((float)-25);
	refAngles[4]=(robsim::float_type)deg2rad((float)-25);
	refAngles[5]=(robsim::float_type)deg2rad((float)25);
	refAngles[7]=(robsim::float_type)deg2rad((float)-25);
	refAngles[8]=(robsim::float_type)deg2rad((float)25);
	refAngles[10]=(robsim::float_type)deg2rad((float)25);
	refAngles[11]=(robsim::float_type)deg2rad((float)-25);
}

StarlETH::~StarlETH(void)
{
}

///ustawia poczatkowa pozycje i orientacje robota
void StarlETH::setInitialPosition(robsim::float_type x, robsim::float_type y, robsim::float_type z, robsim::float_type alpha, robsim::float_type beta, robsim::float_type gamma){
	imu.setInitialPosition(x,y,z,alpha,beta,gamma);
}

/// read current joint positions
void StarlETH::readLegAngles(uint_fast8_t legNo, std::vector<robsim::float_type>& currentAngles) const{
	currentAngles[0] = dJointGetHingeAngle(Joints[legNo*STARLETH_SERVOS_PER_LEG]);
	currentAngles[1] = dJointGetHingeAngle(Joints[legNo*STARLETH_SERVOS_PER_LEG+1]);
	currentAngles[2] = dJointGetHingeAngle(Joints[legNo*STARLETH_SERVOS_PER_LEG+2]);
}

/// read current joints speed
void StarlETH::readSpeed(std::vector<robsim::float_type>& currentSpeed) const {
	for (int i =0;i<STARLETH_JOINTS_NO;i++)
		currentSpeed[i] = dJointGetHingeAngleRate(Joints[i]);
}

/// read current joints speed
void StarlETH::readLegSpeed(uint_fast8_t legNo, std::vector<robsim::float_type>& currentSpeed) const {
	currentSpeed[0] = dJointGetHingeAngleRate(Joints[legNo*STARLETH_SERVOS_PER_LEG]);
	currentSpeed[1] = dJointGetHingeAngleRate(Joints[legNo*STARLETH_SERVOS_PER_LEG+1]);
	currentSpeed[2] = dJointGetHingeAngleRate(Joints[legNo*STARLETH_SERVOS_PER_LEG+2]);
}

/// read current joints speed
void StarlETH::readLoad(std::vector<robsim::float_type>& currentLoad) const {
	std::cout << "not implemented" << endl;
}

/// read current joints speed
void StarlETH::readLegLoad(uint_fast8_t legNo, std::vector<robsim::float_type>& currentLoad) {
	std::cout << "not implemented" << endl;
}

/// read reference value for servomotor (angle)
const robsim::float_type& StarlETH::getRefAngle(uint_fast8_t leg, uint_fast8_t joint) const{
	return refAngles[leg*STARLETH_SERVOS_PER_LEG+joint];
}

/// read reference value for servomotor (speed)
const robsim::float_type& StarlETH::getRefSpeed(uint_fast8_t leg, uint_fast8_t joint) const{
	return refSpeed[leg*STARLETH_SERVOS_PER_LEG+joint];
}

/// read reference value for servomotor (load)
const robsim::float_type& StarlETH::getRefLoad(uint_fast8_t leg, uint_fast8_t joint) const{
	return refLoad[leg*STARLETH_SERVOS_PER_LEG+joint];
}

/// ODE - symulacja regulatora w serwomechanizmie
void StarlETH::setServo(uint_fast8_t servoNo, robsim::float_type value) {
	dReal Gain = (dReal) 55.1465;
	dReal v_max = (dReal) (refSpeed[servoNo]/114.0)*STARLETH_MAX_SERVO_SPEED;
	dReal MaxForce = (dReal)13.0;

	dReal TruePosition = dJointGetHingeAngle(Joints[servoNo]);
	dReal DesiredPosition = (dReal)value;
	dReal Error = TruePosition - DesiredPosition;

	dReal DesiredVelocity = -Error * Gain;
	if (DesiredVelocity>v_max) 
		DesiredVelocity=v_max;
	if (DesiredVelocity<-v_max) 
		DesiredVelocity=-v_max;

	dJointSetHingeParam(Joints[servoNo], dParamVel, DesiredVelocity);

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

/// set all servos using selected controller (P/PI/PID)
void StarlETH::setAllServos() {
	for (int i =0;i<STARLETH_JOINTS_NO;i++)
		setServo(i, refAngles[i]);
}

/// read current joint positions
void StarlETH::readAngles(std::vector<robsim::float_type>& currentAngles) const{
	for (int i=0;i<STARLETH_JOINTS_NO;i++) {
		currentAngles[i]=dJointGetHingeAngle(Joints[i]);
	}
}

/// Create ODE object
void StarlETH::createODEObject(dSpaceID& space, uint_fast8_t objectId, robsim::float_type* rot, CPunctum& pose, robsim::float_type mass, robsim::float_type* sides){
	dMass m;
    dBodySetLinearVel(Object[objectId].Body, 0, 0, 0);
	dMatrix3 R, R1, R2, R3;
    dRFromAxisAndAngle(R1, 1, 0, 0, rot[0]);
	dRFromAxisAndAngle(R2, 0, 1, 0, rot[1]);
	dRFromAxisAndAngle(R3, 0, 0, 1, rot[2]);
	dMultiply0 (R, R2, R1, 3, 3, 3);
	dMultiply0 (R, R, R3, 3, 3, 3);
	dBodySetRotation(Object[objectId].Body, R);
    dBodySetPosition(Object[objectId].Body, pose.getElement(1,4), pose.getElement(3,4), -pose.getElement(2,4));
	dMassSetBoxTotal(&m, mass, sides[0], sides[1], sides[2]);
    Object[objectId].Geom[0] = dCreateBox(space, sides[0], sides[1], sides[2]);
    dGeomSetBody(Object[objectId].Geom[0], Object[objectId].Body);
    dBodySetMass(Object[objectId].Body, &m);
}

/// definicja robota w srodowisku ODE
void StarlETH::ODEcreateRobot(dWorldID& world, dSpaceID& space, dJointGroupID& jointgroup, robsim::float_type dt) {

	#pragma warning( disable : 4244 4305 )

	imu.setDT(dt);

	double mass_corp=0.908;
	double mass_tibia=0.163;
	double mass_femur=0.108;
	double mass_coxa=0.134;
	double mass_foot=0.05;
	
    dReal sides[3];
    dMass m;
    dMatrix3 R; // macierz rotacji
    CVector tempVect(0.0, 0.0, 0.0);
	double mass=mass_corp;
	double leg_ref[3]={0*PI/180,(-25)*PI/180,(50)*PI/180};
	float robot_position[3];
	imu.getIMUposition(robot_position);
	double offset_z=robot_position[2]; //przesuniecie pozycji robota w osi z
	double offset_x=robot_position[0]; //przesuniecie pozycji robota w osi x
	double offset_y=-robot_position[1]; //przesuniecie pozycji robota w osi y

    double promien = 0.005; //promien piguly - 3 segment nogi
    double dlugosc = STARLETH_SEGMENT3-2*promien; //dlugosc piguly - 2*3 segment nogi

	for (int bodies = 0;bodies<STARLETH_LINKS_NO;bodies++)
		Object[bodies].Body = dBodyCreate(world);

    // Set up for static object - robot's body
    sides[0] = 0.03;    sides[1] = 0.03;    sides[2] = 0.03;
	robsim::float_type rot[3] = {0,leg_ref[0],0};
	robsim::float_type side[3] = {sides[0],sides[1],sides[2]};
	CPunctum platform,a1,a2,a3,a4;
	platform.createTRMatrix(0,0,0,robot_position[0],robot_position[1],robot_position[2]);
	createODEObject(space, 0, rot, platform, mass, side);

	imu.setIMUBody(Object[0].Body, Object[0].Geom[0]);//ustawienie imu

	//a1 matrix
	a1.setEye();
	a1.setElement(cos(leg_ref[0]),1,1);	a1.setElement(-sin(leg_ref[0]),1,2);
	a1.setElement(sin(leg_ref[0]),2,1);	a1.setElement(cos(leg_ref[0]),2,2);
	cout << "a1: " << endl;
	a1.showMatrix();
//macierz a2
	a2.setEye();
	a2.setElement(cos(leg_ref[1]),1,1);	a2.setElement(-sin(leg_ref[1]),1,2); a2.setElement(STARLETH_SEGMENT1,1,4);
	a2.setElement(0.0,2,2); a2.setElement(-1.0,2,3); a2.setElement(sin(leg_ref[1]),3,1); a2.setElement(cos(leg_ref[1]),3,2); a2.setElement(0.0,3,3);
	CPunctum a2half(a2);
	a2half.setElement(a2.getElement(1,4)/2.0,1,4);
	cout << "a2: " << endl;
	a2.showMatrix();

//macierz a3
	a3.setEye();
	a3.setElement(cos(leg_ref[2]),1,1);	a3.setElement(-sin(leg_ref[2]),1,2); a3.setElement(STARLETH_SEGMENT2,1,4);
	a3.setElement(sin(leg_ref[2]),2,1);	a3.setElement(cos(leg_ref[2]),2,2);
	CPunctum a3half(a3);
	a3half.setElement(a3.getElement(1,4)/2.0,1,4);
	cout << "a3: " << endl;
	a3.showMatrix();
	//a4 matrix
	a4.setEye();
	a4.setElement(STARLETH_SEGMENT3,1,4);
	CPunctum a4half(a4);
	a4half.setElement(a4.getElement(1,4)/2.0,1,4);
	cout << "a4: " << endl;
	a4.showMatrix();

	CPunctum leg1start;
	leg1start.createTRMatrix(0,PI/2.0,0,STARLETH_LENGTH,-STARLETH_WIDTH,0);
	CPunctum leg2start;
	leg2start.createTRMatrix(0,PI/2.0,0,-STARLETH_LENGTH,-STARLETH_WIDTH,0);
	CPunctum leg3start;
	leg3start.createTRMatrix(0,PI/2.0,0,-STARLETH_LENGTH,STARLETH_WIDTH,0);
	CPunctum leg4start;
	leg4start.createTRMatrix(0,PI/2.0,0,STARLETH_LENGTH,STARLETH_WIDTH,0);
	CPunctum tmp;
	tmp = platform*leg1start*a1*a2half;
	platform.showMatrix();
	cout << "leg 1a: " << endl;
	tmp.showMatrix();
	mass=mass_coxa;
	// Set up for static object - hip
    sides[0] = 0.03;    sides[1] = 0.03;    sides[2] = 0.03;
	robsim::float_type rot1[3] = {0,leg_ref[0],0};
	robsim::float_type side1[3] = {sides[0],sides[1],sides[2]};
	createODEObject(space, 1, rot1, tmp, mass, side1);

	tmp = platform*leg1start*a1*a2*a3half;
	cout << "leg 1b: " << endl;
	tmp.showMatrix();
	mass=mass_femur;
	// Set up for static object - thigh
    sides[0] = 0.03;    sides[1] = 0.03;    sides[2] = 0.03;
	robsim::float_type rot2[3] = {0,leg_ref[0],leg_ref[1]};
	robsim::float_type side2[3] = {sides[0],sides[1],sides[2]};
	createODEObject(space, 2, rot2, tmp, mass, side2);
	dMatrix3 R1, R2;
	
	mass=mass_tibia;
	tmp = platform*leg1start*a1*a2*a3*a4half;
	cout << "leg 1c: " << endl;
	tmp.showMatrix();
	// Set up for static object - shank
	sides[0] = 0.02;    sides[1] = 0.02;    sides[2] = 0.02;
	robsim::float_type rot3[3] = {0,0,leg_ref[1]+leg_ref[2]};
	robsim::float_type side3[3] = {sides[0],sides[1],sides[2]};
	createODEObject(space, 3, rot3, tmp, mass, side3);

	leg_ref[1]= - leg_ref[1]; leg_ref[2] = -leg_ref[2];
	a2.setElement(cos(leg_ref[1]),1,1);	a2.setElement(-sin(leg_ref[1]),1,2); 
	a2.setElement(sin(leg_ref[1]),3,1); a2.setElement(cos(leg_ref[1]),3,2);
	a3.setElement(cos(leg_ref[2]),1,1);	a3.setElement(-sin(leg_ref[2]),1,2);
	a3.setElement(sin(leg_ref[2]),2,1);	a3.setElement(cos(leg_ref[2]),2,2);

	tmp = platform*leg2start*a1*a2half;
	mass=mass_coxa;
	// Set up for static object - shank leg 2
    sides[0] = 0.03;    sides[1] = 0.03;    sides[2] = 0.03;
	robsim::float_type rot4[3] = {0,0,0};
	robsim::float_type side4[3] = {sides[0],sides[1],sides[2]};
	createODEObject(space, 4, rot4, tmp, mass, side4);

	tmp = platform*leg2start*a1*a2*a3half;
	mass=mass_femur;
	// Set up for static object - drugi czlon nogi 2
    sides[0] = 0.03;    sides[1] = 0.03;    sides[2] = 0.03;
	robsim::float_type rot5[3] = {0,0,leg_ref[1]};
	robsim::float_type side5[3] = {sides[0],sides[1],sides[2]};
	createODEObject(space, 5, rot5, tmp, mass, side5);

	tmp = platform*leg2start*a1*a2*a3*a4half;
	mass=mass_tibia;
	// Set up for static object - trzeci czlon nogi 2
    sides[0] = 0.02;    sides[1] = 0.02;    sides[2] = 0.02;
	robsim::float_type rot6[3] = {0,0,leg_ref[1]+leg_ref[2]};
	robsim::float_type side6[3] = {sides[0],sides[1],sides[2]};
	createODEObject(space, 6, rot6, tmp, mass, side6);

	tmp = platform*leg3start*a1*a2half;
	mass=mass_coxa;
	// Set up for static object - pierwszy czlon nogi 3
    sides[0] = 0.03;    sides[1] = 0.03;    sides[2] = 0.03;
	robsim::float_type rot7[3] = {0,leg_ref[0],0};
	robsim::float_type side7[3] = {sides[0],sides[1],sides[2]};
	createODEObject(space, 7, rot7, tmp, mass, side7);

	tmp = platform*leg3start*a1*a2*a3half;
	mass=mass_femur;
	// Set up for static object - drugi czlon nogi 3
    sides[0] = 0.03;    sides[1] = 0.03;    sides[2] = 0.03;
	robsim::float_type rot8[3] = {0,leg_ref[0],leg_ref[1]};
	robsim::float_type side8[3] = {sides[0],sides[1],sides[2]};
	createODEObject(space, 8, rot8, tmp, mass, side8);

	tmp = platform*leg3start*a1*a2*a3*a4half;
	mass=mass_tibia;
	// Set up for static object - trzeci czlon nogi 3
    sides[0] = 0.02;    sides[1] = 0.02;    sides[2] = 0.02;
	robsim::float_type rot9[3] = {0,0,leg_ref[1]+leg_ref[2]};
	robsim::float_type side9[3] = {sides[0],sides[1],sides[2]};
	createODEObject(space, 9, rot9, tmp, mass, side9);

	leg_ref[1]= - leg_ref[1]; leg_ref[2] = -leg_ref[2];
	a2.setElement(cos(leg_ref[1]),1,1);	a2.setElement(-sin(leg_ref[1]),1,2); 
	a2.setElement(sin(leg_ref[1]),3,1); a2.setElement(cos(leg_ref[1]),3,2);
	a3.setElement(cos(leg_ref[2]),1,1);	a3.setElement(-sin(leg_ref[2]),1,2);
	a3.setElement(sin(leg_ref[2]),2,1);	a3.setElement(cos(leg_ref[2]),2,2);
	tmp = platform*leg4start*a1*a2half;
	mass=mass_coxa;
	// Set up for static object - pierwszy czlon nogi 4
    sides[0] = 0.03;    sides[1] = 0.03;    sides[2] = 0.03;
	robsim::float_type rot10[3] = {0,leg_ref[0],0};
	robsim::float_type side10[3] = {sides[0],sides[1],sides[2]};
	createODEObject(space, 10, rot10, tmp, mass, side10);

	tmp = platform*leg4start*a1*a2*a3half;
	mass=mass_femur;
	// Set up for static object - drugi czlon nogi 4
    sides[0] = 0.03;    sides[1] = 0.03;    sides[2] = 0.03;
	robsim::float_type rot11[3] = {0,leg_ref[0],leg_ref[1]};
	robsim::float_type side11[3] = {sides[0],sides[1],sides[2]};
	createODEObject(space, 11, rot11, tmp, mass, side11);

	tmp = platform*leg4start*a1*a2*a3*a4half;
	mass=mass_tibia;
	// Set up for static object - trzeci czlon nogi 4
    sides[0] = 0.02;    sides[1] = 0.02;    sides[2] = 0.02;
	robsim::float_type rot12[3] = {0,0,leg_ref[1]+leg_ref[2]};
	robsim::float_type side12[3] = {sides[0],sides[1],sides[2]};
	createODEObject(space, 12, rot12, tmp, mass, side12);

	tmp = platform*leg1start*a1*a2*a3*a4;
	cout << "foot 1: " << endl;
	tmp.showMatrix();
	mass=mass_foot;
	// Set up for static object - stopa nogi 1
    sides[0] = 0.015;    sides[1] = 0.005;    sides[2] = 0.015;
	robsim::float_type rot13[3] = {0,0,0};
	robsim::float_type side13[3] = {sides[0],sides[1],sides[2]};
	createODEObject(space, 13, rot13, tmp, mass, side13);

	//stopa nogi 2
	tmp = platform*leg2start*a1*a2*a3*a4;
	robsim::float_type rot14[3] = {0,0,0};
	robsim::float_type side14[3] = {sides[0],sides[1],sides[2]};
	createODEObject(space, 14, rot14, tmp, mass, side14);

	//stopa nogi 3
	tmp = platform*leg3start*a1*a2*a3*a4;
	robsim::float_type rot15[3] = {0,0,0};
	robsim::float_type side15[3] = {sides[0],sides[1],sides[2]};
	createODEObject(space, 15, rot15, tmp, mass, side15);

	//stopa nogi 4
	tmp = platform*leg4start*a1*a2*a3*a4;
	robsim::float_type rot16[3] = {0,0,0};
	robsim::float_type side16[3] = {sides[0],sides[1],sides[2]};
	createODEObject(space, 16, rot16, tmp, mass, side16);

	leg_ref[1]= -25; leg_ref[2] = 50;
	a2.setElement(cos(leg_ref[1]),1,1);	a2.setElement(-sin(leg_ref[1]),1,2); 
	a2.setElement(sin(leg_ref[1]),3,1); a2.setElement(cos(leg_ref[1]),3,2);
	a3.setElement(cos(leg_ref[2]),1,1);	a3.setElement(-sin(leg_ref[2]),1,2);
	a3.setElement(sin(leg_ref[2]),2,1);	a3.setElement(cos(leg_ref[2]),2,2);
	// zlacze pomiedzy glownym elementem a pierwszym czlonem nogi 1
	tmp = platform*leg1start;
    Joints[0] = dJointCreateHinge(world, jointgroup);
    dJointAttach(Joints[0], Object[0].Body, Object[1].Body);
    dJointSetHingeAnchor(Joints[0], tmp.getElement(1,4), tmp.getElement(3,4), -tmp.getElement(2,4));
    dJointSetHingeAxis(Joints[0], 1, 0, 0);
    dJointSetHingeParam(Joints[0], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[0], dParamHiStop, PI/2);

	// zlacze pomiedzy 1, a 2 elementem nogi 1
	tmp = platform*leg1start*a1*a2;
    Joints[1] = dJointCreateHinge(world, jointgroup);
    dJointAttach(Joints[1], Object[1].Body, Object[2].Body);
    dJointSetHingeAnchor(Joints[1], tmp.getElement(1,4), tmp.getElement(3,4), -tmp.getElement(2,4));
    dJointSetHingeAxis(Joints[1], 0, 0, 1);
    dJointSetHingeParam(Joints[1], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[1], dParamHiStop, PI/2);

	// zlacze pomiedzy 2, a 3 elementem nogi 1
	tmp = platform*leg1start*a1*a2*a3;
    Joints[2] = dJointCreateHinge(world, jointgroup);
    dJointAttach(Joints[2], Object[2].Body, Object[3].Body);
    dJointSetHingeAnchor(Joints[2], tmp.getElement(1,4), tmp.getElement(3,4), -tmp.getElement(2,4));
    dJointSetHingeAxis(Joints[2], 0, 0, 1);
    dJointSetHingeParam(Joints[2], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[2], dParamHiStop, PI/2);

	leg_ref[1]= - leg_ref[1]; leg_ref[2] = -leg_ref[2];
	a2.setElement(cos(leg_ref[1]),1,1);	a2.setElement(-sin(leg_ref[1]),1,2); 
	a2.setElement(sin(leg_ref[1]),3,1); a2.setElement(cos(leg_ref[1]),3,2);
	a3.setElement(cos(leg_ref[2]),1,1);	a3.setElement(-sin(leg_ref[2]),1,2);
	a3.setElement(sin(leg_ref[2]),2,1);	a3.setElement(cos(leg_ref[2]),2,2);
	// zlacze pomiedzy prawym wypustkiem a pierwszym czlonem nogi 2
	tmp = platform*leg2start;
	Joints[3] = dJointCreateHinge(world, jointgroup);
    dJointAttach(Joints[3], Object[0].Body, Object[4].Body);
	dJointSetHingeAnchor(Joints[3], tmp.getElement(1,4), tmp.getElement(3,4), -tmp.getElement(2,4));
    dJointSetHingeAxis(Joints[3], 1, 0, 0);
    dJointSetHingeParam(Joints[3], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[3], dParamHiStop, PI/2);

	// zlacze pomiedzy 1, a 2 elementem nogi 2
	tmp = platform*leg2start*a1*a2;
	Joints[4] = dJointCreateHinge(world, jointgroup);
    dJointAttach(Joints[4], Object[4].Body, Object[5].Body);
    dJointSetHingeAnchor(Joints[4], tmp.getElement(1,4), tmp.getElement(3,4), -tmp.getElement(2,4));
    dJointSetHingeAxis(Joints[4], 0, 0, 1);
    dJointSetHingeParam(Joints[4], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[4], dParamHiStop, PI/2);

	// zlacze pomiedzy 2, a 3 elementem nogi 2
	tmp = platform*leg2start*a1*a2*a3;
    Joints[5] = dJointCreateHinge(world, jointgroup);
    dJointAttach(Joints[5], Object[5].Body, Object[6].Body);
    dJointSetHingeAnchor(Joints[5], tmp.getElement(1,4), tmp.getElement(3,4), -tmp.getElement(2,4));
    dJointSetHingeAxis(Joints[5], 0, 0, 1);
    dJointSetHingeParam(Joints[5], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[5], dParamHiStop, PI/2);

	// zlacze pomiedzy glownym elementem a pierwszym czlonem nogi 3
	tmp = platform*leg3start;
    Joints[6] = dJointCreateHinge(world, jointgroup);
    dJointAttach(Joints[6], Object[0].Body, Object[7].Body);
    dJointSetHingeAnchor(Joints[6], tmp.getElement(1,4), tmp.getElement(3,4), -tmp.getElement(2,4));
    dJointSetHingeAxis(Joints[6], 1, 0, 0);
    dJointSetHingeParam(Joints[6], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[6], dParamHiStop, PI/2);

	// zlacze pomiedzy 1, a 2 elementem nogi 3 
	tmp = platform*leg3start*a1*a2;
    Joints[7] = dJointCreateHinge(world, jointgroup);
    dJointAttach(Joints[7], Object[7].Body, Object[8].Body);
    dJointSetHingeAnchor(Joints[7], tmp.getElement(1,4), tmp.getElement(3,4), -tmp.getElement(2,4));
    dJointSetHingeAxis(Joints[7], 0, 0, 1);
    dJointSetHingeParam(Joints[7], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[7], dParamHiStop, PI/2);

	// zlacze pomiedzy 2, a 3 elementem nogi 3 
	tmp = platform*leg3start*a1*a3;
    Joints[8] = dJointCreateHinge(world, jointgroup);
    dJointAttach(Joints[8], Object[8].Body, Object[9].Body);
    dJointSetHingeAnchor(Joints[8], tmp.getElement(1,4), tmp.getElement(3,4), -tmp.getElement(2,4));
    dJointSetHingeAxis(Joints[8], 0, 0, 1);
    dJointSetHingeParam(Joints[8], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[8], dParamHiStop, PI/2);

	leg_ref[1]= - leg_ref[1]; leg_ref[2] = -leg_ref[2];
	a2.setElement(cos(leg_ref[1]),1,1);	a2.setElement(-sin(leg_ref[1]),1,2); 
	a2.setElement(sin(leg_ref[1]),3,1); a2.setElement(cos(leg_ref[1]),3,2);
	a3.setElement(cos(leg_ref[2]),1,1);	a3.setElement(-sin(leg_ref[2]),1,2);
	a3.setElement(sin(leg_ref[2]),2,1);	a3.setElement(cos(leg_ref[2]),2,2);
	// zlacze pomiedzy glownym elementem a pierwszym czlonem nogi 4
	tmp = platform*leg4start;
    Joints[9] = dJointCreateHinge(world, jointgroup);
    dJointAttach(Joints[9], Object[0].Body, Object[10].Body);
    dJointSetHingeAnchor(Joints[9], tmp.getElement(1,4), tmp.getElement(3,4), -tmp.getElement(2,4));
    dJointSetHingeAxis(Joints[9], 1, 0, 0);
    dJointSetHingeParam(Joints[9], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[9], dParamHiStop, PI/2);

	// zlacze pomiedzy 1, a 2 elementem nogi 4
	tmp = platform*leg4start*a1*a2;
	Joints[10] = dJointCreateHinge(world, jointgroup);
    dJointAttach(Joints[10], Object[10].Body, Object[11].Body);
    dJointSetHingeAnchor(Joints[10], tmp.getElement(1,4), tmp.getElement(3,4), -tmp.getElement(2,4));
    dJointSetHingeAxis(Joints[10], 0, 0, 1);
    dJointSetHingeParam(Joints[10], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[10], dParamHiStop, PI/2);

	// zlacze pomiedzy 2, a 3 elementem nogi 4 
	tmp = platform*leg4start*a1*a2*a3;
    Joints[11] = dJointCreateHinge(world, jointgroup);
    dJointAttach(Joints[11], Object[11].Body, Object[12].Body);
    dJointSetHingeAnchor(Joints[11], tmp.getElement(1,4), tmp.getElement(3,4), -tmp.getElement(2,4));
    dJointSetHingeAxis(Joints[11], 0, 0, 1);
    dJointSetHingeParam(Joints[11], dParamLoStop, -PI/2);
    dJointSetHingeParam(Joints[11], dParamHiStop, PI/2);

	// zlacze pomiedzy noga 1, a stopa
    Joints[12] = dJointCreateUniversal(world, jointgroup);
    dJointAttach(Joints[12], Object[3].Body, Object[13].Body);
    dJointSetUniversalAnchor(Joints[12], 0,0,0);
	dJointSetUniversalAxis1 (Joints[12], 1,0,0);
	dJointSetUniversalAxis2 (Joints[12], 0,0,1);

	// zlacze pomiedzy noga 2, a stopa
    Joints[13] = dJointCreateUniversal(world, jointgroup);
    dJointAttach(Joints[13], Object[6].Body, Object[14].Body);
    dJointSetUniversalAnchor(Joints[13], 0,0,0);
	dJointSetUniversalAxis1(Joints[13], 1,0,0);
	dJointSetUniversalAxis2(Joints[13], 0,0,1);

	// zlacze pomiedzy noga 3, a stopa
    Joints[14] = dJointCreateUniversal(world, jointgroup);
    dJointAttach(Joints[14], Object[9].Body, Object[15].Body);
    dJointSetUniversalAnchor(Joints[14], 0,0,0);
	dJointSetUniversalAxis1(Joints[14], 1,0,0);
	dJointSetUniversalAxis2(Joints[14], 0,0,1);

	// zlacze pomiedzy noga 4, a stopa
    Joints[15] = dJointCreateUniversal(world, jointgroup);
    dJointAttach(Joints[15], Object[12].Body, Object[16].Body);
    dJointSetUniversalAnchor(Joints[15], 0,0,0);
	dJointSetUniversalAxis1 (Joints[15], 1,0,0);
	dJointSetUniversalAxis2 (Joints[15], 0,0,1);

	dReal MaxForce = (dReal)3.0;

	for (int i=0;i<STARLETH_JOINTS_NO;i++){
		dJointSetHingeParam(Joints[i], dParamFMax, MaxForce);
	}

	for (int i=0;i<STARLETH_LEGS;i++){
		dJointSetUniversalParam(Joints[STARLETH_JOINTS_NO+i], dParamLoStop, -0.00001*PI/180);
		dJointSetUniversalParam(Joints[STARLETH_JOINTS_NO+i], dParamHiStop, 0.00001*PI/180);
		dJointSetUniversalParam(Joints[STARLETH_JOINTS_NO+i], dParamLoStop2, -0.00001*PI/180);
		dJointSetUniversalParam(Joints[STARLETH_JOINTS_NO+i], dParamHiStop2, 0.00001*PI/180);
		dJointSetUniversalParam(Joints[STARLETH_JOINTS_NO+i], dParamFudgeFactor, 0.1);
		dJointSetUniversalParam(Joints[STARLETH_JOINTS_NO+i], dParamFudgeFactor2, 0.1);
		dJointSetUniversalParam(Joints[STARLETH_JOINTS_NO+i], dParamFMax, 30.0);
	}

}

void StarlETH::setPositionSensors() {
	imu.measure();
}

/// ODE pobiera pozycje stopy numeracja stop 1 do 6
void StarlETH::getFootPosition(uint_fast8_t foot, CPunctum& pose) const{
	dVector3 result;
	double foot_z = 0.005/2.0; // foot position in the last joint reference frame
	dBodyGetRelPointPos (Object[STARLETH_LINKS_NO-STARLETH_LEGS+foot].Body, 0, 0, -foot_z, result);
	pose.createTRMatrix(0,0,0, result[0], -result[2], result[1]);
}

/// get robot pose
const CPunctum StarlETH::getRobotState(void)
{
	imu.measure();
	CPunctum robot_pos;
	float robot_orientation[3];
	imu.getIMUorientation(robot_orientation);
	float robot_position[3];
	imu.getIMUposition(robot_position);
	robot_pos.createTRMatrix(robot_orientation[0],robot_orientation[1],robot_orientation[2],robot_position[0],robot_position[1],robot_position[2]);
	return robot_pos;

}

void StarlETH::getRPY(robsim::float_type (&rpy)[3]){
	float angles[3];
	imu.measure();
	imu.getIMUorientation(angles);
	rpy[0] = angles[0]; rpy[1] = angles[1]; rpy[2] = angles[2];
}

void StarlETH::getPosition(robsim::float_type (&position)[3]){
	imu.measure();
	float robot_position[3];
	imu.getIMUposition(robot_position);
	position[0] = robot_position[0]; position[1] = robot_position[1]; position[2] = robot_position[2];
}

/// Get ODE geom id
const dGeomID StarlETH::getGeomId(uint_fast8_t partNo) const{
	return Object[partNo].Geom[0];
}

/// Check if considered parts should collide
bool StarlETH::collide(dBodyID& b1, dBodyID& b2) const {
	//if ((b1!=Object[3].Body)&&(b1!=Object[6].Body)&&(b1!=Object[9].Body)&&(b1!=Object[12].Body)&&(b1!=Object[15].Body)&&(b1!=Object[18].Body)&&(b2!=Object[3].Body)&&(b2!=Object[6].Body)&&(b2!=Object[9].Body)&&(b2!=Object[12].Body)&&(b2!=Object[15].Body)&&(b2!=Object[16].Body))
		return true;//always detect collisions
	//else
	//	return false;
}

/// Set info about ODE collisions
void StarlETH::setODEContacts(dBodyID& b1, dBodyID& b2, dBodyID& groundId) {
	for (int it=0;it<STARLETH_LEGS;it++)	{
		if (b1==Object[STARLETH_LINKS_NO - STARLETH_LEGS + it].Body&&b2==groundId||b1==groundId&&b2==Object[STARLETH_LINKS_NO - STARLETH_LEGS + it].Body) {
			setODEContact(it,true);
		}
	}
}

robsim::SimRobot* robsim::createSimStarlETH(void) {
	robot.reset(new StarlETH());
	return robot.get();
}