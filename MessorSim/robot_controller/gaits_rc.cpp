#include "gaits_rc.h"
#include <math.h>

CGaits_RC::CGaits_RC(COdeWorld* dynamicWorld, RobotStructure* robot_structure)
{
	robot_rc = new CRobot_RC(dynamicWorld,robot_structure);
	setInitZeroAngle();
	smart_gait_iterator = 0;
	fposlizg = fopen("poslizg.txt","w");
}

CGaits_RC::~CGaits_RC(void)
{

}

/// ustawia poczatkowa konfiguracje neutralna dla robota (0, 24, -114)
void CGaits_RC::setInitZeroAngle()
{
	for (int i=0;i<6;i++){
	    if ((i==0)||(i==5)) robot_rc->leg[i].zero_angle[0]=(float)0.785;
		if ((i==1)||(i==4)) robot_rc->leg[i].zero_angle[0]=(float)0;
	    if ((i==2)||(i==3)) robot_rc->leg[i].zero_angle[0]=(float)-0.785;
		robot_rc->leg[i].zero_angle[1]=(float)0.31867;
		robot_rc->leg[i].zero_angle[2]=(float)-1.4887;
	}
}

/// ustawia konczyne w pozycji neutralnej, offset_up - wysokosc podnoszenia stop
bool CGaits_RC::zeroPosition(unsigned char leg_no, float offset_up, float speed)
{
	//leg "leg_no" up
	if (!robot_rc->changeFootRobot(0, 0, offset_up, leg_no, speed))
		return false;

	//leg to neutral position
	int part;
	if (leg_no<3) part=1;
	else part=0;
	CPunctum neutral_point = robot_rc->leg[leg_no].forward_kinematic(robot_rc->leg[leg_no].zero_angle[0],robot_rc->leg[leg_no].zero_angle[1],robot_rc->leg[leg_no].zero_angle[2],part);
	CPunctum end_point = robot_rc->leg[leg_no].forward_kinematic(robot_rc->leg[leg_no].getAngle(0),robot_rc->leg[leg_no].getAngle(1),robot_rc->leg[leg_no].getAngle(2),part);
	end_point = neutral_point-end_point;//zadane przesuniecie do pozycji neutralnej
	if (!robot_rc->changeFoot(end_point.getElement(1,4), end_point.getElement(2,4), end_point.getElement(3,4), leg_no, speed))
		return false;
	return true;
}

/// ustawia konczyny w pozycji neutralnej
bool CGaits_RC::setZeroPosition(float offset_up, float speed)
{
	if (!zeroPosition(0, offset_up, speed)) return false;
	robot_rc->sleepODE(100);
	if (!zeroPosition(3, offset_up, speed)) return false;
	robot_rc->sleepODE(100);
	if (!zeroPosition(1, offset_up, speed)) return false;
	robot_rc->sleepODE(100);
	if (!zeroPosition(4, offset_up, speed)) return false;
	robot_rc->sleepODE(100);
	if (!zeroPosition(2, offset_up, speed)) return false;
	robot_rc->sleepODE(100);
	if (!zeroPosition(5, offset_up, speed)) return false;
	return true;
}

/// prepare to tripod gait - set neutral position of legs and move platform to init level
bool CGaits_RC::tripodPrepare(unsigned int delay_up, unsigned int delay_forward, float x, float y, float z, float alpha, float beta, float gamma, float offset_up, float speed)
{
	//setZeroPosition(offset_up, speed);
	//if (!robot_rc->changePlatformRobot(0, 0, z, 0, 0, 0, speed,1)) return false;
	//unosimy parzyste
	float x_ref[6],y_ref[6],z_ref[6];
	for (int i=0;i<6;i++){
	    	x_ref[i]=0; y_ref[i]=0; 
		if ((i==1)||(i==3)||(i==5)) z_ref[i]=offset_up; else z_ref[i]=0;
	}
	if (!robot_rc->changeAllfeetRobot(x_ref, y_ref, z_ref, speed)) return false;
	robot_rc->sleepODE(delay_up/1000);//usleep(delay_up);
	return true;
}

/// tripod step
bool CGaits_RC::tripodStep(unsigned int delay_up, unsigned int delay_forward, float x, float y, float z, float alpha, float beta, float gamma, float offset_up, float speed,int accel)
{
	x=x/4.0; y=y/4.0; z=z/4.0;
	alpha=alpha/4.0; beta=beta/4.0; gamma=gamma/4.0;
	float x_ref[6],y_ref[6],z_ref[6],alpha_ref[6],beta_ref[6],gamma_ref[6];
	//8 faz ruchu
	//zakrok na nieparzystych
	for (int i=0;i<6;i=i+2){
	    	x_ref[i]=x; y_ref[i]=y; z_ref[i]=z; alpha_ref[i]=alpha; beta_ref[i]=beta; gamma_ref[i]=gamma;
	}
	for (int i=1;i<6;i=i+2){
	    	x_ref[i]=-x; y_ref[i]=-y; z_ref[i]=z; alpha_ref[i]=-alpha; beta_ref[i]=-beta; gamma_ref[i]=-gamma;
	}
	if (!robot_rc->changePlatformRobot(x_ref, y_ref, z_ref, alpha_ref, beta_ref, gamma_ref, speed, accel)) return false;
	robot_rc->sleepODE(delay_forward/1000);//usleep(delay_forward);
	//opuszczamy parzyste
	for (int i=0;i<6;i++){
	    	x_ref[i]=0; y_ref[i]=0; 
		if ((i==1)||(i==3)||(i==5)) z_ref[i]=-offset_up; else z_ref[i]=0;
	}
	//if (!robot_rc->changeAllfeetRobot(x_ref, y_ref, z_ref, speed)) return false;
	if (!robot_rc->Placefeet(0.005,speed)) return false;
	robot_rc->sleepODE(delay_up/1000);//usleep(delay_up);
	//podnosimy nieparzyste
	for (int i=0;i<6;i++){
	    	x_ref[i]=0; y_ref[i]=0; 
		if ((i==0)||(i==2)||(i==4)) z_ref[i]=offset_up; else z_ref[i]=0;
	}
	if (!robot_rc->changeAllfeetRobot(x_ref, y_ref, z_ref, speed)) return false;
	robot_rc->sleepODE(delay_up/1000);//usleep(delay_up);
	//przejscie do polowy na parzystych
	for (int i=0;i<6;i=i+2){
	    	x_ref[i]=-x; y_ref[i]=-y; z_ref[i]=z; alpha_ref[i]=-alpha; beta_ref[i]=-beta; gamma_ref[i]=-gamma;
	}
	for (int i=1;i<6;i=i+2){
	    	x_ref[i]=x; y_ref[i]=y; z_ref[i]=z; alpha_ref[i]=alpha; beta_ref[i]=beta; gamma_ref[i]=gamma;
	}
	if (!robot_rc->changePlatformRobot(x_ref, y_ref, z_ref, alpha_ref, beta_ref, gamma_ref, speed,accel)) return false;
	robot_rc->sleepODE(delay_forward/1000);//usleep(delay_forward);
	//zakrok na parzystych
	if (!robot_rc->changePlatformRobot(x_ref, y_ref, z_ref, alpha_ref, beta_ref, gamma_ref, speed,accel)) return false;
	//opuszczamy nieparzyste
	for (int i=0;i<6;i++){
	    	x_ref[i]=0; y_ref[i]=0; 
		if ((i==0)||(i==2)||(i==4)) z_ref[i]=-offset_up; else z_ref[i]=0;
	}
	//if (!robot_rc->changeAllfeetRobot(x_ref, y_ref, z_ref, speed)) return false;
	if (!robot_rc->Placefeet(0.005,speed)) return false;
	robot_rc->sleepODE(delay_forward/1000);//usleep(delay_forward);
	//podnosimy parzyste
	for (int i=0;i<6;i++){
	    	x_ref[i]=0; y_ref[i]=0; 
		if ((i==1)||(i==3)||(i==5)) z_ref[i]=offset_up; else z_ref[i]=0;
	}
	if (!robot_rc->changeAllfeetRobot(x_ref, y_ref, z_ref, speed)) return false;
	robot_rc->sleepODE(delay_up/1000);//usleep(delay_up);
	// przejscie do pozycji neutralnej na nieparzystych
	for (int i=0;i<6;i=i+2){
	    	x_ref[i]=x; y_ref[i]=y; z_ref[i]=z; alpha_ref[i]=alpha; beta_ref[i]=beta; gamma_ref[i]=gamma;
	}
	for (int i=1;i<6;i=i+2){
	    	x_ref[i]=-x; y_ref[i]=-y; z_ref[i]=z; alpha_ref[i]=-alpha; beta_ref[i]=-beta; gamma_ref[i]=-gamma;
	}
	if (!robot_rc->changePlatformRobot(x_ref, y_ref, z_ref, alpha_ref, beta_ref, gamma_ref, speed,accel)) return false;
	robot_rc->sleepODE(delay_forward/1000);//usleep(delay_forward);
	//if (!robot_rc->Placefeet(0.005,speed)) return false;
	return true;
}

/// short tripod step  - cycle is shorter:triangular trajectory of the leg's tip
bool CGaits_RC::shortTripodStep(unsigned int delay_up, unsigned int delay_forward, float x, float y, float z, float alpha, float beta, float gamma, float offset_up, float speed, int accel){
	x=x/4.0; y=y/4.0; z=z/4.0;
	alpha=alpha/4.0; beta=beta/4.0; gamma=gamma/4.0;
	float x_ref[6],y_ref[6],z_ref[6],alpha_ref[6],beta_ref[6],gamma_ref[6];
	//8 faz ruchu
	//zakrok na nieparzystych
	for (int i=0;i<6;i=i+2){
	    	x_ref[i]=x; y_ref[i]=y; z_ref[i]=z; alpha_ref[i]=alpha; beta_ref[i]=beta; gamma_ref[i]=gamma;
	}
	for (int i=1;i<6;i=i+2){
	    	x_ref[i]=-x; y_ref[i]=-y; z_ref[i]=z+offset_up; alpha_ref[i]=-alpha; beta_ref[i]=-beta; gamma_ref[i]=-gamma;
	}
	if (!robot_rc->changePlatformRobot(x_ref, y_ref, z_ref, alpha_ref, beta_ref, gamma_ref, speed,accel)) return false;
	robot_rc->sleepODE(delay_forward/1000);//usleep(delay_forward);

	//przejscie do polowy na parzystych
	for (int i=0;i<6;i=i+2){
	    	x_ref[i]=-x; y_ref[i]=-y; z_ref[i]=z-offset_up; alpha_ref[i]=-alpha; beta_ref[i]=-beta; gamma_ref[i]=-gamma;
	}
	for (int i=1;i<6;i=i+2){
	    	x_ref[i]=x; y_ref[i]=y; z_ref[i]=z; alpha_ref[i]=alpha; beta_ref[i]=beta; gamma_ref[i]=gamma;
	}
	if (!robot_rc->changePlatformRobot(x_ref, y_ref, z_ref, alpha_ref, beta_ref, gamma_ref, speed, accel)) return false;
	robot_rc->sleepODE(delay_forward/1000);//usleep(delay_forward);
	//zakrok na parzystych
	for (int i=0;i<6;i=i+2){
	    	x_ref[i]=-x; y_ref[i]=-y; z_ref[i]=z+offset_up; alpha_ref[i]=-alpha; beta_ref[i]=-beta; gamma_ref[i]=-gamma;
	}
	if (!robot_rc->changePlatformRobot(x_ref, y_ref, z_ref, alpha_ref, beta_ref, gamma_ref, speed, accel)) return false;
	robot_rc->sleepODE(delay_forward/1000);//usleep(delay_forward);
	// przejscie do pozycji neutralnej na nieparzystych
	for (int i=0;i<6;i=i+2){
	    	x_ref[i]=x; y_ref[i]=y; z_ref[i]=z; alpha_ref[i]=alpha; beta_ref[i]=beta; gamma_ref[i]=gamma;
	}
	for (int i=1;i<6;i=i+2){
	    	x_ref[i]=-x; y_ref[i]=-y; z_ref[i]=z-offset_up; alpha_ref[i]=-alpha; beta_ref[i]=-beta; gamma_ref[i]=-gamma;
	}
	if (!robot_rc->changePlatformRobot(x_ref, y_ref, z_ref, alpha_ref, beta_ref, gamma_ref, speed, accel)) return false;
	robot_rc->sleepODE(delay_forward/1000);//usleep(delay_forward);
	return true;
}


/// finish tripod gait - set neutral position of legs and move platform to init level
bool CGaits_RC::tripodFinish(unsigned int delay_up, unsigned int delay_forward, float x, float y, float z, float alpha, float beta, float gamma, float offset_up, float speed)
{
	//setZeroPosition(offset_up, speed);
	if (!robot_rc->changePlatformRobot(0, 0, 0.0, 0, 0, 0, speed,1)) return false;
	//unosimy parzyste
	float x_ref[6],y_ref[6],z_ref[6];
	for (int i=0;i<6;i++){
	    	x_ref[i]=0; y_ref[i]=0; 
		if ((i==1)||(i==3)||(i==5)) z_ref[i]=-offset_up; else z_ref[i]=0;
	}
	if (!robot_rc->changeAllfeetRobot(x_ref, y_ref, z_ref, speed)) return false;
	robot_rc->sleepODE(delay_up/1000);//usleep(delay_up);
	return true;
}


/// prepare to wave gait - set neutral position of legs and move platform to init level
bool CGaits_RC::wavePrepare(unsigned int delay_up, unsigned int delay_forward, float x, float y, float z, float alpha, float beta, float gamma, float offset_up, float speed, int accel)
{
	float x_ref[6],y_ref[6],z_ref[6],alpha_ref[6],beta_ref[6],gamma_ref[6];

	//setZeroPosition(offset_up, speed);
	//robot_rc->changePlatformRobot(0, 0, z, 0, 0, 0, speed);

	for (int i=0;i<6;i++){	x_ref[i]=0; y_ref[i]=0; z_ref[i]=0; alpha_ref[i]=0; beta_ref[i]=0; gamma_ref[i]=0;} //init

	int i = 0;
	float dividor = 0;
	while (i<6) {
		// noga 1 do gory
		if (!robot_rc->changeFootRobot(0, 0, offset_up, i, speed)) return false;
		robot_rc->sleepODE(delay_up/1000);//usleep(delay_up);

		// noga i do przodu
		x_ref[i]=x*(-0.5+dividor); y_ref[i]=y*(-0.5+dividor); z_ref[i]=-z*(-0.5+dividor); alpha_ref[i]=alpha*(-0.5+dividor); beta_ref[i]=beta*(-0.5+dividor); gamma_ref[i]=gamma*(-0.5+dividor);
		if (!robot_rc->changePlatformRobot(x_ref, y_ref, z_ref, alpha_ref, beta_ref, gamma_ref, speed, accel)) return false;
		robot_rc->sleepODE(delay_forward/1000);//usleep(delay_forward);
		x_ref[i]=0; y_ref[i]=0; z_ref[i]=0; alpha_ref[i]=0; beta_ref[i]=0; gamma_ref[i]=0;

		// noga i na dol
		if (!robot_rc->changeFootRobot(0, 0, -offset_up, i, speed)) return false;
		robot_rc->sleepODE(delay_up/1000);//usleep(delay_up);

		if (i==0) i = 5;
		else if (i==1) i = 4;
		else if (i==2) i = 3;
		else if (i==4) i = 2;
		else if (i==5) i = 1;
		else i = 6; //koniec cyklu
		dividor+=0.2;
	}
	return true;
}


/// wave step
bool CGaits_RC::waveStep(unsigned int delay_up, unsigned int delay_forward, float x, float y, float z, float alpha, float beta, float gamma, float offset_up, float speed, int accel)
{
	float x_ref[6],y_ref[6],z_ref[6],alpha_ref[6],beta_ref[6],gamma_ref[6];
	
	for (int i=0;i<6;i++){	x_ref[i]=0; y_ref[i]=0; z_ref[i]=0; alpha_ref[i]=0; beta_ref[i]=0; gamma_ref[i]=0;} //init

	int leg_no = 3;
	while (leg_no<6) {
		// noga leg_no do gory
		if (!robot_rc->changeFootRobot(0, 0, offset_up, leg_no, speed)) return false;
		z_ref[leg_no]=0;

		// noga leg_no do przodu, pozostale o 1/5 do tylu
		for (int i=0;i<6;i++) {
			if (i==leg_no) {
				x_ref[i]=-x; y_ref[i]=-y; z_ref[i]=-z; alpha_ref[i]=-alpha; beta_ref[i]=-beta; gamma_ref[i]=-gamma;
			}
			else {
				x_ref[i]=x*0.2; y_ref[i]=y*0.2; z_ref[i]=z*0.2; alpha_ref[i]=alpha*0.2; beta_ref[i]=beta*0.2; gamma_ref[i]=gamma*0.2;
			}
		}
		if (!robot_rc->changePlatformRobot(x_ref, y_ref, z_ref, alpha_ref, beta_ref, gamma_ref, speed, accel)) return false;
		robot_rc->sleepODE(delay_forward/1000);//usleep(delay_forward);
	
		// noga leg_no na ziemie
		if (!robot_rc->changeFootRobot(0, 0, -offset_up-z, leg_no, speed)) return false;
		robot_rc->sleepODE(delay_up/1000);//usleep(delay_up);

		if (leg_no==1) leg_no = 5;
		else if (leg_no==2) leg_no = 4;
		else if (leg_no==3) leg_no = 2;
		else if (leg_no==4) leg_no = 1;
		else if (leg_no==5) leg_no = 0;
		else leg_no = 6; //koniec cyklu
	}
	return true;
}

/// short wave step - cycle is shorter:triangular trajectory of the leg's tip
bool CGaits_RC::shortWaveStep(unsigned int delay_up, unsigned int delay_forward, float x, float y, float z, float alpha, float beta, float gamma, float offset_up, float speed, int accel)
{
	float x_ref[6],y_ref[6],z_ref[6],alpha_ref[6],beta_ref[6],gamma_ref[6];
	
	for (int i=0;i<6;i++){	x_ref[i]=0; y_ref[i]=0; z_ref[i]=0; alpha_ref[i]=0; beta_ref[i]=0; gamma_ref[i]=0;} //init

	int leg_no = 3;
	while (leg_no<6) {
		//narastajace ramie trojkata
		for (int i=0;i<6;i++) {
			if (i==leg_no) {
				x_ref[i]=-x/2.0; y_ref[i]=-y/2.0; z_ref[i]=-z/2.0-offset_up; alpha_ref[i]=-alpha/2.0; beta_ref[i]=-beta/2.0; gamma_ref[i]=-gamma/2.0;
			}
			else {
				x_ref[i]=x*0.1; y_ref[i]=y*0.1; z_ref[i]=z*0.1; alpha_ref[i]=alpha*0.1; beta_ref[i]=beta*0.1; gamma_ref[i]=gamma*0.1;
			}
		}
		if (!robot_rc->changePlatformRobot(x_ref, y_ref, z_ref, alpha_ref, beta_ref, gamma_ref, speed, accel)) return false;
		robot_rc->sleepODE(delay_forward/1000);//usleep(delay_forward);
		//opadajace ramie trojkat
		for (int i=0;i<6;i++) {
			if (i==leg_no) {
				x_ref[i]=-x/2.0; y_ref[i]=-y/2.0; z_ref[i]=-z/2.0+offset_up; alpha_ref[i]=-alpha/2.0; beta_ref[i]=-beta/2.0; gamma_ref[i]=-gamma/2.0;
			}
			else {
				x_ref[i]=x*0.1; y_ref[i]=y*0.1; z_ref[i]=z*0.1; alpha_ref[i]=alpha*0.1; beta_ref[i]=beta*0.1; gamma_ref[i]=gamma*0.1;
			}
		}
		if (!robot_rc->changePlatformRobot(x_ref, y_ref, z_ref, alpha_ref, beta_ref, gamma_ref, speed, accel)) return false;
		robot_rc->sleepODE(delay_forward/1000);//usleep(delay_forward);

		if (leg_no==1) leg_no = 5;
		else if (leg_no==2) leg_no = 4;
		else if (leg_no==3) leg_no = 2;
		else if (leg_no==4) leg_no = 1;
		else if (leg_no==5) leg_no = 0;
		else leg_no = 6; //koniec cyklu
	}
	return true;
}


/// prepare to smart gait
bool CGaits_RC::SmartGait(float x, float y, float z, float rotx, float roty, float rotz, float foot_up, float speed, int accel){
	float iter_no = 10;
	float imu_rot[3];
	float x_foot1[6];//wspolrzedne x stop robota
	float y_foot1[6];
	float z_foot1[6];//wspolrzedne z stop robota
	float x_foot_after[6];//wspolrzedne x stop robota
	float y_foot_after[6];
	float z_foot_after[6];//wspolrzedne z stop robota
	float foot_distance[6]; // poslizg
	/*for (int i=0;i<3*iter_no;i++){
		if (!robot_rc->changePlatformRobot(0,0,0.005,0,0,0,speed,accel))//przenosimy platforme
		  return false;
		else
			robot_rc->modifyRobotPosition(0,0,0.005,0,0,0);
		//make scan
		usleep(100000);
		rpccaller_map->makeScan(-1.57, 1.57, 1, robot_rc->robot_global_pos.getElement(1,4),robot_rc->robot_global_pos.getElement(2,4),robot_rc->robot_global_pos.getElement(3,4),robot_rc->robot_platform_pos[3],robot_rc->robot_platform_pos[4],robot_rc->robot_platform_pos[5]);
	}
	for (int i=0;i<3*iter_no;i++){
		if (!robot_rc->changePlatformRobot(0,0,-0.005,0,0,0,speed,accel))//przenosimy platforme
		  return false;
		else
			robot_rc->modifyRobotPosition(0,0,-0.005,0,0,0);
		//make scan
		usleep(100000);
		rpccaller_map->makeScan(-1.57, 1.57, 1, robot_rc->robot_global_pos.getElement(1,4),robot_rc->robot_global_pos.getElement(2,4),robot_rc->robot_global_pos.getElement(3,4),robot_rc->robot_platform_pos[3],robot_rc->robot_platform_pos[4],robot_rc->robot_platform_pos[5]);
	}	*/
        CPunctum movement;
	movement.createTRMatrix(rotx, roty, rotz, x, y, z);
	CPunctum  robot_pos;
	robot_pos = robot_rc->robot_global_pos * movement;//pozycja robota po ruchu
	CPunctum rob_pos = robot_rc->getRobotState();
	z=0.215-rob_pos.getElement(3,4);
	//obliczenie pozycji stop po wykonaniu ruchu
	// obliczenie trajektorii
	float x_foot[6]={0,0,0,0,0,0};
	float y_foot[6]={0,0,0,0,0,0};
	float z_foot[6]={0.0,0,0.0,0,0.0,0};
	for (int i=smart_gait_iterator;i<6;i+=2) z_foot[i]=0.14;
	while (!robot_rc->changeAllfeetRobot(x_foot, y_foot, z_foot, speed)){//move legs up
		for (int i=smart_gait_iterator;i<6;i+=2) z_foot[i]-=0.01;
		if (z_foot[smart_gait_iterator]<=0) 
			return false;
	}
	//SleepODE(500000);
	float x_robot[6]={x/iter_no,x/iter_no,x/iter_no,x/iter_no,x/iter_no,x/iter_no};
	float y_robot[6]={y/iter_no,y/iter_no,y/iter_no,y/iter_no,y/iter_no,y/iter_no};
	float z_robot[6]={z/iter_no,z/iter_no,z/iter_no,z/iter_no,z/iter_no,z/iter_no};
	float alpha_robot[6]={rotx/iter_no,rotx/iter_no,rotx/iter_no,rotx/iter_no,rotx/iter_no,rotx/iter_no};
	float beta_robot[6]={roty/iter_no,roty/iter_no,roty/iter_no,roty/iter_no,roty/iter_no,roty/iter_no};
	float gamma_robot[6]={rotz/iter_no,rotz/iter_no,rotz/iter_no,rotz/iter_no,rotz/iter_no,rotz/iter_no};
	for (int i=smart_gait_iterator;i<6;i+=2) {
	    x_robot[i]=-x_robot[i]; y_robot[i]=-y_robot[i]; z_robot[i]=-z_robot[i];
	    alpha_robot[i]=-alpha_robot[i]; beta_robot[i]=-beta_robot[i]; gamma_robot[i]=-gamma_robot[i];
	}
	for (int n=0;n<6;n++)//odczyt pozycji stopy przed ruchem
		robot_rc->dynamicWorld->rec_robot_leg[n].getLastPoint(&x_foot1[n],&y_foot1[n],&z_foot1[n]);
	for (int i=0;i<iter_no;i++){
		for (int j=smart_gait_iterator;j<6;j+=2){
			x_robot[j]=(i+1)*(-x/iter_no); y_robot[j]=(i+1)*(-y/iter_no); z_robot[j]=(i+1)*(-z/iter_no);
			alpha_robot[j]=(i+1)*(-rotx/iter_no); beta_robot[j]=(i+1)*(-roty/iter_no); gamma_robot[j]=(i+1)*(-rotz/iter_no);
		}
		
		if (!robot_rc->changePlatformRobotNeutral(x_robot,y_robot,z_robot,alpha_robot,beta_robot,gamma_robot,z_foot[smart_gait_iterator]-foot_up,smart_gait_iterator,speed,accel))//przenosimy platforme
			return false;
		else
			robot_rc->modifyRobotPosition(x/iter_no, y/iter_no, z/iter_no, rotx/iter_no, roty/iter_no, rotz/iter_no);
		//make scan
		//SleepODE(100000);
		robot_rc->getRotAngles(imu_rot);
	//	rpccaller_map->makeScan(-1.57, 1.57, 1, robot_rc->robot_global_pos.getElement(1,4),robot_rc->robot_global_pos.getElement(2,4),robot_rc->robot_global_pos.getElement(3,4), imu_rot[0],imu_rot[1],0);
	}
	for (int n=0;n<6;n++)//odczyt pozycji stopy po ruchu
		robot_rc->dynamicWorld->rec_robot_leg[n].getLastPoint(&x_foot_after[n],&y_foot_after[n],&z_foot_after[n]);
	for (int n=0;n<6;n++)//obliczenie poslizgu
		foot_distance[n]=sqrt(pow(x_foot_after[n]-x_foot1[n],2)+pow(y_foot_after[n]-y_foot1[n],2)+pow(z_foot_after[n]-z_foot1[n],2));
	//odczyt aktualnej pozycji
	fprintf(fposlizg,"%f, ",foot_distance[1]+foot_distance[3]+foot_distance[5]); //zapisujemy dane wejsciowe
		
	//SleepODE(500000);
	for (int i=0;i<6;i++) z_foot[i]=0;
//	if (!MoveLegsDown(smart_gait_iterator,0,0,&z_foot[0]))//nieparzyste nogi na ziemie
//	  return false;
	for (int i=smart_gait_iterator;i<6;i+=2) z_foot[i]=-z_foot[i];
	if (!robot_rc->changeAllfeetRobot(x_foot, y_foot, z_foot, speed))//opuszczamy konczyny na dol
	  return false;
	//	if (!robot_rc->Placefeet(0.005, smart_gait_iterator, speed))
	//	  return false;
	//	robot_rc->stabilizePlatform(speed, accel);
//	SleepODE(100);
	robot_rc->stabilizePlatform(0.05, accel);
	if (smart_gait_iterator == 1) //next step
	   smart_gait_iterator = 0;
	else
	   smart_gait_iterator = 1;
	return true;
}