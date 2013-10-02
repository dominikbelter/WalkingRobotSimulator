#include "pathPlanner.h"
#include "robot.h"

CPathPlanner::CPathPlanner(void)
{
	robot_platform_traj.setDelay(0);
}

CPathPlanner::~CPathPlanner(void)
{
}

//Simulate ODE for x miliseconds
void CPathPlanner::SleepODE(int miliseconds)
{
	rpccaller->sleepODE(miliseconds);
	//int step_count=(int)(miliseconds/(robot.dynamicWorld.stepDT/0.001));
	//for (int i=1;i<step_count;i++) {
	//	//gl->display()
	//	robot.dynamicWorld.SimStep();
	//	//Sleep(10);
	//}
}

bool CPathPlanner::EvaluateLineTr(float x, float y, float z, float alpha, float beta, float gamma, int num_steps, int step_dividor) {
	robot_platform_traj.clear();
	CPunctum start_pos = robot->getRobotState();
	float angles[3];
	float position[3];
	for (int i=0;i<3;i++)
		position[i]=start_pos.getElement(i+1,4);
	robot->getRotAngles(angles);
	robot_platform_traj.savePositionAndOrientation(position[0],position[1],position[2],angles[0],angles[1],angles[2]);//punkt poczatkowy
	float dpos[6];
	int points_no=num_steps*step_dividor;
	dpos[0]=(x-position[0])/points_no;
	dpos[1]=(y-position[1])/points_no;
	dpos[2]=(z-position[2])/points_no;
	dpos[3]=(alpha-angles[0])/points_no;
	dpos[4]=(beta-angles[1])/points_no;
	dpos[5]=(gamma-angles[2])/points_no;
	for (int i=0;i<points_no;i++){
		for (int j=0;j<3;j++){
			angles[j]+=dpos[j+3];
			position[j]+=dpos[j];
		}
		robot_platform_traj.savePositionAndOrientation(position[0],position[1],position[2],angles[0],angles[1],angles[2]);//kolejne punkty
	}
	return true;
}

//sprawdza czy punkty lezace na sciezce stopy sa wewnatrz przestrzenii roboczej
bool CPathPlanner::verifyAchievability(CPunctum body, CPunctum foot, int leg_no){
	if (!robot->isFootPositionAvailableGlobal(body,foot.getElement(1,4),foot.getElement(2,4),foot.getElement(3,4),leg_no))
		return false;
	return true;
}

//sprawdza czy punkty lezace na sciezce stopy sa wewnatrz przestrzenii roboczej
bool CPathPlanner::verifyAchievability(CPunctum body, CPunctum * foots, float scale){
	for (int i=0;i<6;i++){
		if (!robot->isFootPositionAvailableGlobal(body,foots[i].getElement(1,4),foots[i].getElement(2,4),foots[i].getElement(3,4),i,scale))
			return false;
	}
//	if (!robot.checkCollisions(body,foots))
//		return false;
	return true;
}

/// wyznacza sekwencje przestawien stop na podstawie "zapasu" kinematycznego
void CPathPlanner::computeSequence(CPunctum * foots_start, CPunctum * body_start, CPunctum * body_finish, int num_iter, int * sequence){
	float dpos[6];
	for (int i=0;i<3;i++){
		dpos[i] = (body_finish->getElement(i+1,4)-body_start->getElement(i+1,4))/num_iter;
		dpos[i+3] = (body_finish->orientation[i]-body_start->orientation[i])/num_iter;
	}
	CPunctum body;
	int seq[6]={-1,-1,-1,-1,-1,-1};
	int iterator = 0;
	for (int i=0;i<num_iter;i++){
		body.createTRMatrix(body_start->orientation[0]+dpos[3]*i,body_start->orientation[1]+dpos[4]*i,body_start->orientation[2]+dpos[5]*i,body_start->getElement(1,4)+dpos[0]*i,body_start->getElement(2,4)+dpos[1]*i,body_start->getElement(3,4)+dpos[2]*i);
		for (int j=0;j<6;j++){
			if (seq[j]==-1){
				if (!robot->isFootPositionAvailableGlobal(body,foots_start[sequence[j]].getElement(1,4),foots_start[sequence[j]].getElement(2,4),foots_start[sequence[j]].getElement(3,4),sequence[j])){
					seq[iterator]=sequence[j];
				}
			}
		}
	}
	int sequence_temp[6];
	for (int i=0;i<6;i++){
		if (seq[i]==-1){//trzeba sprawdzic czy w tablicy seq znajduja sie kolejne nmery nog. Pierwszy znaleziony jest wybierany. kolejnosc wyszukiwana wg. tablicy sequence
			for (int j=0;j<6;j++){
				bool isthere=false;
				for (int k=0;k<6;k++){
					if (sequence[j]==seq[k])
						isthere=true;
				}
				if (!isthere){
					sequence_temp[i]=sequence[j];
					seq[i]=sequence[j];
					break;
				}
			}
		}
		else {
			sequence_temp[i]=seq[i];
		}
	}
	for (int i=0;i<6;i++){
		sequence[i]=sequence_temp[i];
	}
}

//sprawdza czy punkty lezace na sciezce stopy sa wewnatrz przestrzenii roboczej
bool CPathPlanner::verifyAchievability(CPunctum body, CPunctum foot1, CPunctum foot2, CPunctum foot3, CPunctum foot4, CPunctum foot5, CPunctum foot6, float scale){
	if (!robot->isFootPositionAvailableGlobal(body,foot1.getElement(1,4),foot1.getElement(2,4),foot1.getElement(3,4),0,scale))
		return false;
	if (!robot->isFootPositionAvailableGlobal(body,foot2.getElement(1,4),foot2.getElement(2,4),foot2.getElement(3,4),1,scale))
		return false;
	if (!robot->isFootPositionAvailableGlobal(body,foot3.getElement(1,4),foot3.getElement(2,4),foot3.getElement(3,4),2,scale))
		return false;
	if (!robot->isFootPositionAvailableGlobal(body,foot4.getElement(1,4),foot4.getElement(2,4),foot4.getElement(3,4),3,scale))
		return false;
	if (!robot->isFootPositionAvailableGlobal(body,foot5.getElement(1,4),foot5.getElement(2,4),foot5.getElement(3,4),4,scale))
		return false;
	if (!robot->isFootPositionAvailableGlobal(body,foot6.getElement(1,4),foot6.getElement(2,4),foot6.getElement(3,4),5,scale))
		return false;
	float odl = sqrt(pow(foot1.getElement(1,4)-foot2.getElement(1,4),2.0)+pow(foot1.getElement(2,4)-foot2.getElement(2,4),2)+pow(foot1.getElement(3,4)-foot2.getElement(3,4),2));
	if (sqrt(pow(foot1.getElement(1,4)-foot2.getElement(1,4),2.0)+pow(foot1.getElement(2,4)-foot2.getElement(2,4),2)+pow(foot1.getElement(3,4)-foot2.getElement(3,4),2))<0.12) 
		return false;
	odl = 	sqrt(pow(foot2.getElement(1,4)-foot3.getElement(1,4),2.0)+pow(foot2.getElement(2,4)-foot3.getElement(2,4),2)+pow(foot2.getElement(3,4)-foot3.getElement(3,4),2));
	if (sqrt(pow(foot2.getElement(1,4)-foot3.getElement(1,4),2.0)+pow(foot2.getElement(2,4)-foot3.getElement(2,4),2)+pow(foot2.getElement(3,4)-foot3.getElement(3,4),2))<0.12) 
		return false;
	odl = sqrt(pow(foot4.getElement(1,4)-foot5.getElement(1,4),2.0)+pow(foot4.getElement(2,4)-foot5.getElement(2,4),2)+pow(foot4.getElement(3,4)-foot5.getElement(3,4),2));
	if (sqrt(pow(foot4.getElement(1,4)-foot5.getElement(1,4),2.0)+pow(foot4.getElement(2,4)-foot5.getElement(2,4),2)+pow(foot4.getElement(3,4)-foot5.getElement(3,4),2))<0.12) 
		return false;
	odl = sqrt(pow(foot5.getElement(1,4)-foot6.getElement(1,4),2.0)+pow(foot5.getElement(2,4)-foot6.getElement(2,4),2)+pow(foot5.getElement(3,4)-foot6.getElement(3,4),2));
	if (sqrt(pow(foot5.getElement(1,4)-foot6.getElement(1,4),2.0)+pow(foot5.getElement(2,4)-foot6.getElement(2,4),2)+pow(foot5.getElement(3,4)-foot6.getElement(3,4),2))<0.12) 
		return false;
	CPunctum foots[6] = {foot1, foot2, foot3, foot4, foot5, foot6};
	if (robot->checkCollisions(body,foots))
		return false;
	return true;
}