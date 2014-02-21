#include "Crrt.h"
#include <math.h>
#include <boost/timer.hpp> 

Crrt::Crrt(CIdealMap* map, RPCCaller* rpccaller, CRobot* robot)
{
	this->rpccaller=rpccaller;
	this->map=map;
	this->robot=robot;
	CrrtNode temp_node(0,0,0,0,0,0);
	AddNode(-1,&temp_node);
	initializeParameters();
	initializeRand();
	pather = new MicroPather( this);
}

Crrt::Crrt(float x, float y, float z, float alpha, float beta, float gamma, CIdealMap* map, RPCCaller* rpccaller, CRobot* robot)
{
	initializeRand();
	this->rpccaller=rpccaller;
	this->map=map;
	this->robot=robot;
	CrrtNode temp_node(x, y, z, alpha, beta, gamma);
	AddNode(-1,&temp_node);
	initializeParameters();
	pather = new MicroPather( this);
}

Crrt::Crrt(float * position, float * orientation, CIdealMap* map, RPCCaller* rpccaller, CRobot* robot)
{
	initializeRand();
	this->rpccaller=rpccaller;
	this->map=map;
	this->robot=robot;
	CrrtNode temp_node(position, orientation);
	AddNode(-1,&temp_node);
	initializeParameters();
	pather = new MicroPather(this);
}

Crrt::~Crrt(void)
{
	delete pather;
}

/// set root node
void Crrt::setRoot(float * position, float * orientation, CPunctum body, CPunctum * feet){
	CrrtNode temp_node(position, orientation);
	temp_node.body = body;
//	temp_node.even = _even;
	for (int i=0;i<6;i++)
		temp_node.feet[i] = feet[i];
	AddNode(-1,&temp_node);
}

/// initialize parameters
void Crrt::initializeParameters(){
	max_z_rotation = 0.2;
	node_no = 0; // the number of nodes in tree (only parent)
}

///add node
void Crrt::AddNode(int parent, CrrtNode *new_node){
	new_node->SetParentIndex(parent);
	new_node->SetNodeIndex(node_no);
	tree.push_back(*new_node);
	node_no++;
}

///get node
CrrtNode Crrt::getNode(int node_no){
	return tree[node_no];
}

/// gets number of nodes
int Crrt::GetNodeNo()
{
	return node_no;
}

/// sets number of nodes
void Crrt::SetNodeNo(int _node_no)
{
	node_no = _node_no;
}

/// erase vector
void Crrt::erase(){
	tree.erase(tree.begin(),tree.end());
	node_no = 0; // the number of nodes in tree (only parent)
}

/// finds the nearest neighbour
CrrtNode Crrt::NearestNeighbour(CrrtNode test_node)
{
	CrrtNode result = tree[0];
	double distance=dist(tree[0],test_node);
	for (int iter=0; iter<(int)tree.size();iter++) {
		if (dist(tree[iter],test_node)<distance)
		{
			distance=dist(tree[iter],test_node);
			result=tree[iter];
		}
	}
	return result;
}

///computes distance between nodes
double Crrt::dist(CrrtNode node1, CrrtNode node2) { 
	return sqrt(pow(double(node2.pos[0]-node1.pos[0]),2)+pow(double(node2.pos[1]-node1.pos[1]),2));   
}

/// check if the change of the robots position is possible
int Crrt::checkPassing(CPositionRecorder * feet_traj, CPunctum * body_traj, CPunctum *body_start, CPunctum *body_finish, CPunctum *feet_start, CPunctum * feet_finish, float distance2ground, int shift){
	int success_no = 0;
	int start = 0;
	CPunctum foothold;
	for (int smart_gait_iterator=0;smart_gait_iterator<2;smart_gait_iterator++){
		for (int i=smart_gait_iterator;i<6;i+=2) {
			generateFootTraj(feet_traj,feet_start[i], feet_finish[i],*body_start, *body_finish,distance2ground,i, (int)shift/2);//jezeli udalo sie przejsc do kolejnego punktu podparcia
		}
		for (int i=(smart_gait_iterator==0?1:0);i<6;i+=2) {//pozostale nogi w tym czasie sie nie ruszaja
			if (smart_gait_iterator==0)
				foothold.createTRMatrix(0,0,0,feet_start[i].getElement(1,4),feet_start[i].getElement(2,4),feet_start[i].getElement(3,4));
			else
				foothold.createTRMatrix(0,0,0,feet_finish[i].getElement(1,4),feet_finish[i].getElement(2,4),feet_finish[i].getElement(3,4));
			foothold.setFoothold(true);
			for (int j=0;j<(int)shift/2;j++) {
				feet_traj[i].savePunctum(foothold);
			}
		}
	}
	for (int i=0;i<6;i++){
		feet_traj[i].setBegin();
	}
	float delta[6], pos[6];
	float dividor = shift-1;
	for (int i=0;i<3;i++) {
		delta[i] = (body_finish->getElement(i+1,4)-body_start->getElement(i+1,4))/dividor;
		delta[i+3] = (body_finish->orientation[i]-body_start->orientation[i])/dividor;
		pos[i] = body_start->getElement(i+1,4);
		pos[i+3] = body_start->orientation[i];
	}
	CPunctum act_pos;
	CPunctum _feet[6];

	for (int i=0;i<(int)dividor;i++){
		for (int j=0;j<6;j++){
			pos[j]+=delta[j];
			feet_traj[j].getElement(&_feet[j]);
		}
		act_pos.createTRMatrix(pos[3],pos[4],pos[5],pos[0],pos[1],pos[2]);
		body_traj[i]=act_pos;
		
		if (verifyAchievability(act_pos,_feet[0],_feet[1],_feet[2],_feet[3],_feet[4],_feet[5],0.85)){//sprawdz czy osiagalny - miesci sie w przestrzenii roboczej
				success_no=6;
		}
		else
			return 0;
	}
	body_traj[shift-1]=*body_finish;
	return success_no;
}

/// check if the change of the robots position is possible
int Crrt::checkPassingOpt(CPositionRecorder * feet_traj, CPunctum * body_traj, CPunctum *body_start, CPunctum *body_finish, CPunctum *feet_start, CPunctum * feet_finish, float distance2ground, int shift){
	int success_no = 0;
	int start = 0;
	CPunctum foothold;
	CPositionRecorder feet_traj_temp[6];
	for (int smart_gait_iterator=0;smart_gait_iterator<2;smart_gait_iterator++){
		for (int i=smart_gait_iterator;i<6;i+=2) {
			generateFootTraj(feet_traj_temp,feet_start[i], feet_finish[i],*body_start, *body_finish,distance2ground,i, (int)shift/2);//jezeli udalo sie przejsc do kolejnego punktu podparcia
		}
		for (int i=(smart_gait_iterator==0?1:0);i<6;i+=2) {//pozostale nogi w tym czasie sie nie ruszaja
			if (smart_gait_iterator==0)
				foothold.createTRMatrix(0,0,0,feet_start[i].getElement(1,4),feet_start[i].getElement(2,4),feet_start[i].getElement(3,4));
			else
				foothold.createTRMatrix(0,0,0,feet_finish[i].getElement(1,4),feet_finish[i].getElement(2,4),feet_finish[i].getElement(3,4));
			foothold.setFoothold(true);
			for (int j=0;j<(int)shift/2;j++) {
				feet_traj_temp[i].savePunctum(foothold);
			}
		}
	}
	for (int i=0;i<6;i++){
		feet_traj_temp[i].setBegin();
	}
	float delta[6], pos[6];
	float dividor = shift-1;
	for (int i=0;i<3;i++) {
		delta[i] = (body_finish->getElement(i+1,4)-body_start->getElement(i+1,4))/dividor;
		delta[i+3] = (body_finish->orientation[i]-body_start->orientation[i])/dividor;
		pos[i] = body_start->getElement(i+1,4);
		pos[i+3] = body_start->orientation[i];
	}
	CPunctum act_pos;
	CPunctum _feet[6];
	CPunctum _feet_next[6];
	for (int j=0;j<6;j++){
		feet_traj_temp[j].getElement(&_feet_next[j]);
	}

	bool is_fh[6];
	CPunctum opt_pos;
	for (int i=0;i<(int)dividor;i++){
		for (int j=0;j<6;j++){
			pos[j]+=delta[j];
			is_fh[j]=_feet[j].isFoothold();
			_feet[j]=_feet_next[j];
			feet_traj_temp[j].getElement(&_feet_next[j]);
			if (is_fh[j]&&!_feet[j].isFoothold())
				is_fh[j]=true;
			else
				is_fh[j]=false;
		}
		act_pos.createTRMatrix(pos[3],pos[4],pos[5],pos[0],pos[1],pos[2]);

		//if (verifyAchievability(act_pos,_feet[0],_feet[1],_feet[2],_feet[3],_feet[4],_feet[5],0.85)){//sprawdz czy osiagalny - miesci sie w przestrzenii roboczej
		if ((!_feet[1].isFoothold()&&_feet_next[1].isFoothold())){//((i<2)||(i>(dividor-2))||(!_feet[0].isFoothold()&&_feet_next[0].isFoothold())||(!_feet[1].isFoothold()&&_feet_next[1].isFoothold())||is_fh[0]||is_fh[1]){
			if (verifyAchievability(act_pos,_feet[0],_feet[1],_feet[2],_feet[3],_feet[4],_feet[5])){//sprawdz czy osiagalny - miesci sie w przestrzenii roboczej
				body_traj[i]=act_pos;
				for (int j=0;j<6;j++){
					feet_traj[j].savePunctum(_feet[j]);
				}
			}
			else
				return 0;
		}
		else {
			if (optimizePostureSwingApprox(&act_pos,_feet,&opt_pos)){
				if (verifyAchievability(opt_pos,_feet[0],_feet[1],_feet[2],_feet[3],_feet[4],_feet[5])){//sprawdz czy osiagalny - miesci sie w przestrzenii roboczej
					body_traj[i]=opt_pos;
					for (int j=0;j<6;j++){
						feet_traj[j].savePunctum(_feet[j]);
					}
					success_no=6;
				}
				else
					return 0;
			}
			else
				return 0;
		}
	}
	body_traj[shift-1]=*body_finish;
	for (int i=0;i<6;i++)
		feet_traj[i].savePunctum(feet_finish[i]);
	return success_no;
}

/// check if the change of the robots position is possible - wave gait
int Crrt::checkPassingWave(CPositionRecorder * feet_traj, CPunctum * body_traj, CPunctum *body_start, CPunctum *body_finish, CPunctum *feet_start, CPunctum * feet_finish, float distance2ground, int shift){
	int success_no = 0;
	int start = 0;
	CPunctum foothold;
	int sequence[6]={3,2,4,1,5,0};
	for (int i=0;i<6;i++){
		generateFootTraj(feet_traj,feet_start[sequence[i]], feet_finish[sequence[i]],*body_start, *body_finish,distance2ground,sequence[i], shift);//jezeli udalo sie przejsc do kolejnego punktu podparcia
		foothold.createTRMatrix(0,0,0,feet_finish[sequence[i]].getElement(1,4),feet_finish[sequence[i]].getElement(2,4),feet_finish[sequence[i]].getElement(3,4));
		foothold.setFoothold(true);
		for (int j=0;j<shift*(5-i);j++) {
			feet_traj[sequence[i]].savePunctum(foothold);
		}
		for (int j=i+1;j<6;j++){
			foothold.createTRMatrix(0,0,0,feet_start[sequence[j]].getElement(1,4),feet_start[sequence[j]].getElement(2,4),feet_start[sequence[j]].getElement(3,4));
			foothold.setFoothold(true);
			for (int k=0;k<shift;k++) {
				feet_traj[sequence[j]].savePunctum(foothold);
			}
		}
	}
	for (int i=0;i<6;i++){
		feet_traj[i].setBegin();
	}
	float delta[6], pos[6];
	float dividor = 6*shift-1;
	for (int i=0;i<3;i++) {
		delta[i] = (body_finish->getElement(i+1,4)-body_start->getElement(i+1,4))/dividor;
		delta[i+3] = (body_finish->orientation[i]-body_start->orientation[i])/dividor;
		pos[i] = body_start->getElement(i+1,4);
		pos[i+3] = body_start->orientation[i];
	}
	CPunctum act_pos;
	CPunctum _feet[6];
	for (int i=0;i<(int)dividor;i++){
		for (int j=0;j<6;j++){
			pos[j]+=delta[j];
			feet_traj[j].getElement(&_feet[j]);
		}
		act_pos.createTRMatrix(pos[3],pos[4],pos[5],pos[0],pos[1],pos[2]);
		body_traj[i]=act_pos;
		if (verifyAchievability(act_pos,_feet[0],feet_start[1],_feet[2],feet_start[3],_feet[4],feet_start[5])){//sprawdz czy osiagalny - miesci sie w przestrzenii roboczej
				success_no=6;
		}
		else
			return 0;
	}
	body_traj[6*shift-1]=*body_finish;
	return success_no;
}

/// check if the change of the robots position is possible - wave gait
int Crrt::checkPassingFree(CPositionRecorder * feet_traj, CPunctum * body_traj, CPunctum *body_start, CPunctum *body_finish, CPunctum *feet_start, CPunctum * feet_finish, float distance2ground, int shift){
	int success_no = 0;
	int start = 0;
	CPunctum foothold;
	int sequence[6]={3,2,4,1,5,0};
	computeSequence(feet_start, body_start, body_finish, 20, sequence);
	for (int i=0;i<6;i++){
		generateFootTraj(feet_traj,feet_start[sequence[i]], feet_finish[sequence[i]],*body_start, *body_finish,distance2ground,sequence[i], shift);//jezeli udalo sie przejsc do kolejnego punktu podparcia
		foothold.createTRMatrix(0,0,0,feet_finish[sequence[i]].getElement(1,4),feet_finish[sequence[i]].getElement(2,4),feet_finish[sequence[i]].getElement(3,4));
		foothold.setFoothold(true);
		for (int j=0;j<shift*(5-i);j++) {
			feet_traj[sequence[i]].savePunctum(foothold);
		}
		for (int j=i+1;j<6;j++){
			foothold.createTRMatrix(0,0,0,feet_start[sequence[j]].getElement(1,4),feet_start[sequence[j]].getElement(2,4),feet_start[sequence[j]].getElement(3,4));
			foothold.setFoothold(true);
			for (int k=0;k<shift;k++) {
				feet_traj[sequence[j]].savePunctum(foothold);
			}
		}
	}
	for (int i=0;i<6;i++){
		feet_traj[i].setBegin();
	}
	float delta[6], pos[6];
	float dividor = 6*shift-1;
	for (int i=0;i<3;i++) {
		delta[i] = (body_finish->getElement(i+1,4)-body_start->getElement(i+1,4))/dividor;
		delta[i+3] = (body_finish->orientation[i]-body_start->orientation[i])/dividor;
		pos[i] = body_start->getElement(i+1,4);
		pos[i+3] = body_start->orientation[i];
	}
	CPunctum act_pos;
	CPunctum _feet[6];
	for (int i=0;i<(int)dividor;i++){
		for (int j=0;j<6;j++){
			pos[j]+=delta[j];
			feet_traj[j].getElement(&_feet[j]);
		}
		act_pos.createTRMatrix(pos[3],pos[4],pos[5],pos[0],pos[1],pos[2]);
		body_traj[i]=act_pos;
		if (verifyAchievability(act_pos,_feet[0],feet_start[1],_feet[2],feet_start[3],_feet[4],feet_start[5])){//sprawdz czy osiagalny - miesci sie w przestrzenii roboczej
				success_no=6;
		}
		else
			return 0;
	}
	body_traj[6*shift-1]=*body_finish;
	return success_no;
}

/// optimize posture
bool Crrt::optimizePosture(CPunctum *body_init, CPunctum * feet_init, CPunctum *body_opt){
	int pop_size=20; //population size
	SParticle * population = new SParticle[pop_size]; //swarm
	CPunctum current_body; // current body posture 
	float g_fitness=0; // the best fitness
	int g_best=0; // the best particle
	int max_epoch=20;
	SParticle best_particle;

	//initialization
	for (int i=0;i<pop_size;i++){
		population[i].fitness=0;
		for (int j=0;j<3;j++){
			if ((j==2)){
				population[i].pos[j]=body_init->getElement(j+1,4)+randFloat(-0.15,0.15);
			}
			else {
				population[i].pos[j]=body_init->getElement(j+1,4)+randFloat(-0.02,0.02);
			}
			population[i].pos[j+3]=randFloat(-0.5,0.5);
			population[i].p_best[j]=population[i].pos[j];
			population[i].p_best[j+3]=population[i].pos[j+3];
			population[i].p_change[j]=0;
			population[i].p_change[j+3]=0;
		}
		population[i].pos[5]=body_init->orientation[2];
		current_body.createTRMatrix(population[i].pos[3],population[i].pos[4],population[i].pos[5],population[i].pos[0],population[i].pos[1],population[i].pos[2]);
		for (int j=0;j<3;j++) current_body.orientation[j]=population[i].pos[j+3];
		if (!collisionsWithGround(&current_body))
			population[i].fitness=robot->computeStabilityMargin(current_body,feet_init)+robot->computeKinematicMarginApprox(&current_body,feet_init);
		else
			population[i].fitness=0;
		if (population[i].fitness>g_fitness){
			*body_opt = current_body;
			g_fitness=population[i].fitness;
			g_best=i;
			for (int j=0;j<5;j++){
				best_particle.pos[j]=population[i].pos[j];
			}
		}
	}

	// normal iteration of the algorithm
	double test;
	for (int k=0;k<max_epoch;k++){
		for (int i=0;i<pop_size;i++){
			for (int j=0;j<5;j++){
				population[i].p_change[j]=population[i].p_change[j]+0.5*randFloat(0,1)*(best_particle.pos[j]-population[i].pos[j])+0.5*randFloat(0,1)*(population[i].p_best[j]-population[i].pos[j]);
				population[i].pos[j]+=population[i].p_change[j];
			}
			for (int j=0;j<2;j++){
				if (population[i].pos[j]>(body_init->getElement(j+1,4)+0.02)) population[i].pos[j]=body_init->getElement(j+1,4)+0.02;
				if (population[i].pos[j]<(body_init->getElement(j+1,4)-0.02)) population[i].pos[j]=body_init->getElement(j+1,4)-0.02;
			}
			current_body.createTRMatrix(population[i].pos[3],population[i].pos[4],population[i].pos[5],population[i].pos[0],population[i].pos[1],population[i].pos[2]);
			for (int j=0;j<3;j++) current_body.orientation[j]=population[i].pos[j+3];
			if (!collisionsWithGround(&current_body)){
				population[i].fitness=robot->computeStabilityMargin(current_body,feet_init)+robot->computeKinematicMarginApprox(&current_body,feet_init);
				if ((population[i].fitness>1)||(population[i].fitness<0))
					population[i].fitness=0;
				/*if (abs(robot.computeKinematicMarginApprox(current_body,feet_init)-robot.computeKinematicMargin(current_body,feet_init))>0.05){
					float s1 = robot.computeKinematicMargin(current_body,feet_init);
					float s2 = robot.computeKinematicMarginApprox(current_body,feet_init);
					int g=0;
				}*/
			}
			else
				population[i].fitness=0;
			if (population[i].fitness>g_fitness){
				*body_opt = current_body;
				for (int j=2;j<5;j++){
					best_particle.pos[j]=population[i].pos[j];
				}
				g_fitness=population[i].fitness;
				g_best=i;
			}
		}
	}
	delete [] population;
	if (g_fitness<0.02)
		return false;
	else
		return true;
}

/// optimize posture - approx
bool Crrt::optimizePostureApprox(CPunctum *body_init, CPunctum * feet_init, CPunctum *body_opt){
	int pop_size=20; //population size
	SParticle * population = new SParticle[pop_size]; //swarm
	CPunctum current_body; // current body posture 
	float g_fitness=0; // the best fitness
	int g_best=0; // the best particle
	int max_epoch=15;
	SParticle best_particle;

	//for (int j=0;j<6;j++) population[i].feet[j] = feet_init[j];
	//robot->increaseStabilityMargin(body_init,feet_init,0.02);
	//int pos[2];
	//map->CalculateGroundCoordinates(body_init->getElement(1,4), body_init->getElement(2,4),pos);
	body_init->setElement(body_init->getElement(3,4)-0.15,3,4);
	while (collisionsWithGround(body_init)){
		body_init->setElement(body_init->getElement(3,4)+0.05,3,4);
	}
	//initialization
	for (int i=0;i<pop_size;i++){
		population[i].fitness=0;
		for (int j=0;j<3;j++){
			population[i].pos[j]=body_init->getElement(j+1,4);
			if (j==2) population[i].pos[j]=body_init->getElement(j+1,4)+randFloat(-0.05,0.1);
			if (j!=2) population[i].pos[j+3]=randFloat(-0.5,0.5);
			else population[i].pos[j+3]=body_init->orientation[2];
			population[i].p_best[j]=population[i].pos[j];
			population[i].p_best[j+3]=population[i].pos[j+3];
			population[i].p_change[j]=0;
			population[i].p_change[j+3]=0;
		}
		current_body.createTRMatrix(population[i].pos[3],population[i].pos[4],population[i].pos[5],population[i].pos[0],population[i].pos[1],population[i].pos[2]);
		for (int j=0;j<3;j++) current_body.orientation[j]=population[i].pos[j+3];
		//population[i].fitness=robot.computeKinematicMargin(current_body,feet_init);
		population[i].fitness=robot->computeKinematicMarginApprox(&current_body, feet_init);
		//createRobotStateFoothold(population[i].pos, &population[i].pos[3], &current_body, population[i].feet);
		//createFootholds(&current_body, population[i].feet);
		if (population[i].fitness>g_fitness&&!collisionsWithGround(&current_body)){
			*body_opt = current_body;
			g_fitness=population[i].fitness;
			g_best=i;
			for (int j=0;j<5;j++){
				best_particle.pos[j]=population[i].pos[j];
			}
		}
	}

	// normal iteration of the algorithm
	double test;
	for (int k=0;k<max_epoch;k++){
		for (int i=0;i<pop_size;i++){
			for (int j=2;j<5;j++){
				population[i].p_change[j]=population[i].p_change[j]+0.5*randFloat(0,1)*(best_particle.pos[j]-population[i].pos[j])+0.5*randFloat(0,1)*(population[i].p_best[j]-population[i].pos[j]);
				population[i].pos[j]+=population[i].p_change[j];
			}
			current_body.createTRMatrix(population[i].pos[3],population[i].pos[4],population[i].pos[5],population[i].pos[0],population[i].pos[1],population[i].pos[2]);
			//createRobotStateFoothold(population[i].pos, &population[i].pos[3], &current_body, population[i].feet);
			for (int j=0;j<2;j++) current_body.orientation[j]=population[i].pos[j+3];
			population[i].fitness=robot->computeKinematicMarginApprox(&current_body, feet_init);
			if (((population[i].fitness>1)||(population[i].fitness<0)))
				population[i].fitness=0;
			if ((population[i].fitness>g_fitness)&&(!collisionsWithGround(&current_body))){
				*body_opt = current_body;
				for (int j=0;j<6;j++){
					best_particle.pos[j]=population[i].pos[j];
				}
				g_fitness=population[i].fitness;
				g_best=i;
			}
		}
	}
	delete [] population;

	/*while (collisionsWithGround(body_opt)){
		body_opt->setElement(body_opt->getElement(3,4)+0.01,3,4);
	}*/

	for (int i=0;i<20;i++){
		current_body.createTRMatrix(best_particle.pos[3],best_particle.pos[4],best_particle.pos[5],best_particle.pos[0],best_particle.pos[1],best_particle.pos[2]-0.15+i*0.015);
		if (!collisionsWithGround(&current_body)){
			if (robot->computeKinematicMarginApprox(&current_body,best_particle.feet)>g_fitness) {
				*body_opt = current_body;
				for (int j=0;j<6;j++) feet_init[j] = best_particle.feet[j];
			}
		}
	}

	if (g_fitness<0.02)
		return false;
	else
		return true;
}

/// optimize posture - approx
bool Crrt::optimizePostureSwingApprox(CPunctum *body_init, CPunctum * feet_init, CPunctum *body_opt){

//	if (robot.computeStabilityMargin(*body_init,feet_init)<0.12){
//		robot.increaseStabilityMarginSwing(body_init,feet_init,0.02);//tylko w kierunku prostopadlym do ruchu
//	}

	/*while (collisionsWithGround(body_init)){
		body_init->setElement(body_init->getElement(3,4)+0.04,3,4);
	}*/

	//optimizePostureApprox(body_init, feet_init, body_init,true);

	//Dla kazdej uniesionej nogi gradientowo optymalizacja marginesu kinematycznego
	for (int i=0;i<6;i++){
		if (!feet_init[i].isFoothold())
			if (!optimizeSwingGrad(*body_init, &(feet_init[i]),i))
				return false;
	}
	*body_opt=*body_init;
	return true;
}

bool Crrt::optimizeSwingGrad(CPunctum body_init, CPunctum * foot_init, int leg_no){

	CPunctum foot;
	SParticle particle; //swarm
	float g_fitness=0; // the best fitness
	float g_best=0; // the best particle
	int max_epoch=20;
	SParticle best_particle;
	float fitness;
	float dist=0.002;
	bool okej=false;
	foot=*foot_init;
	CPunctum best_foot=*foot_init;
	CPunctum foot_tmp;
	CPunctum D;
	D.createTRMatrix(0,0,0,0,0,0);
	*foot_init = body_init;
	foot_init->setElement(foot.getElement(1,4),1,4); foot_init->setElement(foot.getElement(2,4),2,4); foot_init->setElement(foot.getElement(3,4),3,4);
	int raster[2];
	float height;
	/*
	FILE * plik = fopen("grad_opt.m","w");
	fprintf(plik,"close all; clear all;\n[X,Y]=meshgrid([-30:1:29],[-30:1:29]);\n");
	for (int i=-30;i<30;i++){
			for (int j=-30;j<30;j++){
				particle.pos[0]=0.01*i; particle.pos[1]=0.01*j;
				D.setElement(particle.pos[0],1,4);
				D.setElement(particle.pos[1],3,4);
				foot = (*foot_init) * D;
				fitness=robot.computeKinematicMarginApprox(&body_init,&foot,leg_no);
				fprintf(plik,"X(%d,%d)=%f; Y(%d,%d)=%f; Z(%d,%d)=%f;\n",i+31,j+31,foot.getElement(1,4),i+31,j+31,foot.getElement(3,4),i+31,j+31,fitness);
			}
	}
	fprintf(plik,"surf(X,Y,Z);\nview(0,90);\n");

	fprintf(plik,"x=[%f]; y=[%f]; z=[%f];\n",foot_init->getElement(1,4),foot_init->getElement(2,4),foot_init->getElement(3,4));
	*/
	for (int k=0;k<max_epoch;k++){

		//wyznaczenie gradientu
		int best_dir[3]={-1,-1};
		float best_fit=g_best;
		g_best=robot->computeKinematicMarginApprox(&body_init,foot_init,leg_no);
		if (g_best<0.02)
			dist+=0.01;
		else
			okej=true;

		float max=0;
		for (int i=-1;i<2;i++){ //znalezienie kierunku najwiekszego spadku
			for (int j=-1;j<2;j++){
				particle.pos[0]=dist*i; particle.pos[1]=dist*j;
				D.setElement(particle.pos[0],1,4);
				D.setElement(particle.pos[1],3,4);
				foot = (*foot_init) * D;
				fitness=robot->computeKinematicMarginApprox(&body_init,&foot,leg_no);
				map->CalculateGroundCoordinates(foot.getElement(1,4),foot.getElement(2,4),raster);
				for (int l=-3;l<4;l++){//znalezienie maksymalnej wysokosci
					for (int m=-3;m<4;m++){
						height = map->getHeight(raster[0]+l,raster[1]+m);
						if (height>max) max = height;
					}
				}
				if (fitness>best_fit&&foot.getElement(3,4)>max+0.03){
					best_fit=fitness;
					okej=true;
					best_dir[0]=i;best_dir[1]=j;
				}
			}
		}
		if (okej==true){
			if (best_dir[0]==0&&best_dir[1]==0){
				//fprintf(plik,"figure(); plot3(x,y,z,'LineWidth',3);\nview(0,90);\n");
			//	fclose(plik);
				return true;
			}
			else {
				for (int i=1;i<100;i++){
					D.setElement(best_dir[0]*0.002*i,1,4); 
					D.setElement(best_dir[1]*0.002*i,3,4);
					foot = (*foot_init) * D;
					fitness = robot->computeKinematicMarginApprox(&body_init,&foot,leg_no);
					map->CalculateGroundCoordinates(foot.getElement(1,4),foot.getElement(2,4),raster);
					for (int l=-3;l<4;l++){//znalezienie maksymalnej wysokosci
						for (int m=-3;m<4;m++){
							height = map->getHeight(raster[0]+l,raster[1]+m);
							if (height>max) max = height;
						}
					}
					if (fitness>g_best&&foot.getElement(3,4)>max+0.03){
						//foot_tmp.createTRMatrix(0,0,0,0,0,0);
						//foot_tmp.setElement(foot.getElement(1,4),1,4); foot_tmp.setElement(foot.getElement(2,4),2,4); foot_tmp.setElement(foot.getElement(3,4),3,4);
						//*foot_init = foot;
						best_foot=foot;
				//		fprintf(plik,"x=[x,%f]; y=[y,%f]; z=[z,%f];\n",foot_init->getElement(1,4),foot_init->getElement(2,4),foot_init->getElement(3,4));
						g_best= fitness;
						dist=0.002;
						if (fitness>0.08)
							return true;
					}
					else if (fitness<g_best&&fitness>0){
						*foot_init=best_foot;
						break;				
					}
				}
			}
		}
	}
	if (g_best>0.02)
		return true;
	else
		return false;
}

/// Compute optimal posture - approx
bool Crrt::computeOptimalPosture(CPunctum *body_init, CPunctum * feet_init, CPunctum *body_opt){
	CPunctum current_body; // current body posture 

	current_body=*body_init;
	robot->increaseStabilityMargin(&current_body,feet_init,0.02);


	collisionsWithGround(&current_body);
	robot->computeKinematicMarginApprox(&current_body,feet_init);
	
	return true;
}


///does body collide with ground?
bool Crrt::collisionsWithGround(CPunctum * current_body){
	float scale_factor = map->getScaleFactorX();
	int raster_no = (int) (0.2/scale_factor)+1; //liczba rastrow przypadajaca na polowe szerokosci platformy robota
	float actual_height;
	float pos[3]={current_body->getElement(1,4),current_body->getElement(2,4),current_body->getElement(3,4)};
	float rot_xy[3]={current_body->orientation[0],current_body->orientation[1],current_body->orientation[2]};
	CPunctum robot_height, trans;
	int pos_int[2];
	for (int i=-raster_no;i<raster_no;i++){
		for (int j=-raster_no;j<raster_no;j++){
			//actual_height = map->getHeight(pos[0]+int(i*cos(rot_xy[2])-j*sin(rot_xy[2])),pos[1]+int(i*sin(rot_xy[2])+j*cos(rot_xy[2])));
			trans.createTRMatrix(0,0,0,i*scale_factor,j*scale_factor,0);
			robot_height = (*current_body)*trans;
			map->CalculateGroundCoordinates(robot_height.getElement(1,4),robot_height.getElement(2,4),pos_int);
			actual_height = map->getHeight(pos_int[0],pos_int[1]);
			if (robot_height.getElement(3,4)<(actual_height+0.04))
				return true;
		}
	}
	return false;	
}

///does body collide with ground?
bool Crrt::fastCollisionsWithGround(CPunctum * current_body){
	float scale_factor = map->getScaleFactorX();
	int raster_no = (int) (0.2/scale_factor)+1; //liczba rastrow przypadajaca na polowe szerokosci platformy robota
	float actual_height;
	float pos[3]={current_body->getElement(1,4),current_body->getElement(2,4),current_body->getElement(3,4)};
	float rot_xy[3]={current_body->orientation[0],current_body->orientation[1],current_body->orientation[2]};
	CPunctum robot_height, trans;
	for (int i=-raster_no;i<raster_no;i+=5){
		for (int j=-raster_no;j<raster_no;j+=5){
			actual_height = map->getHeight(pos[0]+int(i*cos(rot_xy[2])-j*sin(rot_xy[2])),pos[1]+int(i*sin(rot_xy[2])+j*cos(rot_xy[2])));
			trans.createTRMatrix(0,0,0,i*scale_factor,j*scale_factor,0);
			robot_height = (*current_body)*trans;
			if (robot_height.getElement(3,4)<(actual_height))
				return true;
		}
	}
	return false;	
}

/// The EXTEND operation
/// return: 0 - trapped
/// return: 1 - reached
/// return: 2 - advanced
int Crrt::ExtendFreeGait(CrrtNode node, CrrtNode * q_new, bool reverse, float distance2ground, int shift, bool can_connect,float * robot_pos)
{
	CrrtNode temp = node;
	CrrtNode q_near = NearestNeighbour(node);//find the nearest neighbour
	///! jezeli mapa lokalna (dokladna ma 1,6m)
	//if (sqrt(pow(q_near.pos[0]-robot_pos[0],2)+pow(q_near.pos[1]-robot_pos[1],2))>0.8)
	//	return ExtendSimple(node, q_new,reverse,distance2ground,shift,can_connect);
	float distance=dist(q_near,node);
	float max_distance = 0.15;

	//make a movement to desired point
	float delta_x = node.pos[0]-q_near.pos[0];
	float delta_y = node.pos[1]-q_near.pos[1];
	
	//obliczenie orientacji wokol osi 'z', jaka powienien posiadac robot
	float rot_z;
	if (!reverse)
		rot_z = atan2(delta_y, delta_x);
	else
		rot_z = atan2(-delta_y, -delta_x);
	rot_z=(rot_z-1.57+q_near.rot[2])/2;
	//obliczenie zmiany orientacji wokol osi z
	rot_z = rot_z - q_near.rot[2];//
	if (rot_z<-3.14) rot_z=-6.28-rot_z;
	else if (rot_z>3.14) rot_z=6.28-rot_z;
	if (rot_z>max_z_rotation)// jezeli rotacja w jednym kroku wieksza od zadanej
		rot_z = max_z_rotation;
	else if (rot_z<-max_z_rotation)
		rot_z = -max_z_rotation;
	//sprawdzenie czy robot moze przejsc do zadanego wezla
	//tworzymy pozycje zwiazana z robotem
	node.rot[2] = q_near.rot[2]+rot_z;
	CPunctum body_node;
	CPunctum feet_node[6];
	int success_no = 0;
	CPositionRecorder feet_traj[6];
	if ((distance<max_distance)&&(can_connect)){
			//if (createRobotStateFoothold(node.pos, node.rot, &body_node, feet_node)){
				//for (int i=0;i<6;i++){
				if (!reverse){
					success_no = checkPassingFree(feet_traj,node.body_traj,&q_near.body,&node.body,q_near.feet,node.feet, distance2ground,shift/6);
				}
				else {
					success_no = checkPassingFree(feet_traj,node.body_traj,&node.body,&q_near.body,node.feet,q_near.feet, distance2ground,shift/6);
				}
				//}
				if (success_no==6) {//if (robot.checkStability(body_node, feet_node, 0)) {
					//if (robot.checkStability(body_node, feet_node, 0)) {
					for (int j=0;j<6;j++){
						feet_traj[j].setBegin();
					}
					for (int k=0;k<shift;k++){
						for (int j=0;j<6;j++){
							feet_traj[j].getElement(&node.legs_traj[j][k]);
						}
					}
					node.body = body_node; 
					for (int k=0;k<6;k++) node.feet[k] = feet_node[k];
					node.pos[0]=node.body.getElement(1,4); node.pos[1]=node.body.getElement(2,4); 
					node.pos[2]=node.body.getElement(3,4);
					node.rot[0]=node.body.orientation[0]; node.rot[1]=node.body.orientation[1]; node.rot[2]=node.body.orientation[2];
					*q_new = node;
					AddNode(q_near.GetNodeIndex(),&node);
					*q_new = node;
					return 1;
				}
			//}
	}

	/// jezeli nie osiagnal zadanego punktu to szukamy najblizszego po drodze
	float x_change, y_change;
	while ((abs(delta_x)>max_distance)||(abs(delta_y)>max_distance)) {
		delta_x/=2; delta_y/=2;
		node.pos[0]+=-delta_x; node.pos[1]+=-delta_y;
	}
	x_change=-delta_x/4;
	y_change=-delta_y/4;
	//if (delta_x>0) x_change=-0.02; else x_change=0.02;
	//if (delta_y>0) y_change=-0.02; else y_change=0.02;
	success_no=0;
	for (int i=0;i<3;i++) {
		node.pos[0]+=x_change; node.pos[1]+=y_change;
		computeOrientationAndHeight(node.pos[0], node.pos[1], &node.pos[2], node.rot, 0.1);
		if ((abs(node.rot[0])>0.15)||(abs(node.rot[1])>0.14)) {//ograniczam duzy przechyl platformy
			node.pos[2]-=0.07;
			body_node.setElement(body_node.getElement(3,4)-0.07,3,4);
		}
		if ((abs(node.rot[0])<2.5)&&(abs(node.rot[1])<2.5)) {//ograniczam duzy przechyl platformy
			bool zmiana=true;
			for (int k=1;k<17;k++){
				body_node.createTRMatrix(node.rot[0], node.rot[1], node.rot[2], node.pos[0], node.pos[1], node.pos[2]);
				if (createRobotStateFoothold(node.pos, node.rot, &body_node, feet_node)){
					success_no=0;
					for (int j=0;j<6;j++)
						feet_traj[j].clear();
					if (!reverse){
						success_no = checkPassingFree(feet_traj,node.body_traj,&q_near.body,&body_node,q_near.feet,feet_node, distance2ground,shift/6);
					}
					else {
						success_no = checkPassingFree(feet_traj,node.body_traj,&body_node,&q_near.body,feet_node,q_near.feet, distance2ground,shift/6);
					}
					//}
					if (success_no==6){
						//float test = robot.computeStabilityMargin(body_node,feet_node);
						//float test1 = robot.computeKinematicMargin(body_node,feet_node);
						//optimizePosture(&body_node, feet_node, &body_node);
						//if (robot.checkStability(body_node, feet_node, 0)) {
						for (int j=0;j<6;j++){
							feet_traj[j].setBegin();
						}
						for (int l=0;l<shift;l++){
							for (int j=0;j<6;j++){
								feet_traj[j].getElement(&node.legs_traj[j][l]);
							}
						}
						node.body = body_node; 
						for (int l=0;l<6;l++) node.feet[l] = feet_node[l];
						node.pos[0]=node.body.getElement(1,4); node.pos[1]=node.body.getElement(2,4); 
						node.pos[2]=node.body.getElement(3,4);
						node.rot[0]=node.body.orientation[0]; node.rot[1]=node.body.orientation[1]; node.rot[2]=node.body.orientation[2];
						*q_new = node;
						AddNode(q_near.GetNodeIndex(),&node);
						*q_new = node;
						return 2;
						//}
					}
				}
				if (zmiana){
					node.pos[2]+=k*0.015;
					body_node.setElement(body_node.getElement(3,4)+k*0.015,3,4);
				}
				else{
					node.pos[2]-=k*0.01;
					body_node.setElement(body_node.getElement(3,4)-k*0.015,3,4);
				}
				zmiana^=1;
				//node.rot[0]+=0.1*(q_near.rot[0]-node.rot[0]);
				//node.rot[1]+=0.1*(q_near.rot[1]-node.rot[0]);
			}
		}
	}
	return 0;
}

///EXTEND with optimization
int Crrt::ExtendOpt(CrrtNode node, CrrtNode *q_new, bool reverse, float distance2ground, int shift, bool can_connect, float *robot_pos){
	CrrtNode temp = node;
	CrrtNode nodes[5];
	CrrtNode q_near = NearestNeighbour(node);//find the nearest neighbour
	float distance=dist(q_near,node);
	float max_distance = 0.15;

	//move to desired point
	float delta_x = node.pos[0]-q_near.pos[0];
	float delta_y = node.pos[1]-q_near.pos[1];
	
	//obliczenie orientacji wokol osi 'z', jaka powienien posiadac robot
	float rot_z,direction;
	direction = atan2(delta_y, delta_x);
	if (!reverse)
		rot_z = atan2(delta_y, delta_x);
	else
		rot_z = atan2(-delta_y, -delta_x);
	rot_z=(rot_z-1.57+q_near.rot[2])/2;
	//obliczenie zmiany orientacji wokol osi z
	rot_z = rot_z - q_near.rot[2];//
	if (rot_z<-3.14) rot_z=-6.28-rot_z;
	else if (rot_z>3.14) rot_z=6.28-rot_z;
	if (rot_z>max_z_rotation)// jezeli rotacja w jednym kroku wieksza od zadanej
		rot_z = max_z_rotation;
	else if (rot_z<-max_z_rotation)
		rot_z = -max_z_rotation;
	//sprawdzenie czy robot moze przejsc do zadanego wezla
	//tworzymy pozycje zwiazana z robotem
	node.rot[2] = q_near.rot[2]+rot_z;
	CPunctum body_node;
	CPunctum feet_node[6];
	int success_no = 0;
	CPositionRecorder feet_traj[6];

	rrt_goalx[0]=node.pos[0];
	rrt_goaly[0]=node.pos[1];
	rrt_goalz[0]=0.1;

	///obliczanie kosztu przejscia
	int pos_int[2];
	float init_pos[3];
	float dest_pos[3];
	if (!reverse){
		dest_pos[0]=node.pos[0]; dest_pos[1]=node.pos[1]; dest_pos[2]=node.pos[2];
		init_pos[0]=q_near.pos[0]; init_pos[1]=q_near.pos[1]; init_pos[2]=q_near.pos[2];
	} else {
		dest_pos[0]=q_near.pos[0]; dest_pos[1]=q_near.pos[1]; dest_pos[2]=q_near.pos[2];
		init_pos[0]=node.pos[0]; init_pos[1]=node.pos[1]; init_pos[2]=node.pos[2];
	}
	map->CalculateGroundCoordinates(dest_pos[0],dest_pos[1], pos_int);
	dest_pos[2]=map->getHeight(pos_int[0],pos_int[1])+0.1;
	float variance = map->ComputeSphericalVariance(4, pos_int[0], pos_int[1]);
	float distance1 = sqrt(pow(dest_pos[0]-init_pos[0],2)+pow(dest_pos[1]-init_pos[1],2)+pow(dest_pos[2]-init_pos[2],2))/0.6;
	float traverse_cost=(variance+distance1)/2;

	float traverse_cost_thresh=10000.1;
	if ((distance<max_distance)&&(can_connect)){
			//if (createRobotStateFoothold(node.pos, node.rot, &body_node, feet_node)){
				//for (int i=0;i<6;i++){
				if (!reverse){
					if (traverse_cost>traverse_cost_thresh)
						success_no = checkPassingFree(feet_traj,node.body_traj,&q_near.body,&node.body,q_near.feet,node.feet, distance2ground,shift/6);
					else
						success_no = checkPassingOpt(feet_traj,node.body_traj,&q_near.body,&node.body,q_near.feet,node.feet, distance2ground,shift);
				}
				else {
					if (traverse_cost>traverse_cost_thresh)
						success_no = checkPassingFree(feet_traj,node.body_traj,&node.body,&q_near.body,node.feet,q_near.feet, distance2ground,shift/6);
					else
						success_no = checkPassingOpt(feet_traj,node.body_traj,&node.body,&q_near.body,node.feet,q_near.feet, distance2ground,shift);
				}
				//}
				if (success_no==6) {//if (robot.checkStability(body_node, feet_node, 0)) {
					//if (robot.checkStability(body_node, feet_node, 0)) {
					for (int j=0;j<6;j++){
						feet_traj[j].setBegin();
					}
					for (int k=0;k<shift;k++){
						for (int j=0;j<6;j++){
							feet_traj[j].getElement(&node.legs_traj[j][k]);
						}
					}
					for (int k=0;k<6;k++) node.feet[k] = feet_node[k];
					if (traverse_cost>traverse_cost_thresh)
						node.gait_type=0;
					else
						node.gait_type=3;
					node.pos[0]=node.body.getElement(1,4); node.pos[1]=node.body.getElement(2,4); 
					node.pos[2]=node.body.getElement(3,4);
					node.rot[0]=node.body.orientation[0]; node.rot[1]=node.body.orientation[1]; node.rot[2]=node.body.orientation[2];
					*q_new = node;
					AddNode(q_near.GetNodeIndex(),&node);
					*q_new = node;
					return 1;
				}
			//}
	}

	/// jezeli nie osiagnal zadanego punktu to szukamy najblizszego po drodze
	float x_change, y_change;
	CPunctum trans, tmp;
	int int_pos[2];
	for (int i=0;i<5;i++) nodes[i]=node;
	for (int i=0;i<30;i++){
		trans.createTRMatrix(0,0,i*node.rot[2]/20,i*0.01*cos(direction),i*0.01*sin(direction),0);
		tmp = q_near.body*trans;
		if (!verifyAchievability(tmp,q_near.feet[0],q_near.feet[1],q_near.feet[2],q_near.feet[3],q_near.feet[4],q_near.feet[5],0.9)){
			for (int j=0;j<5;j++){
				nodes[j].rot[0]=0; nodes[j].rot[1]=0; nodes[j].rot[2]=i*nodes[j].rot[2]/40;
				trans.createTRMatrix(0,0,i*nodes[j].rot[2]/40,i*0.01*cos(direction)*0.2*j,i*0.01*sin(direction)*0.2*j,0.0);
				nodes[j].body=q_near.body*trans;
				map->CalculateGroundCoordinates(nodes[j].pos[0], nodes[j].pos[1],int_pos);
				nodes[j].pos[0]=nodes[j].body.getElement(1,4); nodes[j].pos[1]=nodes[j].body.getElement(2,4); nodes[j].pos[2]=map->getHeight(int_pos[0],int_pos[1])+0.1;
			}
			break;
		}
	}

	int best_node=-1;
	for (int j=0;j<5;j++){
		rrt_goalx[1]=nodes[j].pos[0];
		rrt_goaly[1]=nodes[j].pos[1];
		rrt_goalz[1]=nodes[j].pos[2];
		if (!createRobotStateFoothold(nodes[j].pos, nodes[j].rot, &nodes[j].body, nodes[j].feet))
			return 0;

			///obliczanie kosztu przejscia
		if (!reverse){
			dest_pos[0]=nodes[j].pos[0]; dest_pos[1]=nodes[j].pos[1]; dest_pos[2]=nodes[j].pos[2];
			init_pos[0]=q_near.pos[0]; init_pos[1]=q_near.pos[1]; init_pos[2]=q_near.pos[2];
		} else {
			dest_pos[0]=q_near.pos[0]; dest_pos[1]=q_near.pos[1]; dest_pos[2]=q_near.pos[2];
			init_pos[0]=nodes[j].pos[0]; init_pos[1]=nodes[j].pos[1]; init_pos[2]=nodes[j].pos[2];
		}
		map->CalculateGroundCoordinates(dest_pos[0],dest_pos[1], pos_int);
		variance = map->ComputeSphericalVariance(4, pos_int[0], pos_int[1]);
		dest_pos[2]=map->getHeight(pos_int[0],pos_int[1])+0.1;
		distance1 = sqrt(pow(dest_pos[0]-init_pos[0],2)+pow(dest_pos[1]-init_pos[1],2)+pow(dest_pos[2]-init_pos[2],2))/0.6;
		traverse_cost=(variance+distance1)/2;

	//	startTimeMeasure();
		if (optimizePostureApprox(&(nodes[j].body), nodes[j].feet, &(nodes[j].body))){
		//	double wynik = stopTimeMeasure();
			success_no=0;
			for (int k=0;k<6;k++)
				feet_traj[k].clear();
			if (!reverse){
				if (traverse_cost>traverse_cost_thresh)
					success_no = checkPassingFree(feet_traj,nodes[j].body_traj,&q_near.body,&nodes[j].body,q_near.feet,nodes[j].feet, distance2ground,shift/6);
				else
					success_no = checkPassingOpt(feet_traj,nodes[j].body_traj,&q_near.body,&nodes[j].body,q_near.feet,nodes[j].feet, distance2ground,shift);
			}
			else {
				if (traverse_cost>traverse_cost_thresh)
					success_no = checkPassingFree(feet_traj,nodes[j].body_traj,&nodes[j].body,&q_near.body,nodes[j].feet,q_near.feet, distance2ground,shift/6);
				else
					success_no = checkPassingOpt(feet_traj,nodes[j].body_traj,&nodes[j].body,&q_near.body,nodes[j].feet,q_near.feet, distance2ground,shift);
			}
			if (success_no==6){
				for (int k=0;k<6;k++){
					feet_traj[k].setBegin();
				}
				for (int l=0;l<shift;l++){
					for (int k=0;k<6;k++){
						feet_traj[k].getElement(&nodes[j].legs_traj[k][l]);
					}
				}
				if (traverse_cost>traverse_cost_thresh)
					nodes[j].gait_type=0;
				else
					nodes[j].gait_type=3;
				nodes[j].pos[0]=nodes[j].body.getElement(1,4); nodes[j].pos[1]=nodes[j].body.getElement(2,4); 
				nodes[j].pos[2]=nodes[j].body.getElement(3,4);
				nodes[j].rot[0]=nodes[j].body.orientation[0]; nodes[j].rot[1]=nodes[j].body.orientation[1]; nodes[j].rot[2]=nodes[j].body.orientation[2];
				best_node=j;
			}
		}
	}

	if (best_node!=-1){
		*q_new = nodes[best_node];
		AddNode(q_near.GetNodeIndex(),&nodes[best_node]);
		*q_new = nodes[best_node];
		return 2;
	}
	else
		return 0;
}

/// The EXTEND operation
/// return: 0 - trapped
/// return: 1 - reached
/// return: 2 - advanced
int Crrt::Extend(CrrtNode node, CrrtNode * q_new, bool reverse, float distance2ground, int shift, bool can_connect,float * robot_pos)
{
	CrrtNode temp = node;
	CrrtNode q_near = NearestNeighbour(node);//find the nearest neighbour
	///! jezeli mapa lokalna (dokladna ma 1,6m)
	//if (sqrt(pow(q_near.pos[0]-robot_pos[0],2)+pow(q_near.pos[1]-robot_pos[1],2))>0.8)
	//	return ExtendSimple(node, q_new,reverse,distance2ground,shift,can_connect);
	float distance=dist(q_near,node);
	float max_distance = 0.15;

	//make a movement to desired point
	float delta_x = node.pos[0]-q_near.pos[0];
	float delta_y = node.pos[1]-q_near.pos[1];
	
	//obliczenie orientacji wokol osi 'z', jaka powienien posiadac robot
	float rot_z;
	if (!reverse)
		rot_z = atan2(delta_y, delta_x);
	else
		rot_z = atan2(-delta_y, -delta_x);
	rot_z=(rot_z-1.57+q_near.rot[2])/2;
	//obliczenie zmiany orientacji wokol osi z
	rot_z = rot_z - q_near.rot[2];//
	if (rot_z<-3.14) rot_z=-6.28-rot_z;
	else if (rot_z>3.14) rot_z=6.28-rot_z;
	if (rot_z>max_z_rotation)// jezeli rotacja w jednym kroku wieksza od zadanej
		rot_z = max_z_rotation;
	else if (rot_z<-max_z_rotation)
		rot_z = -max_z_rotation;
	//sprawdzenie czy robot moze przejsc do zadanego wezla
	//tworzymy pozycje zwiazana z robotem
	node.rot[2] = q_near.rot[2]+rot_z;
	CPunctum body_node;
	CPunctum feet_node[6];
	int success_no = 0;
	CPositionRecorder feet_traj[6];
	if ((distance<max_distance)&&(can_connect)){
			//if (createRobotStateFoothold(node.pos, node.rot, &body_node, feet_node)){
				//for (int i=0;i<6;i++){
				if (!reverse){
					success_no = checkPassing(feet_traj,node.body_traj,&q_near.body,&node.body,q_near.feet,node.feet, distance2ground,shift);
				}
				else {
					success_no = checkPassing(feet_traj,node.body_traj,&node.body,&q_near.body,node.feet,q_near.feet, distance2ground,shift);
				}
				//}
				if (success_no==6) {//if (robot.checkStability(body_node, feet_node, 0)) {
					//if (robot.checkStability(body_node, feet_node, 0)) {
					for (int j=0;j<6;j++){
						feet_traj[j].setBegin();
					}
					for (int k=0;k<shift;k++){
						for (int j=0;j<6;j++){
							feet_traj[j].getElement(&node.legs_traj[j][k]);
						}
					}
					node.body = body_node; 
					for (int k=0;k<6;k++) node.feet[k] = feet_node[k];
					node.pos[0]=node.body.getElement(1,4); node.pos[1]=node.body.getElement(2,4); 
					node.pos[2]=node.body.getElement(3,4);
					node.rot[0]=node.body.orientation[0]; node.rot[1]=node.body.orientation[1]; node.rot[2]=node.body.orientation[2];
					*q_new = node;
					AddNode(q_near.GetNodeIndex(),&node);
					*q_new = node;
					return 1;
				}
			//}
	}

	/// jezeli nie osiagnal zadanego punktu to szukamy najblizszego po drodze
	float x_change, y_change;
	while ((abs(delta_x)>max_distance)||(abs(delta_y)>max_distance)) {
		delta_x/=2; delta_y/=2;
		node.pos[0]+=-delta_x; node.pos[1]+=-delta_y;
	}
	x_change=-delta_x/4;
	y_change=-delta_y/4;
	//if (delta_x>0) x_change=-0.02; else x_change=0.02;
	//if (delta_y>0) y_change=-0.02; else y_change=0.02;
	success_no=0;
	for (int i=0;i<3;i++) {
		node.pos[0]+=x_change; node.pos[1]+=y_change;
		computeOrientationAndHeight(node.pos[0], node.pos[1], &node.pos[2], node.rot, 0.1);
		if ((abs(node.rot[0])>0.15)||(abs(node.rot[1])>0.14)) {//ograniczam duzy przechyl platformy
			node.pos[2]-=0.07;
			body_node.setElement(body_node.getElement(3,4)-0.07,3,4);
		}
		if ((abs(node.rot[0])<2.5)&&(abs(node.rot[1])<2.5)) {//ograniczam duzy przechyl platformy
			bool zmiana=true;
			for (int k=1;k<17;k++){
					body_node.createTRMatrix(node.rot[0], node.rot[1], node.rot[2], node.pos[0], node.pos[1], node.pos[2]);
				if (createRobotStateFoothold(node.pos, node.rot, &body_node, feet_node)){
					success_no=0;
					for (int j=0;j<6;j++)
						feet_traj[j].clear();
					if (!reverse){
						success_no = checkPassing(feet_traj,node.body_traj,&q_near.body,&body_node,q_near.feet,feet_node, distance2ground,shift);
					}
					else {
						success_no = checkPassing(feet_traj,node.body_traj,&body_node,&q_near.body,feet_node,q_near.feet, distance2ground,shift);
					}
					//}
					if (success_no==6){
						//if (robot.checkStability(body_node, feet_node, 0)) {
						for (int j=0;j<6;j++){
							feet_traj[j].setBegin();
						}
						for (int l=0;l<shift;l++){
							for (int j=0;j<6;j++){
								feet_traj[j].getElement(&node.legs_traj[j][l]);
							}
						}
						node.body = body_node; 
						for (int l=0;l<6;l++) node.feet[l] = feet_node[l];
						node.pos[0]=node.body.getElement(1,4); node.pos[1]=node.body.getElement(2,4); 
						node.pos[2]=node.body.getElement(3,4);
						node.rot[0]=node.body.orientation[0]; node.rot[1]=node.body.orientation[1]; node.rot[2]=node.body.orientation[2];
						*q_new = node;
						AddNode(q_near.GetNodeIndex(),&node);
						*q_new = node;
						return 2;
						//}
					}
				}
				if (zmiana){
					node.pos[2]+=k*0.015;
					body_node.setElement(body_node.getElement(3,4)+k*0.015,3,4);
				}
				else{
					node.pos[2]-=k*0.01;
					body_node.setElement(body_node.getElement(3,4)-k*0.015,3,4);
				}
				zmiana^=1;
				//node.rot[0]+=0.1*(q_near.rot[0]-node.rot[0]);
				//node.rot[1]+=0.1*(q_near.rot[1]-node.rot[0]);
			}
		}
	}
	return 0;
}

int Crrt::ExtendSimple(CrrtNode node, CrrtNode * q_new, bool reverse, float distance2ground, int shift, bool can_connect)
{
	CrrtNode temp = node;
	CrrtNode q_near = NearestNeighbour(node);//find the nearest neighbour
	float distance=dist(q_near,node);
	float max_distance = 0.15;

	//make a movement to desired point
	float delta_x = node.pos[0]-q_near.pos[0];
	float delta_y = node.pos[1]-q_near.pos[1];
	
	int poz_int[3];
	map->CalculateGroundCoordinates(node.pos[0],node.pos[1],poz_int);
	node.pos[2]=map->getMaxSquareHeight(poz_int[0],poz_int[1],8)+0.075;
	//sprawdzenie czy robot moze przejsc do zadanego wezla
	if ((distance<max_distance)&&(can_connect)){
			//if (createRobotStateFoothold(node.pos, node.rot, &body_node, feet_node)){
				//for (int i=0;i<6;i++){
				if (checkTraverse(q_near.pos, node.pos)){
					node.body.setElement(node.pos[0],1,4); node.body.setElement(node.pos[1],2,4); 
					node.body.setElement(node.pos[2],3,4);
					*q_new = node;
					AddNode(q_near.GetNodeIndex(),&node);
					*q_new = node;
					return 1;
				}
			//}
	}

	/// jezeli nie osiagnal zadanego punktu to szukamy najblizszego po drodze
	float x_change, y_change;
	while ((abs(delta_x)>max_distance)||(abs(delta_y)>max_distance)) {
		delta_x/=2; delta_y/=2;
		node.pos[0]+=-delta_x; node.pos[1]+=-delta_y;
	}
	x_change=-delta_x/4;
	y_change=-delta_y/4;

	int kinematic;
	for (int i=0;i<3;i++) {
		node.pos[0]+=x_change; node.pos[1]+=y_change;
		map->CalculateGroundCoordinates(node.pos[0],node.pos[1],poz_int);
		node.pos[2]=map->getMaxSquareHeight(poz_int[0],poz_int[1],8)+0.075;
		if (checkTraverse(q_near.pos, node.pos)){
			node.body.setElement(node.pos[0],1,4); node.body.setElement(node.pos[1],2,4); 
			node.body.setElement(node.pos[2],3,4);
			*q_new = node;
			AddNode(q_near.GetNodeIndex(),&node);
			*q_new = node;
			return 2;
		}
		else{
			node.pos[2]+=0.05;
			if (checkTraverse(q_near.pos, node.pos)){
				node.body.setElement(node.pos[0],1,4); node.body.setElement(node.pos[1],2,4); 
				node.body.setElement(node.pos[2],3,4);
				*q_new = node;
				AddNode(q_near.GetNodeIndex(),&node);
				*q_new = node;
				return 2;
			}
			else {
				node.pos[2]+=0.06;
				if (checkTraverse(q_near.pos, node.pos)){
					node.body.setElement(node.pos[0],1,4); node.body.setElement(node.pos[1],2,4); 
					node.body.setElement(node.pos[2],3,4);
					*q_new = node;
					AddNode(q_near.GetNodeIndex(),&node);
					*q_new = node;
					return 2;
				}
			}
		}
	}
	return 0;
}

/// compute orientation according to terrain shape
void Crrt::computeOrientationAndHeight(float x, float y, float * height, float * rot_xy, float distance2ground){
	int pos[2];
	map->CalculateGroundCoordinates((double)x, (double) y,pos);
	float scale_factor = map->getScaleFactorX();
	int raster_no = (int) (0.27/scale_factor)+1; //liczba rastrow przypadajaca na polowe szerokosci platformy robota
	float max_height_f[3]; max_height_f[2]= map->getHeight(pos[0],pos[1]);
	max_height_f[0] = 0;	max_height_f[1] = 0;

	float max_height_b[3]; max_height_b[2]= map->getHeight(pos[0],pos[1]);
	max_height_b[0] = 0;	max_height_b[1] = 0;

	float max_height_l[3]; max_height_l[2]= map->getHeight(pos[0],pos[1]);
	max_height_l[0] = 0;	max_height_l[1] = 0;

	float max_height_r[3]; max_height_r[2]= map->getHeight(pos[0],pos[1]);
	max_height_r[0] = 0;	max_height_r[1] = 0;

	float max_height[3]={0,0,-1e18};
	float actual_height;
	for (int i=-raster_no;i<raster_no;i++){
		for (int j=-raster_no;j<raster_no;j++){
			actual_height = map->getHeight(pos[0]+int(i*cos(rot_xy[2])-j*sin(rot_xy[2])),pos[1]+int(i*sin(rot_xy[2])+j*cos(rot_xy[2])));
			if (j>0) {
				if (actual_height+abs(i)*0.00005+abs(j)*0.00005>max_height_f[2]){
					max_height_f[0] = i*scale_factor;
					max_height_f[1] = j*scale_factor;
					max_height_f[2] = actual_height+abs(i)*0.00005+abs(j)*0.00005;
				}
			}
			if (j<=0) {
				if (actual_height+abs(i)*0.00005+abs(j)*0.00005>max_height_b[2]){
					max_height_b[0] = i*scale_factor;
					max_height_b[1] = j*scale_factor;
					max_height_b[2] = actual_height+abs(i)*0.00005+abs(j)*0.00005;
				}
			}
			if (i>0) {
				if (actual_height+abs(i)*0.00005+abs(j)*0.00005>max_height_r[2]){
					max_height_r[0] = i*scale_factor;
					max_height_r[1] = j*scale_factor;
					max_height_r[2] = actual_height+abs(i)*0.00005+abs(j)*0.00005;
				}
			}
			if (i<=0) {
				if (actual_height+abs(i)*0.00005+abs(j)*0.00005>max_height_l[2]){
					max_height_l[0] = i*scale_factor;
					max_height_l[1] = j*scale_factor;
					max_height_l[2] = actual_height+abs(i)*0.00005+abs(j)*0.00005;
				}
			}
			if (actual_height>max_height[2]){
				max_height[0] = i*scale_factor;
				max_height[1] = j*scale_factor;
				max_height[2] = actual_height;
			}
		}
	}
	float a_x,a_y,b_x,b_y;
	if (abs(max_height_f[1]-max_height_b[1])<0.06){
		rot_xy[0]=0;
		a_x=0; b_x=0;
	}
	else {
		a_x=((max_height_f[2]-max_height_b[2])/(max_height_f[1]-max_height_b[1]));
		b_x=(((max_height_f[1]-max_height_b[1])*max_height_b[2]-(max_height_f[2]-max_height_b[2])*max_height_b[1])/(max_height_f[1]-max_height_b[1]));
		rot_xy[0]=-atan(double(a_x));
		if (rot_xy[0]>0.2){
			rot_xy[0]=0.2;
			a_x=-0.2;
		}
		else if (rot_xy[0]<-0.2){
			rot_xy[0]=-0.2;
			a_x=0.2;
		}
	}
	if (abs(max_height_r[0]-max_height_l[0])<0.06){
		rot_xy[1]=0;
		a_y=0; b_y=0;
	}
	else {//-((y2-y1)/(x2-x1))*x2+y2+x*((y2-y1)/(x2-x1))=y
		a_y=((max_height_r[2]-max_height_l[2])/(max_height_r[0]-max_height_l[0]));
		b_y=(((max_height_r[0]-max_height_l[0])*max_height_l[2]-(max_height_r[2]-max_height_l[2])*max_height_l[0])/(max_height_r[0]-max_height_l[0]));
		rot_xy[1]=-atan(double(a_y));
		if (rot_xy[1]>0.15){
			rot_xy[1]=0.15;
			a_x=-0.15;
		}
		else if (rot_xy[1]<-0.15){
			rot_xy[1]=-0.15;
			a_x=-0.15;
		}
	}
	float height_maxx = -1e18;//a_x *max_height[1]+ b_x;//wysokosc prostej nad najwyzszym punktem dla osi x
	float height_maxy = -1e18;//a_y *max_height[0]+ b_y;//wysokosc prostej nad najwyzszym punktem dla osi y
	float delta_x = 0;//max_height[2] - height_maxx+distance2ground;// o ile trzeba podniesc wysokosc robota wynikajaca z osi x
	float delta_y = 0;//max_height[2] - height_maxy+distance2ground; // o ile trzeba podniesc wysokosc robota wynikajaca z osi y
	float height_temp;
	for (int i=-raster_no;i<raster_no;i++){
		for (int j=-raster_no;j<raster_no;j++){
			height_maxx = -a_x *j*scale_factor + b_x;//wysokosc prostej nad najwyzszym punktem dla osi x
			height_maxy = -a_y *i*scale_factor + b_y;//wysokosc prostej nad najwyzszym punktem dla osi y
			height_temp = map->getHeight(pos[0]+int(i*cos(rot_xy[2])-j*sin(rot_xy[2])),pos[1]+int(i*sin(rot_xy[2])+j*cos(rot_xy[2])));
			if (delta_x < (height_temp - height_maxx+distance2ground))
				delta_x = height_temp - height_maxx+distance2ground;// o ile trzeba podniesc wysokosc robota wynikajaca z osi x
			if (delta_y < (height_temp - height_maxy+distance2ground))
				delta_y = height_temp - height_maxy+distance2ground; // o ile trzeba podniesc wysokosc robota wynikajaca z osi y
		}
	}
	if (delta_x>delta_y)
		*height = b_x+delta_x;
	else
		*height = b_y+delta_y;
}

/// get full body state Foothold
bool Crrt::createRobotState(float pos_end[], float rot_end[], CPunctum * body, CPunctum *feet){
	//body->createTRMatrix(rot_end[0], rot_end[1], rot_end[2], pos_end[0], pos_end[1], pos_end[2]);
	int r[2];
	CPunctum foothold, _body=*body;
	int part;
	float pos[3];
	float dist_offset=0;
	for (int i=0;i<6;i++) {
		(i<3) ? part=1 : part=-1;
		feet[i] = (*body)*robot->leg[i].start*robot->leg[i].getNeutralPosition(part);
		map->CalculateGroundCoordinates((float)feet[i].getElement(1,4), (float)feet[i].getElement(2,4),r);
		map->CalculateGlobalCoordinates(r[0],r[1],pos);
		pos[2] = map->getHeight(r[0],r[1]);
		feet[i].setPos(pos);
	}
	for (int i=0;i<3;i++) body->orientation[i]=rot_end[i];
	return true;
}

/// get full body state Foothold
bool Crrt::createRobotStateFoothold(float pos_end[], float rot_end[], CPunctum * body, CPunctum *feet){
	body->createTRMatrix(rot_end[0], rot_end[1], rot_end[2], pos_end[0], pos_end[1], pos_end[2]);
	float pos[3];
	createRobotState(pos_end, rot_end, body, feet);
	//if (!createFootholds(body, feet))
//		return false;
	return true;
}

/// get footholds
bool Crrt::createFootholds(CPunctum * body, CPunctum *feet){
	//body->createTRMatrix(rot_end[0], rot_end[1], rot_end[2], pos_end[0], pos_end[1], pos_end[2]);
	int r[2];
	int part;
	float pos[3];
	for (int i=0;i<6;i++) {
		if (i<3) part=1; else part=-1;
		map->CalculateGroundCoordinates((float)feet[i].getElement(1,4), (float)feet[i].getElement(2,4),r);
		if (!selectFoothold(*body, r[0], r[1], i, pos))
			return false;
		map->CalculateGlobalCoordinates(pos[0],pos[1],pos);
		feet[i].createTRMatrix(0,0,0,pos[0],pos[1],pos[2]);
	}
	return true;
}

/// draw tree
void Crrt::drawTree(double red, double green, double blue, double thickness, const char marker){
	glColor3f(red, green, blue);
	int index;
	for (int iter=1; iter<(int)tree.size();iter++) {
		index = tree[iter].GetParentIndex();
		if (index!=-1)
		{
			if (tree[iter].gait_type==0)
				glColor3f(1.0,1.0,1.0);
			else
				glColor3f(red, green, blue);
			glPushMatrix();
			glLineWidth(thickness);
			glBegin(GL_LINES);
			glVertex3f(tree[iter].body.getElement(1,4)*10, tree[iter].body.getElement(3,4)*10, -tree[iter].body.getElement(2,4)*10);
				glVertex3f(tree[index].body.getElement(1,4)*10, tree[index].body.getElement(3,4)*10, -tree[index].body.getElement(2,4)*10);
			glEnd();
			glPopMatrix();
		}
		tree[iter].drawNode(red, green, blue, thickness, marker);
	}
/*
	std::vector< void* > path;
	float totalCost;
	int startState[2], endState[2];
	map->CalculateGroundCoordinates(2.3, -2.3, startState);
	map->CalculateGroundCoordinates(-2.3, 2.3, endState);

	int result = pather->Solve( XYToNode( startState[0], startState[1] ), XYToNode( endState[0], endState[1] ), &path, &totalCost );
	saveAstar2file(&path, "astarpath.m");
	pather->Reset();
*/
	// do something with the path
/*
	float x=-2.8;
	float y=-2.8;
	float distance;
	float variance=0;
	float kinematic;
	float pos[2];
	float init_pos[6]={0,0,0,0,0,0};
	float dest_pos[6]={0,0,0,0,0,0};
	float neutral_height, height;
	int pos_node[2];
	FILE * plik;
map->CalculateGroundCoordinates(-2.8,2.8,pos_node);
	fopen_s(&plik, "variance_coef.m","w+t");

	for (int i=0;i<57;i++){
		for (int j=0;j<57;j++){
	map->CalculateGlobalCoordinates(pos_node[0]+i*10,pos_node[1]+j*10, pos);
	neutral_height = map->getMaxSquareHeight(pos_node[0]+i*10, pos_node[1]+j*10, 4);
	init_pos[0]=pos[0]; init_pos[1]=pos[1]; init_pos[2]=neutral_height+0.075;
			for (int k=-1;k<=1;k++){
				for (int l=-1;l<=1;l++){
					if (!((k==0)&&(l==0))){
						height = map->getMaxSquareHeight(pos_node[0]+i*10+k*10,pos_node[1]+j*10+l*10, 4);
						variance = map->ComputeSphericalVariance(4, pos_node[0]+i*10, pos_node[1]+j*10);
						distance = sqrt(pow(k*0.1,2)+pow(l*0.1,2)+pow(height-neutral_height,2))/0.25;
						if (distance>1) distance=1;
						dest_pos[0]=pos[0]+k*0.1; dest_pos[1]=pos[1]+l*0.1; dest_pos[2]=height+0.05;
						if (checkTraverse(init_pos, dest_pos))
							kinematic = 0;
						else{
							init_pos[2]+=0.05;
							dest_pos[2]+=0.05;
							if (checkTraverse(init_pos, dest_pos))
								kinematic = 0;
							else {
								init_pos[2]+=0.06;
								dest_pos[2]+=0.06;
								if (checkTraverse(init_pos, dest_pos))
									kinematic = 0;
								else
									kinematic = 1;
							}
						}
						//glColor3f((distance+variance)/2,(distance+variance)/2,(distance+variance)/2);

						if (l>0){
							fprintf(plik,"x1_coef=[%f %f]; ",x+i*0.1-0.01, x+i*0.1+k*0.1-0.01);
							fprintf(plik,"y1_coef=[%f %f]; ",y+j*0.1, y+j*0.1+l*0.1);
							fprintf(plik,"z1_coef=[%f %f]; ",neutral_height+0.05, height+0.05);
						}
						if (l<0){
							fprintf(plik,"x1_coef=[%f %f]; ",x+i*0.1+0.01, x+i*0.1+k*0.1+0.01);
							fprintf(plik,"y1_coef=[%f %f]; ",y+j*0.1, y+j*0.1+l*0.1);
							fprintf(plik,"z1_coef=[%f %f]; ",neutral_height+0.05, height+0.05);
						}
						if ((l==0)&&(k==1)){
							fprintf(plik,"x1_coef=[%f %f]; ",x+i*0.1, x+i*0.1+k*0.1);
							fprintf(plik,"y1_coef=[%f %f]; ",y+j*0.1-0.01, y+j*0.1+l*0.1-0.01);
							fprintf(plik,"z1_coef=[%f %f]; ",neutral_height+0.05, height+0.05);
						}
						if ((l==0)&&(k==-1)){
							fprintf(plik,"x1_coef=[%f %f]; ",x+i*0.1, x+i*0.1+k*0.1);
							fprintf(plik,"y1_coef=[%f %f]; ",y+j*0.1+0.01, y+j*0.1+l*0.1+0.01);
							fprintf(plik,"z1_coef=[%f %f]; ",neutral_height+0.05, height+0.05);
						}
						float all=(variance+distance)/2;
						if (kinematic) all=1;
						fprintf(plik,"plot3(y1_coef*100,x1_coef*100,z1_coef*100,'Color',[%f %f %f]);\n",variance*2,variance*2,variance*2);

						glColor3f(kinematic, kinematic, kinematic);
						glPushMatrix();
						glLineWidth(thickness);
						glBegin(GL_LINES);
							glVertex3f((x+i*0.1+k*0.1)*10, height*10, -(y+j*0.1+l*0.1)*10);
							glVertex3f((x+i*0.1)*10, neutral_height*10, -(y+j*0.1)*10);
						glEnd();
						glPopMatrix();
					}
				}
			}
		}
	}
	fclose(plik);
*/
}

/// save A* to file
void Crrt::saveAstar2file(std::vector< void* > *path, const char * filename){
	int x, y;
	float pos[2];
	FILE * plik;
	if (path->size()>0){
		fopen_s(&plik, filename,"w+t");
		NodeToXY( (*path)[0],&x,&y);
		map->CalculateGlobalCoordinates(x,y, pos);
		fprintf(plik,"x1(1)=%f; ",pos[0]);
		fprintf(plik,"y1(1)=%f; ",pos[1]);
		fprintf(plik,"z1(1)=%f; \n",0.42);
			
		for (int iter=1; iter<(int)path->size();iter++) {
			NodeToXY( (*path)[iter],&x,&y);
			map->CalculateGlobalCoordinates(x,y, pos);
				fprintf(plik,"x1(%d)=%f; ",iter+1,pos[0]);
				fprintf(plik,"y1(%d)=%f; ",iter+1,pos[1]);
				fprintf(plik,"z1(%d)=%f; \n",iter+1,0.42);
		}
		fprintf(plik,"plot3(-y1*100,x1*100,z1*100,'g');\n");
		glEnd();
		glPopMatrix();
		fclose(plik);
	}
}

///finds path using a-star algorithm
int Crrt::astarFind(std::vector< void* > *path, float * init_pos, float * dest_pos){
	float totalCost;
	int startState[2], endState[2];
	for (int i=0;i<2;i++){
		if (init_pos[i]>0)	init_pos[i]=float(int((init_pos[i]+0.05)*10))/10; 
		else init_pos[i]=float(int((init_pos[i]-0.05)*10))/10; 
		if (dest_pos[i]>0)	dest_pos[i]=float(int((dest_pos[i]+0.05)*10))/10; 
		else dest_pos[i]=float(int((dest_pos[i]-0.05)*10))/10; 
	}
	map->CalculateGroundCoordinates(init_pos[0], init_pos[1], startState);
	map->CalculateGroundCoordinates(dest_pos[0], dest_pos[1], endState);
	pather->Reset();
	int result = pather->Solve( XYToNode( startState[0], startState[1] ), XYToNode( endState[0], endState[1] ), path, &totalCost );
	pather->Reset();

	return result;
}

void Crrt::NodeToXY( void* node, int* x, int* y ) 
{
	int index = (int)node;
	*y = index / map->getXnumPoints();
	*x = index - *y * map->getXnumPoints();
}

void* Crrt::XYToNode( int x, int y )
{
	return (void*) ( y*map->getXnumPoints() + x );
}

float Crrt::LeastCostEstimate( void* nodeStart, void* nodeEnd ) 
{
	int start_pos[2], end_pos[2];
	NodeToXY( nodeStart, start_pos, &start_pos[1] ); 
	NodeToXY( nodeEnd, end_pos, &end_pos[1] );
	/* Compute the minimum path cost using distance measurement. It is possible
	   to compute the exact minimum path using the fact that you can move only 
	   on a straight line or on a diagonal, and this will yield a better result.
	*/

//	float dx = map->points[start_pos[0]][start_pos[1]][0] - map->points[end_pos[0]][end_pos[1]][0];
//	float dy = map->points[start_pos[0]][start_pos[1]][1] - map->points[end_pos[0]][end_pos[1]][1];
	
	float dx = (map->getXsize()/(map->getXnumPoints()-1))*start_pos[0]-(map->getXsize()/2) - (map->getXsize()/(map->getXnumPoints()-1))*end_pos[0]-(map->getXsize()/2);
	float dy = (map->getYsize()/(map->getYnumPoints()-1))*start_pos[1]-(map->getYsize()/2) - (map->getYsize()/(map->getYnumPoints()-1))*end_pos[1]-(map->getYsize()/2);
	
//	points[i][j][0]=(mesh_x_length/(mesh_x_size-1))*i-(mesh_x_length/2);// wspolrzedna x
//	points[i][j][1]=(mesh_y_length/(mesh_y_size-1))*j-(mesh_y_length/2);// wspolrzedna y
	
	return (float) sqrt( (double)(dx*dx) + (double)(dy*dy) );
}

void Crrt::AdjacentCost( void* node, std::vector< StateCost > *neighbors ) 
{
	float distance;
	float variance=0;
	float kinematic;
	float pos[2];
	float neutral_height, height;
	float init_pos[6]={0,0,0,0,0,0};
	float dest_pos[6]={0,0,0,0,0,0};
	const int dx[8] = { 1, 1, 0, -1, -1, -1,  0,  1 };
	const int dy[8] = { 0, 1, 1,  1,  0, -1, -1, -1 };
	float cost;
	int pos_node[2];
	NodeToXY( node, pos_node, &pos_node[1] ); 
	map->CalculateGlobalCoordinates(pos_node[0],pos_node[1], pos);
	neutral_height = map->getMaxSquareHeight(pos_node[0], pos_node[1], 4);
	init_pos[0]=pos[0]; init_pos[1]=pos[1]; init_pos[2]=neutral_height+0.075;
	//iter_adj++;
	for( int i=0; i<8; ++i ) {
		height = map->getMaxSquareHeight(pos_node[0]+dx[i]*10,pos_node[1]+dy[i]*10, 4);
		variance = map->ComputeSphericalVariance(4, pos_node[0], pos_node[1]);
		distance = sqrt(pow(dx[i]*0.1,2)+pow(dy[i]*0.1,2)+pow(height-neutral_height,2));
		if (distance>1) distance=1;
		dest_pos[0]=pos[0]+dx[i]*0.1; dest_pos[1]=pos[1]+dy[i]*0.1; dest_pos[2]=height+0.075;
		if (abs(dest_pos[2]-neutral_height)>0.02){
			if (checkTraverse(init_pos, dest_pos))
				kinematic = 0;
			else{
				init_pos[2]+=0.05;
				dest_pos[2]+=0.05;
				if (checkTraverse(init_pos, dest_pos))
					kinematic = 0;
				else {
					init_pos[2]+=0.06;
					dest_pos[2]+=0.06;
					if (checkTraverse(init_pos, dest_pos))
						kinematic = 0;
					else
						kinematic = 1;
				}
			}
		}
		cost = (distance+variance)/2;
		//cost = distance;
		//if ( distance > 0.33||kinematic==1 ) {
		if ( kinematic==1 ) {
			//noway_no++;
			StateCost nodeCost = { XYToNode( pos_node[0]+dx[i]*10, pos_node[1]+dy[i]*10 ), FLT_MAX };
			neighbors->push_back( nodeCost );
		}
		else {
			StateCost nodeCost = { XYToNode( pos_node[0]+dx[i]*10, pos_node[1]+dy[i]*10 ), cost };
			neighbors->push_back( nodeCost );
		}
	}
}

void Crrt::PrintStateInfo( void* node ) 
{
	/*int x, y;
	NodeToXY( node, &x, &y );
	printf( "(%d,%d)", x, y );*/
}

/// save to file
void Crrt::save2file(const char * filename){
	FILE * plik;
	int index;
	fopen_s(&plik, filename,"w+t");
	for (int iter=1; iter<(int)tree.size();iter++) {
		index = tree[iter].GetParentIndex();
		if (index!=-1)
		{
			fprintf(plik,"x=[%f, %f];",tree[iter].body.getElement(1,4)*10, tree[index].body.getElement(1,4)*10);
			fprintf(plik,"y=[%f, %f];",tree[iter].body.getElement(3,4)*10, tree[index].body.getElement(3,4)*10);
			fprintf(plik,"z=[%f, %f];",-tree[iter].body.getElement(2,4)*10, -tree[index].body.getElement(2,4)*10);
		}
		if (tree[iter].gait_type==0)
			fprintf(plik,"plot3(z*10,x*10,y*10,'k','LineWidth',4);");
		else
			fprintf(plik,"plot3(z*10,x*10,y*10,'g','LineWidth',4);");
		fprintf(plik,"hold on;");
		fprintf(plik,"plot3(z*10,x*10,y*10,'.b','MarkerSize',4);\n");
	}

	fclose(plik);
}