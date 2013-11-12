#include "motion_planner.h"

CMotionPlanner::CMotionPlanner(CIdealMap* map, RPCCaller* rpccaller)
{
	this->rpccaller=rpccaller;
	this->robot=new CRobot(rpccaller);
	robot->initializeRealRobot();
	initializeRand();
	rrt_begin=new Crrt(map,rpccaller,robot);
	rrt_finish=new Crrt(map,rpccaller,robot);
}

CMotionPlanner::~CMotionPlanner()
{
	delete rrt_begin;
	delete rrt_finish;
	delete robot;
}

/// save path to file
void CMotionPlanner::savePath2File(const char * filename){
	ofstream f_path (filename);
	CPunctum body, foot;
	if (f_path.is_open()) {
		size_t size=robot_platform_traj.getSize()-1;
		robot_platform_traj.getFirst(&body);
		body.export2file(&f_path);
		for (int i=0;i<6;i++){
			legs_traj[i].getFirst(&foot);
			foot.export2file(&f_path);
		}
		f_path << "\n";
		for (size_t i=0;i<size;i++){
			robot_platform_traj.getNext(&body);
			body.export2file(&f_path);
			for (int i=0;i<6;i++){
				legs_traj[i].getNext(&foot);
				foot.export2file(&f_path);
			}
			f_path << "\n";
		}
		f_path.close();
	}
	else cout << "Unable to open file";
}

/// load path from file
void CMotionPlanner::loadPathFromFile(const char * filename){
  ifstream f_path (filename);
  if (f_path.is_open())  {
	  robot_platform_traj.clear();
	  for (int i=0;i<6;i++)
		  legs_traj[i].clear();
	// parse text file
    const char* DELIM = " \t,;:";
	const int PUNCTUM_SIZE = 17; //4x4=16 floats for homogenous matrix + foothold
	const int NUM_POSES = 7;//body+6legs
	vector <float> pose_tab (PUNCTUM_SIZE, 0);
    for (string line; !f_path.eof() && getline(f_path, line); )
        if (!line.empty()&&(line[0]!='#')) {
            char *token = std::strtok(const_cast<char*>(line.c_str()), DELIM);
            for (size_t index = 0; token != NULL && index < PUNCTUM_SIZE*NUM_POSES; token = std::strtok(NULL, DELIM), ++index){
				pose_tab[index % PUNCTUM_SIZE] = float(atof(token));
				if (index % PUNCTUM_SIZE == PUNCTUM_SIZE-1) {
					CPunctum pose(&pose_tab[0]);
					if (int (index/PUNCTUM_SIZE)==0)
						robot_platform_traj.savePunctum(pose);
					else
						legs_traj[index/PUNCTUM_SIZE-1].savePunctum(pose);
				}
			}
       //     trajectory.push_back(config);
       } 
    f_path.close();
  }
  else cout << "Unable to open file"; 
}

//execute trajectory
bool CMotionPlanner::executeTrajectory(float speed){
	CPunctum body, body_prev;
	CPunctum feet[6],feet_prev[6],feet_temp[6];
	char prev_foothold=0;
	for (int i=0;i<6;i++)
		legs_traj[i].getFirst(&feet_prev[i]);
	robot_platform_traj.getFirst(&body_prev);
	float dpos[6];
	CPunctum robot_pos;
	CPunctum body_offset; body_offset.setIdentity();
	CPunctum foot_offset[6];
	for (int i=0; i<6;i++) foot_offset[i].setIdentity();

	size_t size=robot_platform_traj.getSize()-1;
	for (size_t i=0;i<size;i++){
		robot_platform_traj.getNext(&body);
		for (int i=0;i<6;i++)
			legs_traj[i].getNext(&feet[i]);
		if ((feet[0].isFoothold())&&(prev_foothold==1)) {//jezeli nogi nieparzyste maja byc opuszczane
			prev_foothold=0;
			/*robot->sleepODE(100);
			robot_pos = robot->getRobotState();
			dpos[0] = body.getElement(1,4)-robot_pos.getElement(1,4);
			dpos[1] = body.getElement(2,4)-robot_pos.getElement(2,4);
			dpos[2] = body.getElement(3,4)-robot_pos.getElement(3,4);
			robot->getRotAngles(&dpos[3]);
			dpos[3] = body.orientation[0]-dpos[3];
			dpos[4] = body.orientation[1]-dpos[4];
			dpos[5] = body.orientation[2]-dpos[5];
			CPunctum delta = robot_pos.inv(robot_pos)*body;
			//if (!robot->changePlatformRobotTripod(dpos[0],dpos[1],dpos[2],dpos[3],dpos[4],dpos[5],0,speed,1))
			if (!robot->changePlatformRobotTripod(delta.getElement(1,4),delta.getElement(2,4),delta.getElement(3,4),dpos[3],dpos[4],dpos[5],1,speed,1))
				return false;*/
			if (!robot->Placefeet(0.005, 0, speed))
				return false;
			//robot->getFullRobotState(&body, feet);
		}
		else if ((feet[1].isFoothold())&&(prev_foothold==0)){//jezeli nogi parzyste maja byc opuszczane
			prev_foothold=1;
			/*robot->sleepODE(100);
			robot_pos = robot->getRobotState();
			dpos[0] = body.getElement(1,4)-robot_pos.getElement(1,4);
			dpos[1] = body.getElement(2,4)-robot_pos.getElement(2,4);
			dpos[2] = body.getElement(3,4)-robot_pos.getElement(3,4);
			robot->getRotAngles(&dpos[3]);
			dpos[3] = body.orientation[0]-dpos[3];
			dpos[4] = body.orientation[1]-dpos[4];
			dpos[5] = body.orientation[2]-dpos[5];
			CPunctum delta = robot_pos.inv(robot_pos)*body;
			//if (!robot->changePlatformRobotTripod(dpos[0],dpos[1],dpos[2],dpos[3],dpos[4],dpos[5],0,speed,1))
			if (!robot->changePlatformRobotTripod(delta.getElement(1,4),delta.getElement(2,4),delta.getElement(3,4),dpos[3],dpos[4],dpos[5],0,speed,1))
				return false;*/
			if (!robot->Placefeet(0.005, 1, speed))
				return false;
//			robot->getFullRobotState(&body, feet);
		}
		else {// w przeciwnym wypadku idz do kolejnego punktu trajektorii
			//// robot->dynamicWorld->body=body;
			//body_prev = robot->getRobotState();
			CPunctum body_real = robot->getRobotState();
			CPunctum delta;
			delta = body_real.inv()*body_prev;
			CPunctum delta1;
			delta1 = body_prev.inv()*body_real;
		/*	for (int j=0;j<6;j++){
				if (!feet[j].isFoothold())
					feet[j]=feet[j]*delta;
			}
			//	// robot->dynamicWorld->feet[j]=feet[j];
			
			body = body*delta;*/
			if (!robot->move2GlobalPosition(body_real,body,feet_prev,feet,speed,1))
				return false;
		}
		body_prev=body;
		for (int i=0;i<6;i++)
			feet_prev[i]=feet[i];
	}
	return true;
}


//robot porusza sie po trajektorii zadanej
bool CMotionPlanner::executeTrajectoryWave(float speed){
	CPunctum body, body_prev;
	CPunctum feet[6],feet_prev[6];
	char prev_foothold[6]={1,1,1,1,1,1};
	for (int i=0;i<6;i++)
		legs_traj[i].getFirst(&feet_prev[i]);
	robot_platform_traj.getFirst(&body_prev);
	float dpos[6];
	CPunctum robot_pos;
	CPunctum body_offset; body_offset.setIdentity();
	CPunctum foot_offset[6];
	for (int i=0; i<6;i++) foot_offset[i].setIdentity();

	while (robot_platform_traj.getNext(&body)){
		for (int i=0;i<6;i++)
			legs_traj[i].getNext(&feet[i]);
		if (((feet[0].isFoothold())&&(prev_foothold[0]==0))||((feet[1].isFoothold())&&(prev_foothold[1]==0))||((feet[2].isFoothold())&&(prev_foothold[2]==0))||((feet[3].isFoothold())&&(prev_foothold[3]==0))||((feet[4].isFoothold())&&(prev_foothold[4]==0))||((feet[5].isFoothold())&&(prev_foothold[5]==0))) {//jezeli nogi nieparzyste maja byc opuszczane
			robot->sleepODE(100);
/*			robot_pos = robot->getRobotState();
			dpos[0] = body.getElement(1,4)-robot_pos.getElement(1,4);
			dpos[1] = body.getElement(2,4)-robot_pos.getElement(2,4);
			dpos[2] = body.getElement(3,4)-robot_pos.getElement(3,4);
			robot->getRotAngles(&dpos[3]);
			dpos[3] = body.orientation[0]-dpos[3];
			dpos[4] = body.orientation[1]-dpos[4];
			dpos[5] = body.orientation[2]-dpos[5];
			CPunctum delta = robot_pos.inv(robot_pos)*body;
			if (!robot->changePlatformRobotSense(delta.getElement(1,4),delta.getElement(2,4),delta.getElement(3,4),dpos[3],dpos[4],dpos[5],speed,1))
				return false;*/
			if (!robot->Placefeet(0.005, speed))
				return false;
			for (int i=0;i<6;i++) if (feet[i].isFoothold()) prev_foothold[i]=1; else prev_foothold[i]=0;
		}
		else {// w przeciwnym wypadku idz do kolejnego punktu trajektorii
			CPunctum body_real = robot->getRobotState();
			CPunctum delta;
			delta = body_real.inv()*body_prev;
			CPunctum delta1;
			delta1 = body_prev.inv()*body_real;
		/*	for (int j=0;j<6;j++){
				if (!feet[j].isFoothold())
					feet[j]=feet[j]*delta;
			}
			// robot->dynamicWorld->feet[j]=feet[j];
			
			body = body*delta;*/
			if (!robot->move2GlobalPosition(body_real,body,feet_prev,feet,speed,1))
				return false;
			for (int i=0;i<6;i++) if (feet[i].isFoothold()) prev_foothold[i]=1; else prev_foothold[i]=0;
		}
		body_prev=body;
		for (int i=0;i<6;i++)
			feet_prev[i]=feet[i];
	}
	return true;
}

//wyciecie z zaplanowanej sciezki fragmentow wykraczajacych poza zadana odleglosc
bool CMotionPlanner::reduceTrajectory(float distance){
	CPunctum body, body_prev, feet[6];
	robot_platform_traj.getFirst(&body_prev);
	for (int i=0;i<6;i++)
		legs_traj[i].getFirst(&feet[i]);
	int iter=0,num;
	bool rest;
	while (robot_platform_traj.getNext(&body)){
		rest=true;
		for (int i=0;i<6;i++){
			legs_traj[i].getNext(&feet[i]);
			if (!feet[i].isFoothold())
				rest=false;
		}
		if (rest)
			num=iter;
		if ((rest)&&(sqrt(pow(body.getElement(1,4)-body_prev.getElement(1,4),2)+pow(body.getElement(2,4)-body_prev.getElement(2,4),2))>distance)){
			num = iter;
			break;
		}
		if (iter>robot_platform_traj.getSize())
			break;
		iter++;
	}
	robot_platform_traj.eraseLast(robot_platform_traj.getSize()-num-2);
	for (int i=0;i<6;i++)
		legs_traj[i].eraseLast(legs_traj[i].getSize()-num-2);
	return true;
}

/// connect operation
int CMotionPlanner::Connect(Crrt * tree, CrrtNode * q_new){

	return 1;
}

/// rrt connect
bool CMotionPlanner::rrtConnect(float pos_end[], float rot_end[], float distance2ground, float *map_boundaries, int shift){
	rrt_begin->erase();
	rrt_finish->erase();
	float angles[3];
	float position[3];
	CPunctum start_pos = robot->getRobotState();
	for (int i=0;i<3;i++)
		position[i]=start_pos.getElement(i+1,4);
	robot->getRotAngles(angles);
	CPunctum body; CPunctum feet[6];
	robot->getFullRobotState(&body, feet);
	rrt_begin->setRoot(position, angles, body, feet);

	rrt_finish->computeOrientationAndHeight(pos_end[0],pos_end[1],&pos_end[2],rot_end, 0.1);//oblicza orientacje i wysokosc platformy na koncu ruchu na podstawie uksztaltowania ternu
	body.createTRMatrix(rot_end[0], rot_end[1], rot_end[2], pos_end[0], pos_end[1], pos_end[2]);
	rrt_finish->createRobotStateFoothold(pos_end, rot_end, &body, feet);
	rrt_finish->setRoot(pos_end, rot_end, body, feet);
	initializeRand();

	CrrtNode q_rand, q_new;
	for (int k=0;k<48000;k++){
		q_rand.randomConfig(map_boundaries[0], map_boundaries[1], map_boundaries[2], map_boundaries[3]);
		//Sleep(5000);
		if (rrt_begin->ExtendOpt(q_rand, &q_new,false,distance2ground,shift,false,position)!=0){
		//	Sleep(5000);
			if (rrt_finish->ExtendOpt(q_new, &q_rand,true,distance2ground,shift,true,position)==1){
				createPath(q_new, q_rand,shift);
				pathSmoothing();
 				return true;
			}
		}
		q_rand.randomConfig(map_boundaries[0], map_boundaries[1], map_boundaries[2], map_boundaries[3]);
		//Sleep(5000);
		if (rrt_finish->ExtendOpt(q_rand, &q_new,true,distance2ground,shift,false,position)!=0){
			//Sleep(5000);
			if (rrt_begin->ExtendOpt(q_new, &q_rand,false,distance2ground,shift,true,position)==1){
				createPath(q_rand, q_new,shift);
				pathSmoothing();
				return true;
			}
		}
		if (((k>10)&&(rrt_finish->GetNodeNo()<3))||(k>500))
			return false;
	}
	return false;
}

/// create path according to obtained tree
void CMotionPlanner::createPath(CrrtNode q_begin, CrrtNode q_finish, int shift){
	robot_platform_traj.setDelay(0);
	robot_platform_traj.clear();
	for (int j=0;j<6;j++)
		legs_traj[j].clear();

	//robot_platform_traj.savePositionAndOrientation(q_begin.pos[0],q_begin.pos[1],q_begin.pos[2],q_begin.rot[0],q_begin.rot[1],q_begin.rot[2]);
	CrrtNode q = q_begin;
	while (q.GetParentIndex()!=-1) {
		//robot_platform_traj.savePositionAndOrientation(q.pos[0],q.pos[1],q.pos[2],q.rot[0],q.rot[1],q.rot[2]);
		if (q.GetParentIndex()!=-1){
			for (int i=shift-1;i>-1;i--){
				for (int j=0;j<6;j++)
					legs_traj[j].savePunctum(q.legs_traj[j][i]);
				robot_platform_traj.savePunctum(q.body_traj[i]);
			}
//			robot_platform_traj.savePunctum(q.body);
		}
		q = rrt_begin->getNode(q.GetParentIndex());
	}
	robot_platform_traj.reverseOrder();
	for (int j=0;j<6;j++)
		legs_traj[j].reverseOrder();
	q = q_finish;
	for (int j=0;j<6;j++)
		legs_traj[j].savePunctum(q_begin.feet[j]);
	robot_platform_traj.savePositionAndOrientation(q_finish.pos[0],q_finish.pos[1],q_finish.pos[2],q_finish.rot[0],q_finish.rot[1],q_finish.rot[2]);
	while (q.GetParentIndex()!=-1) {
		//robot_platform_traj.savePositionAndOrientation(q.pos[0],q.pos[1],q.pos[2],q.rot[0],q.rot[1],q.rot[2]);
		if ((q.GetParentIndex()!=-1)){
			for (int i=0;i<shift;i++){
				for (int j=0;j<6;j++)
					legs_traj[j].savePunctum(q.legs_traj[j][i]);
				robot_platform_traj.savePunctum(q.body_traj[i]);
			}
//			robot_platform_traj.savePunctum(q.body);
		}
		q = rrt_finish->getNode(q.GetParentIndex());
	}
}

/// path smoothing
void CMotionPlanner::pathSmoothing(	float alpha, float beta, float alpha_leg, float beta_leg){
	CPunctum point, f_point[6];
	CPositionRecorder new_robot_platform_traj;
	CPositionRecorder new_feet[6];

	robot_platform_traj.getFirst(&point);//copy old path to the new path
	new_robot_platform_traj.savePunctum(point);
	for (int j=0;j<6;j++){
		legs_traj[j].getFirst(&f_point[j]);
		new_feet[j].savePunctum(f_point[j]);
	}
	int list_size = robot_platform_traj.getSize();
	if (list_size>0){
		for (int i=1;i<list_size;i++){
			robot_platform_traj.getNext(&point);
			new_robot_platform_traj.savePunctum(point);
			for (int j=0;j<6;j++){
				if (legs_traj[j].getNext(&point))
					new_feet[j].savePunctum(point);
			}
		}
	}

	float change=10;
	float tolerance =0.001;
	CPunctum new_point, new_prev_point, new_next_point;
	CPunctum f_new_point[6], f_new_prev_point[6], f_new_next_point[6];
	CPunctum prev_point, next_point;
	CPunctum f_prev_point[6], f_next_point[6];
	float vect[6];

	bool * can_modify = new bool[list_size];
	for (int i=0;i<list_size;i++) can_modify[i]=true;

	if (list_size>2){//modyfication of the path
		while (change>tolerance){
			change=0;
	//		for (int j=0;j<6;j++)
//				legs_traj[j].setBegin();

			robot_platform_traj.getFirst(&prev_point);
			new_robot_platform_traj.getFirst(&new_prev_point);
			robot_platform_traj.getNext(&point);
			new_robot_platform_traj.getNext(&new_point);
			for (int k=0;k<6;k++){
				legs_traj[k].getFirst(&f_prev_point[k]);
				new_feet[k].getFirst(&f_new_prev_point[k]);
				legs_traj[k].getNext(&f_point[k]);
				new_feet[k].getNext(&f_new_point[k]);
			}
			for (int i=2;i<list_size;i++){
				robot_platform_traj.getNext(&next_point);
				new_robot_platform_traj.getNext(&new_next_point);
				for (int j=0;j<3;j++){
					float ch1 = alpha*(point.getElement(j+1,4)-new_point.getElement(j+1,4))+beta*(new_next_point.getElement(j+1,4)+new_prev_point.getElement(j+1,4)-2*new_point.getElement(j+1,4));
					vect[j] = new_point.getElement(j+1,4)+ch1;
					float ch2 = alpha*(point.orientation[j]-new_point.orientation[j])+beta*(new_next_point.orientation[j]+new_prev_point.orientation[j]-2*new_point.orientation[j]);
					vect[j+3] = new_point.orientation[j]+ch2;
					change += abs(new_point.getElement(j+1,4)-vect[j]);
					change += abs(new_point.orientation[j]-vect[j+3]);
				}

				new_point.createTRMatrix(vect[3],vect[4],vect[5],vect[0],vect[1],vect[2]);
				for (int j=0;j<3;j++) new_point.orientation[j]=vect[j+3];

				for (int k=0;k<6;k++){
					legs_traj[k].getNext(&f_next_point[k]);
					new_feet[k].getNext(&f_new_next_point[k]);
					if (!f_point[k].isFoothold()){
						for (int j=0;j<3;j++){
							float ch1 = alpha_leg*(f_point[k].getElement(j+1,4)-f_new_point[k].getElement(j+1,4))+beta_leg*(f_new_next_point[k].getElement(j+1,4)+f_new_prev_point[k].getElement(j+1,4)-2*f_new_point[k].getElement(j+1,4));
							vect[j] = f_new_point[k].getElement(j+1,4)+ch1;
						}
						f_new_point[k].createTRMatrix(0,0,0,vect[0],vect[1],vect[2]);
						if (f_point[k].isFoothold())
							f_new_point[k].setFoothold(true);
					}
				}

				if (1){
					new_robot_platform_traj.updatePreviousElement(&new_point);
					for (int k=0;k<6;k++){
						if (this->verifyAchievability(new_point,f_new_point[k],k)){
							new_feet[k].updatePreviousElement(&f_new_point[k]);
						}
					}
				}
									/*
					if ((rrt_begin->robot->computeKinematicMargin(new_point,f_new_point)>0.05)){//&&(rrt_begin->robot->computeStabilityMargin(new_point,f_new_point)>0.05)){
						new_robot_platform_traj.updatePreviousElement(&new_point);
						for (int k=0;k<6;k++)
							new_feet[k].updatePreviousElement(&f_new_point[k]);
					}
					else
						can_modify[i]=false;*/
				new_prev_point=new_point;
				new_point=new_next_point;
				prev_point=point;
				point=next_point;
				for (int k=0;k<6;k++){
					f_new_prev_point[k]=f_new_point[k];
					f_new_point[k]=f_new_next_point[k];
					f_prev_point[k]=f_point[k];
					f_point[k]=f_next_point[k];
				}
			}
		}

		robot_platform_traj.clear();//copy new path to the old path
		for (int j=0;j<6;j++)
			legs_traj[j].clear();
		if (new_robot_platform_traj.getSize()>0){
			new_robot_platform_traj.getFirst(&point);
			robot_platform_traj.savePunctum(point);
			for (int j=0;j<6;j++){
				new_feet[j].getFirst(&f_point[j]);
				legs_traj[j].savePunctum(f_point[j]);
			}
			for (int j=1;j<list_size;j++){
				new_robot_platform_traj.getNext(&point);
				robot_platform_traj.savePunctum(point);
				for (int k=0;k<6;k++){
					new_feet[k].getNext(&f_point[k]);
					legs_traj[k].savePunctum(f_point[k]);
				}
			}
		}
	}
}

/// create path according to obtained tree
void CMotionPlanner::createPath(CrrtNode q_begin, int shift){
	robot_platform_traj.setDelay(0);
	robot_platform_traj.clear();
	for (int j=0;j<6;j++)
		legs_traj[j].clear();

	//robot_platform_traj.savePositionAndOrientation(q_begin.pos[0],q_begin.pos[1],q_begin.pos[2],q_begin.rot[0],q_begin.rot[1],q_begin.rot[2]);
	CrrtNode q = q_begin;
	while (q.GetParentIndex()!=-1) {
		//robot_platform_traj.savePositionAndOrientation(q.pos[0],q.pos[1],q.pos[2],q.rot[0],q.rot[1],q.rot[2]);
		if (q.GetParentIndex()!=-1){
			for (int i=shift-1;i>-1;i--){
				for (int j=0;j<6;j++)
					legs_traj[j].savePunctum(q.legs_traj[j][i]);
				robot_platform_traj.savePunctum(q.body_traj[i]);
			}
		}
		q = rrt_begin->getNode(q.GetParentIndex());
	}
	robot_platform_traj.reverseOrder();
	for (int j=0;j<6;j++)
		legs_traj[j].reverseOrder();
}

///finds path using a-star algorithm
int CMotionPlanner::astarFind(std::vector< void* > *path, float * init_pos, float * dest_pos){
	return rrt_begin->astarFind(path, init_pos, dest_pos);
}