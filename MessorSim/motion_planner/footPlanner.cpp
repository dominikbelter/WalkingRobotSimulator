#include "footPlanner.h"
#include "../math/vector.h"
#include "../functions.h"
#include <boost/timer.hpp> 

/*klasa realizujaca planowanie trajektorii stopy oraz wybor punktu podparcia*/


CFootPlanner::CFootPlanner(void)
{
	for (int i=0;i<6;i++)
	{
		raster_x[i]=0;
		raster_y[i]=0;
	}
	createFootQualityMatrix(21);
	
	FILE * footholds = fopen("footholds.txt","w");
	fclose(footholds);
//	if(Init())
//	  printf("Unable to establish connection to map agent\n");
//	rpccaller_map=new RPCCaller("150.254.46.21",2349);
//	rpccaller_map->Run();
}

CFootPlanner::~CFootPlanner(void)
{
	for (int i=0;i<(search_size);i++){
		delete [] K1[i];
		delete [] K2[i];
		delete [] K3[i];
		delete [] K4[i];
		delete [] K5[i];
	}
	delete [] K1;
	delete [] K2;
	delete [] K3;
	delete [] K4;
	delete [] K5;
}


// int num_points - liczba punktow mapy wycinanej wokol stopy
void CFootPlanner::createFootQualityMatrix(int num_points)
{ 
	search_size = num_points;
	K1=new float *[search_size];
	K2=new float *[search_size];
	K3=new float *[search_size];
	K4=new float *[search_size];
	K5=new float *[search_size];

	for (int i=0;i<(search_size);i++){
		K1[i]=new float[search_size];
		K2[i]=new float[search_size];
		K3[i]=new float[search_size];
		K4[i]=new float[search_size];
		K5[i]=new float[search_size];
	}

	for (int i=0;i<6;i++)
	{
		raster_x[i]=0;
		raster_y[i]=0;
	}
}

/// oblicz wspolczynniki jakosci
void CFootPlanner::ComputeKcoef(int size, int x, int y){

	float neutral_height;
	neutral_height = map->getHeight(x, y); //wysokosc punktu domyslnego

	float height;
	size=size+4;
	/*for (int i=-size;i<=size;i++){
		for (int j=-size;j<=size;j++) {
			//K1[i+size][j+size]=sqrt(pow(i*scale_factor_x,float(2.0))+pow(j*scale_factor_y,float(2.0))+pow(neutral_height-dynamicWorld.ground->getHeight(x+i,y+j),2.0));
			K1[i+size][j+size]=sqrt(pow(i*scale_factor_x,float(2.0))+pow(j*scale_factor_y,float(2.0)));
			//foot.setElement(foot_pos[0]+i*scale_factor_x,1,4); foot.setElement(foot_pos[1]+j*scale_factor_y,2,4);
			//foot.setElement(map->getHeight(x+i, y+j),1,4);
			//K1[i+size][j+size] = 0.2-robot.computeKinematicMarginApprox(&robot_pos,&foot,legnumber);
		}
	}*/

	for (int i=-size;i<=size;i++) {
		for (int j=-size;j<=size;j++) {
			K2[i+size][j+size]=0;
			neutral_height = map->getHeight(x+i,y+j);
			for (int n=-1;n<=1;n++) {
				for (int m=-1;m<=1;m++) {
					height = map->getHeight(x+i+n,y+j+m);
					K2[i+size][j+size]+=neutral_height-height;
				}
			}
		}
	}

	for (int i=-size;i<=size;i++) {
		for (int j=-size;j<=size;j++) {
			K3[i+size][j+size]=0;
			neutral_height = map->getHeight(x+i,y+j);
			for (int n=-1;n<=1;n++) {
				for (int m=-1;m<=1;m++) {
					height = map->getHeight(x+i+n,y+j+m);
					K3[i+size][j+size]+=abs(neutral_height-height);
				}
			}
		}
	}

	for (int i=-size;i<=size;i++) {
		for (int j=-size;j<=size;j++) {
			K5[i+size][j+size]=map->getReliability(x+i,y+j);
		}
	}
}

///"madrze" stawia stope w punkcie o zadanych wspolrzednych rastrowych
bool CFootPlanner::selectFoothold(CPunctum robot_pos, int x, int y, int legnumber, float *pos){
	float foot_pos[2];
	int czy_w_zasiegu=0;

	ComputeKcoef(6,x,y);

	float K_final[21][21];
	float Kmin=10;
	bool found=false;
	float input[2];
	float calc_out=0;
	pos[0] = x;
	pos[1] = y;
	pos[2] = map->getHeight(pos[0],pos[1]);
	CPunctum f_tmp;
	f_tmp.createTRMatrix(0,0,0,0,0,0);
	bool jest_granica[2]={false, false};

	for (int i=0;i<2*(6+4);i++){
		for (int j=0;j<2*(6+4);j++){
			input[0] = K2[i][j];
			input[1] = K3[i][j];

			input[0]=(input[0]-(-0.25))/(0.25-(-0.25));//denormalization
			input[1]=(input[1]-(-0.000))/(0.25-(-0.000));
			
			calc_out=9.912273*exp(-(0.000000*pow(input[0]-(0.234143),2.0)+1.550903*pow(input[1]-(0.238928),2.0)))-0.920301*exp(-(1.299873*pow(input[0]-(0.455745),2.0)+1.392158*pow(input[1]-(0.540615),2.0)))-0.289054*exp(-(0.965865*pow(input[0]-(0.970677),2.0)+2.000479*pow(input[1]-(0.743804),2.0)))+6.670944*exp(-(1.234546*pow(input[0]-(0.513995),2.0)+0.716602*pow(input[1]-(0.338549),2.0)))+18.040905*exp(-(1.207993*pow(input[0]-(0.609503),2.0)+0.216478*pow(input[1]-(0.717137),2.0)))+5.735108*exp(-(0.031054*pow(input[0]-(1.262249),2.0)+0.000000*pow(input[1]-(0.568037),2.0)))-34.902508*exp(-(0.565552*pow(input[0]-(0.519898),2.0)+0.464450*pow(input[1]-(0.346225),2.0)))+14.137899*exp(-(0.116270*pow(input[0]-(0.188365),2.0)+0.000000*pow(input[1]-(0.280456),2.0)))+4.859023*exp(-(0.622169*pow(input[0]-(0.581143),2.0)+0.000000*pow(input[1]-(0.829203),2.0)))+1.046218*exp(-(0.391128*pow(input[0]-(0.119867),2.0)+0.959771*pow(input[1]-(0.003189),2.0)))-7.539943*exp(-(0.068036*pow(input[0]-(-0.141919),2.0)+0.000000*pow(input[1]-(0.487370),2.0)))-1.957351*exp(-(1.301417*pow(input[0]-(1.206275),2.0)+0.055104*pow(input[1]-(0.064505),2.0)))-27.713840*exp(-(0.450000*pow(input[0]-(0.587237),2.0)+0.000000*pow(input[1]-(0.806775),2.0)))-2.680697*exp(-(1.297690*pow(input[0]-(0.300009),2.0)+1.574235*pow(input[1]-(0.088284),2.0)))+15.620176*exp(-(0.000000*pow(input[0]-(0.800079),2.0)+0.000000*pow(input[1]-(0.793374),2.0)));
			map->CalculateGlobalCoordinates(x+i-10,y+j-10, foot_pos);
			
			float height;
			height = map->getHeight(x+i-10,y+j-10);
		/*	if (height<0.05)
				jest_granica[0]=true;
			if (height>0.1)
				jest_granica[1]=true;*/
			//if ((K5[i][j]>0.02)&&(abs(input[0])<1)&&(abs(input[1])<1)&&(input[0]>=0)&&(input[1]>=0)&&(calc_out<1)&&(calc_out>0)&&(robot.isFootPositionAvailableGlobal(foot_pos[0],foot_pos[1],height-0.01,legnumber))&&(robot.isFootPositionAvailableGlobal(foot_pos[0],foot_pos[1],height+0.015,legnumber))){//jezeli rozpatrywany punkt ma mniejsz� wartosc wskaznika jakosci - wybieramy go jako punkt podparcia
			//if ((abs(input[0])<1)&&(abs(input[1])<1)&&(input[0]>=0)&&(input[1]>=0)&&(calc_out<1)&&(calc_out>0)&&(robot.isFootPositionAvailableGlobal(robot_pos,foot_pos[0],foot_pos[1],height-0.01,legnumber))&&(robot.isFootPositionAvailableGlobal(robot_pos,foot_pos[0],foot_pos[1],height+0.015,legnumber))){//jezeli rozpatrywany punkt ma mniejsz� wartosc wskaznika jakosci - wybieramy go jako punkt podparcia
			f_tmp.setElement(foot_pos[0],1,4); f_tmp.setElement(foot_pos[1],2,4); f_tmp.setElement(height,3,4);
			float marg = robot->computeKinematicMarginApprox(&robot_pos,&f_tmp,legnumber);
			
			if ((abs(input[0])<1)&&(abs(input[1])<1)&&(input[0]>=0)&&(input[1]>=0)&&(calc_out<1)&&(calc_out>0)&&(marg>0.02)){//jezeli rozpatrywany punkt ma mniejsz� wartosc wskaznika jakosci - wybieramy go jako punkt podparcia
				czy_w_zasiegu=1;
			//	K_final[i][j]=calc_out+1.5*K1[i][j];
				K_final[i][j]=calc_out+(0.2-marg);
			}
			else
				K_final[i][j]=1.0;//nieosiagalny
			if ((abs(input[0])>1)||(input[0]<0)||(abs(input[1])>1)||(input[1]<0))
				K_final[i][j]=10.0;//niezbadany
			//if ((K5[i][j]>0.01)&&(Kmin>K_final[i][j])&&(abs(calc_out)<1)&&(calc_out>0)){
			if ((Kmin>K_final[i][j])&&(abs(calc_out)<0.4)&&(calc_out>0)){
				Kmin=K_final[i][j];
				pos[0] = x+i-10;
				pos[1] = y+j-10;
				pos[2] = map->getHeight(pos[0],pos[1]);
				found=true;
			}
		}
	}

	if (czy_w_zasiegu==0) 
		return false;
	if (!found) 
		return false;
	Kmin=1e32;
	float quality;
	float Kfin[21][21];
	found = false;
	for (int i=1;i<2*(6+4)-1;i++){
		for (int j=1;j<2*(6+4)-1;j++){
			quality=0;
			for (int k=-1;k<2;k++){
				for (int l=-1;l<2;l++){
					quality+=K_final[i+k][j+l];
				}
			}
			Kfin[i][j]=quality;
			if ((quality<Kmin)&&(quality<5.2)){
				found=true;
				Kmin=quality;
				pos[0] = x+i-10;
				pos[1] = y+j-10;
				pos[2] = map->getHeight(pos[0],pos[1]);
			}
		}
	}
	
	if (!found) 
		return false;

	return true;
}

/// generate foothold trajectory
void CFootPlanner::generateFootholdTrajSimple(void){
	CPunctum actual_point, prev_point, foot;
	robot_platform_traj.getFirst(&prev_point);
	int part;
	float pos[3];
	for (int i=0;i<6;i++){//pozycje poczatkowe stop
		if (i<3) part=1; else part=0;
		foot = prev_point*robot->leg[i].start*robot->leg[i].getNeutralPosition(part);
		legs_traj[i].savePosition(foot.getElement(1,4),foot.getElement(2,4),foot.getElement(3,4));
	}
	int smart_gait_iterator=0;
	int r[2];
	while (robot_platform_traj.getNext(&actual_point)) {
		for (int i=smart_gait_iterator;i<6;i+=2) {
			if (i<3) part=1; else part=0;
			foot = prev_point*robot->leg[i].start*robot->leg[i].getNeutralPosition(part);
			legs_traj[i].savePosition(foot.getElement(1,4),foot.getElement(2,4),0.1);//podnosimy konczyny
			foot = actual_point*robot->leg[i].start*robot->leg[i].getNeutralPosition(part);
			map->CalculateGroundCoordinates((float)foot.getElement(1,4), (float)foot.getElement(2,4),r);
			selectFoothold(actual_point, r[0], r[1], i, pos);
			map->CalculateGlobalCoordinates(pos[0],pos[1],pos);
			legs_traj[i].savePosition(pos[0],pos[1],0.1);//nad wybrany punkt
			legs_traj[i].savePosition(pos[0],pos[1],pos[2]);//opuszczenie stopy
		}
		smart_gait_iterator = 1;
		for (int i=smart_gait_iterator;i<6;i+=2) {//pozostale nogi w tym czasie sie nie ruszaja
			legs_traj[i].getLastPoint(&pos[0],&pos[1],&pos[2]);
			for (int j=0;j<3;j++) legs_traj[i].savePosition(pos[0],pos[1],pos[2]);
		}
		for (int i=smart_gait_iterator;i<6;i+=2) {
			if (i<3) part=1; else part=0;
			foot = prev_point*robot->leg[i].start*robot->leg[i].getNeutralPosition(part);
			legs_traj[i].savePosition(foot.getElement(1,4),foot.getElement(2,4),0.1);//podnosimy konczyny
			foot = actual_point*robot->leg[i].start*robot->leg[i].getNeutralPosition(part);
			map->CalculateGroundCoordinates((float)foot.getElement(1,4), (float)foot.getElement(2,4),r);
			selectFoothold(actual_point, r[0], r[1], i, pos);
			map->CalculateGlobalCoordinates(pos[0],pos[1],pos);
			legs_traj[i].savePosition(pos[0],pos[1],0.1);//nad wybrany punkt
			legs_traj[i].savePosition(pos[0],pos[1],pos[2]);//opuszczenie stopy
		}
		smart_gait_iterator = 0;
		for (int i=smart_gait_iterator;i<6;i+=2) {//pozostale nogi w tym czasie sie nie ruszaja
			legs_traj[i].getLastPoint(&pos[0],&pos[1],&pos[2]);
			for (int j=0;j<3;j++) legs_traj[i].savePosition(pos[0],pos[1],pos[2]);
		}
		prev_point=actual_point;
	}
}

bool CFootPlanner::generateFootTraj(CPositionRecorder *feet_traj, CPunctum foot_start, CPunctum foot_finish,CPunctum body_start, CPunctum body_finish,float distance,int leg_no, int iter_no){
	float dx = (foot_finish.getElement(1,4)-foot_start.getElement(1,4))/float(iter_no-2);
	float dy = (foot_finish.getElement(2,4)-foot_start.getElement(2,4))/float(iter_no-2);
	float pos[3];
	for (int i=0;i<3;i++) pos[i]=foot_start.getElement(i+1,4);
	int raster[2];
	float max, height;
	float * points_x; float * points_y; float * points_z;//punkty trajektorii
	points_x=new float[iter_no-1]; points_y=new float[iter_no-1]; points_z=new float[iter_no-1];
	for (int i=0;i<iter_no-2;i++){//iter_no punktow trajektorii
		max=-50;
		map->CalculateGroundCoordinates(pos[0],pos[1],raster);
		for (int j=-3;j<4;j++){//znalezienie maksymalnej wysokosci
			for (int k=-3;k<4;k++){
				height = map->getHeight(raster[0]+j,raster[1]+k);
				if (height>max) max = height;
			}
		}
		points_x[i]=pos[0]; points_y[i]=pos[1]; points_z[i]=max+distance;
		//legs_traj[leg_no].savePosition(pos[0],pos[1],max+distance);//przechodzimy nad przeszkodami
		pos[0] += dx; 
		pos[1] += dy;
	}
	for (int i=0;i<3;i++) pos[i]=foot_finish.getElement(i+1,4);
	points_x[iter_no-2]=pos[0]; points_y[iter_no-2]=pos[1]; points_z[iter_no-2]=pos[2]+distance;//punkt nad wybranym punktem
	smooth(points_x,points_y,points_z,iter_no-1);
	for (int i=0;i<iter_no-1;i++)
		feet_traj[leg_no].savePosition(points_x[i],points_y[i],points_z[i]);
	CPunctum foothold; foothold.createTRMatrix(0,0,0,pos[0],pos[1],pos[2]);
	foothold.setFoothold(true);
	feet_traj[leg_no].savePunctum(foothold);
	delete [] points_x;
	delete [] points_y;
	delete [] points_z;
	return true;
}

//wygladzanie trajektorii
void CFootPlanner::smooth(float *points_x, float *points_y, float * points_z, int size){
	float x=points_x[0];	float y=points_y[0];	float z=points_z[0];
	int no=1;//liczbapunktow w grupie
	float tangens=-1e32;
	float tangens_new;
	int base=0;
	int end=0;
	for (int i=0;i<size-1;i++){
		base=i;
		no=0;
		end=i+1;
		tangens=-3.14;
		for (int j=i+1;j<size;j++){
			tangens_new=(points_z[j]-points_z[base])/sqrt(pow(points_x[j]-points_x[base],2)+pow(points_y[j]-points_y[base],2));
			if ((tangens_new>tangens)) {
				no=j-i;
				end=j;
				tangens=tangens_new;
			}
		}
		if (no>1){
			float dx=(points_x[end]-points_x[base])/(no-1);
			float dy=(points_y[end]-points_y[base])/(no-1);
			float dz=(points_z[end]-points_z[base])/(no-1);
			for (int j=1;j<no;j++){
				points_x[base+j]=points_x[base+j-1]+dx;
				points_y[base+j]=points_y[base+j-1]+dy;
				points_z[base+j]=points_z[base+j-1]+dz;
			}
		}
		i=end-1;
	}
}

/// check traverse - simpliefied
bool CFootPlanner::checkTraverse(float * init_pos, float * dest_pos){
	CPunctum next_body, prev_body, feet_prev[6], feet_next[6];
	prev_body.createTRMatrix(init_pos[3],init_pos[4],init_pos[5],init_pos[0],init_pos[1],init_pos[2]);
	next_body.createTRMatrix(dest_pos[3],dest_pos[4],dest_pos[5],dest_pos[0],dest_pos[1],dest_pos[2]);
	int part;
	int pos_int[2];
	for (int i=0;i<6;i++){//pozycje poczatkowe stop
		if (i<3) part=1; else part=0;
		feet_prev[i] = prev_body*robot->leg[i].start*robot->leg[i].getNeutralPosition(part);
		map->CalculateGroundCoordinates(feet_prev[i].getElement(1,4),feet_prev[i].getElement(2,4),pos_int);
		feet_prev[i].setElement(map->getMaxSquareHeight(pos_int[0],pos_int[1],5),3,4);
		feet_next[i] = next_body*robot->leg[i].start*robot->leg[i].getNeutralPosition(part);
		map->CalculateGroundCoordinates(feet_next[i].getElement(1,4),feet_next[i].getElement(2,4),pos_int);
		feet_next[i].setElement(map->getMaxSquareHeight(pos_int[0],pos_int[1],5),3,4);
		if ((feet_prev[i].getElement(3,4)==-1)||(feet_next[i].getElement(3,4)==-1)||(feet_prev[i].getElement(3,4)>init_pos[2])||(feet_next[i].getElement(3,4)>dest_pos[2]))
			return false;
	}
	if (!verifyAchievability(next_body,feet_next))
		return false;
	if (!verifyAchievability(next_body,feet_prev))
		return false;
	if (map->getMaxSquareHeight(init_pos[0],init_pos[1],10)>init_pos[2])
		return false;
	if (map->getMaxSquareHeight(dest_pos[0],dest_pos[1],10)>dest_pos[2])
		return false;
	return true;
}

/// generate foothold trajectory
bool CFootPlanner::generateFootholdTraj(int shift){
	CPunctum actual_point, prev_point, foot;
	robot_platform_traj.getFirst(&prev_point);
	int part;
	float pos[3];
	for (int i=0;i<6;i++){//pozycje poczatkowe stop
		if (i<3) part=1; else part=0;
		legs_traj[i].clear();
		foot = prev_point*robot->leg[i].start*robot->leg[i].forward_kinematic(robot->leg[i].getAngle(0),robot->leg[i].getAngle(1),robot->leg[i].getAngle(2),part);
		legs_traj[i].savePosition(foot.getElement(1,4),foot.getElement(2,4),foot.getElement(3,4));
	}
	int smart_gait_iterator=0;
	int r[2];
	CPunctum foothold;
	while (robot_platform_traj.getNextShift(&actual_point,shift)) {
		for (smart_gait_iterator=0;smart_gait_iterator<2;smart_gait_iterator++){
			for (int i=smart_gait_iterator;i<6;i+=2) {
				if (i<3) part=1; else part=-1;
				legs_traj[i].getLastPoint(&pos[0],&pos[1],&pos[2]);
				CPunctum start; start.createTRMatrix(0,0,0,pos[0],pos[1],pos[2]);
				foot = actual_point*robot->leg[i].start*robot->leg[i].getNeutralPosition(part);
				map->CalculateGroundCoordinates((float)foot.getElement(1,4), (float)foot.getElement(2,4),r);
				selectFoothold(actual_point, r[0], r[1], i, pos);
				map->CalculateGlobalCoordinates(pos[0],pos[1],pos);
				CPunctum finish; finish.createTRMatrix(0,0,0,pos[0],pos[1],pos[2]);
				generateFootTraj(legs_traj,start, finish,prev_point, actual_point,0.035,i, (int)shift/2);//jezeli udalo sie przejsc do kolejnego punktu podparcia
			}
			for (int i=(smart_gait_iterator==0?1:0);i<6;i+=2) {//pozostale nogi w tym czasie sie nie ruszaja
				legs_traj[i].getLastPoint(&pos[0],&pos[1],&pos[2]);
				for (int j=0;j<(int)shift/2;j++) {
					foothold.createTRMatrix(0,0,0,pos[0],pos[1],pos[2]);
					foothold.setFoothold(true);
					legs_traj[i].savePunctum(foothold);
				}
			}
		}
		robot_platform_traj.reverse(shift);
		for (int j=0;j<6;j++){ legs_traj[j].reverse(shift);}
		CPunctum feet[6];//tablica zawierajaca pozycje stop w danej konfiguracji
		for (int i=0;i<shift;i++){
			CPunctum body;
			robot_platform_traj.getElement(&body);
			for (int j=0;j<6;j++){
				legs_traj[j].getElement(&feet[j]);
				if (!verifyAchievability(body,feet[j],j)){//sprawdz czy osiagalny - miesci sie w przestrzenii roboczej
					for (int k=0;k<6;k++)//usuwamy ostatni fragment trajektorii
						legs_traj[k].eraseLast(shift);
					robot_platform_traj.eraseLast(shift);
					return false;
				}
			}
			if (!robot->checkStability(body, feet, 0)){
				for (int k=0;k<6;k++)//usuwamy ostatni fragment trajektorii
					legs_traj[k].eraseLast(shift);
				robot_platform_traj.eraseLast(shift);
				return false;
			}
		}
		prev_point=actual_point;
	}
}

