#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <thread>
#include <gl/glut.h>
#include <ode/ode.h>

//modu³y
#include "ODE/OdeWorld.h"
#include "collision_detection/RobotStructure.h"
#include "collision_detection/MessorStructure.h"
#include "collision_detection/StarlETHStructure.h"
#include "openGL/openGLmain.h"
#include "robot_controller/RPCCaller.h"
#include "motion_planner/motion_planner.h"
#include "mapping/idealMap.h"
#include "mapping/localMap.h"
#include "functions.h"

#define _AFXDLL

using namespace std;

//œwiat fizyczny ODE
COdeWorld* dynamicWorld;
//struktura fizuczna robota
RobotStructure* robot_structure;
//RPCCaller
RPCCaller* rpccaller;
//mapa idealna - teren po którym porusza siê robot
CIdealMap* map;
//modu³ planowania ruchu - wyszukiwanie œcie¿ek przez robota
CMotionPlanner* motion_planner;
//mapa tworzona w czasie poruszania siê robota
CLocalMap* local_map;

//rozmiar mapy terenu
int sizex = 400, sizey = 400;

//////////////////////////////////////

//w¹tek kontroli robota
void controller(void)
{	
//	float pos_end[3] = {0.0,0.0,0.13};
//	float rot_end[3] = {0.0,0,0.0};
//	float distance2ground = 0.035;//the distance of feet to the ground
//	float map_boundaries[4]={-map->getXsize()/2,map->getXsize()/2,-map->getYsize()/2,map->getYsize()/2};
//	motion_planner->rrtConnect(pos_end, rot_end, distance2ground, map_boundaries,20);
//	motion_planner->robot_platform_traj.savePosition2file("zaplanowana_sciezka_platformy.txt");
//	motion_planner->legs_traj[0].savePosition2file("zaplanowana_sciezka_stopy0.txt");
//	motion_planner->legs_traj[1].savePosition2file("zaplanowana_sciezka_stopy1.txt");
//	motion_planner->legs_traj[2].savePosition2file("zaplanowana_sciezka_stopy2.txt");
//	motion_planner->legs_traj[3].savePosition2file("zaplanowana_sciezka_stopy3.txt");
//	motion_planner->legs_traj[4].savePosition2file("zaplanowana_sciezka_stopy4.txt");
//	motion_planner->legs_traj[5].savePosition2file("zaplanowana_sciezka_stopy5.txt");
////	motion_planner->pathSmoothing(0.1,0.9,1,1);
//	motion_planner->executeTrajectory(0.9);


	///------DEMO
	//motion in neutral position of the robot, parameter: x, y, z, roll, pitch, yaw, speed, acceleration=1
	rpccaller->movePlatform(0.1,0,0,0,0,0,0.45,1);
	rpccaller->movePlatform(0.0,0.1,0,0,0,0,0.45,1);
	rpccaller->movePlatform(0.0,0,0.1,0,0,0,0.95,1);
	rpccaller->movePlatform(0,0,0,0,0,0,0.15,1);

	//motion in current position of the robot, parameter: x, y, z, roll, pitch, yaw, speed, acceleration=1
	rpccaller->movePlatformRobot(-0.1,0,0,0,0,0,0.15,1);
	rpccaller->movePlatformRobot(0.0,0.1,0,0,0,0,0.15,1);
	rpccaller->movePlatformRobot(0.1,0,0.1,0,0,0,0.15,1);
	rpccaller->movePlatformRobot(0,-0.1,-0.1,0,0,0,0.15,1);

	//complex motions -- each leg generates separate motion of the body
	float xx[6]={0,0,0,0,0,0};
	float zz[6]={0,0,0,0.0,-0.1,0.0};
	rpccaller->movePlatformComplex(xx,xx,zz,xx,xx,xx,0.15,1);
	rpccaller->movePlatform(0,0,0,0,0,0,0.15,1);
	

	//forward
	rpccaller->tripodStepPrepare(0.0,0.1,0,0,0,0,0.1);
	rpccaller->tripodStep(0.0,0.1,0,0,0,0,0.1,1);
	rpccaller->tripodStep(0.0,0.1,0,0,0,0,0.1,1);
	rpccaller->tripodStepFinish(0.0,0.1,0,0,0,0,0.1);
	//sideways
	rpccaller->tripodStepPrepare(0.1,0.0,0,0,0,0,0.1);
	rpccaller->tripodStep(0.1,0.0,0,0,0,0.0,0.1,1);
	rpccaller->tripodStep(0.1,0.0,0,0,0,0.0,0.1,1);
	rpccaller->tripodStepFinish(0.1,0.0,0,0,0,0,0.1);
	
	float pos_end[3] = {0.0,0.7,0.13};
	float rot_end[3] = {0.0,0,0.0};
	float distance2ground = 0.035;//the distance of feet to the ground
	float map_boundaries[4]={-map->getXsize()/2,map->getXsize()/2,-map->getYsize()/2,map->getYsize()/2};
	motion_planner->rrtConnect(pos_end, rot_end, distance2ground, map_boundaries,20);
	motion_planner->robot_platform_traj.savePosition2file("body_path.m");
	motion_planner->legs_traj[0].savePosition2file("foot0_path.m");
	motion_planner->legs_traj[1].savePosition2file("foot1_path.m");
	motion_planner->legs_traj[2].savePosition2file("foot2_path.m");
	motion_planner->legs_traj[3].savePosition2file("foot3_path.m");
	motion_planner->legs_traj[4].savePosition2file("foot4_path.m");
	motion_planner->legs_traj[5].savePosition2file("foot5_path.m");
	motion_planner->savePath2File("path.txt");
	//motion_planner->loadPathFromFile("path1.txt");
	motion_planner->executeTrajectory(0.1);
	///!------DEMO

	for (int i=0;i<8000;i++){
		dynamicWorld->SimStep();
	}
	dynamicWorld->rec_robot_platform.savePosition2file("real_trajectory_pos.m");
	dynamicWorld->rec_robot_orientation.savePosition2file("real_trajectory_rot.m");
	
	while(1)
	{//pêtla SimStep - symulacja fizyki po zakoñczeniu symulacji kroczenia robota
		dynamicWorld->SimStep();
	}
}

//////////////////////////////////////

int main(void)
{
	printf("Initialization...\n");
	//initialize rand
	initializeRand();
	//stworzenie œwiata ODE (sizex x sizey)
	dynamicWorld = new COdeWorld(sizex,sizey);
	//ustalenie pozycji fizycznego robota na scenie (x, y, z, rot_x, rot_y, rot_z)
	dynamicWorld->robotODE->setInitialPosition(-0.1,0.1,0.22,0,0,0);
	//zainicjowanie œwiata ODE (krok ca³kowania, wczytanie mapy z pliku true/false, sizex x sizey)
	dynamicWorld->InitODE(0.0001,true,sizex,sizey);

	//stworzenie fizycznej struktury robota 
	robot_structure = createMessorRobotStructure();
	//robot_structure = createStarlETHRobotStructure();

	//stworzenie idealnej mapy terenu - œrodowiska w którym porusza siê robot
	map = new CIdealMap(dynamicWorld);

	//stworzenie struktury mapy lokalnej - tworzonej przez robota na podstawie mapy idealnej (120 x 120 rastrów, ka¿dy 1.5 x 1.5 cm)
	local_map=new CLocalMap(120,120,0.015,0.015,0,map);

	//stworzenie struktury RPCCaller s³u¿¹cej wysy³aniu poleceñ do robota
	rpccaller = new RPCCaller("150.254.46.220",2426,dynamicWorld,robot_structure,local_map);

	//stworzenie modu³u odpowiedzialnego za planowanie ruchu - wyszukiwanie œcie¿ek przez robota
	motion_planner = new CMotionPlanner(map,rpccaller);

	//zainicjowanie modu³u openGL
	openGLinit(dynamicWorld, robot_structure, motion_planner, local_map);

	//w¹tek kontroli robota
	std::thread robot_controller_thread(controller);

	//pêtla g³ówna openGL
	glutMainLoop();

	robot_controller_thread.join();
	//destruktory
	delete motion_planner;
	delete rpccaller;
	delete local_map;
	delete map;
	dynamicWorld->CloseODE();
	delete dynamicWorld;
}