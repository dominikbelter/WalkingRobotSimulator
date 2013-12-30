#include "OdeWorld.h"
#include <math.h>

	dWorldID World;             // the ode simulation world 
	dJointGroupID contactgroup; // wykorzystywane w kolizjach
	dGeomID *plane_c; //robot - wykrywanie kontaktu
	HexRobot* robot_collision; //robot - wykrywanie kontaktu

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
    int i;

    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
	dBodyID g = dGeomGetBody(*plane_c);

    dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box

    for (i = 0; i < MAX_CONTACTS; i++)
    {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = 8.5;
        contact[i].surface.mu2 = 8.5;
        contact[i].surface.bounce = 0.15;    // changed
        contact[i].surface.bounce_vel = 0.5;     // changed
        contact[i].surface.soft_cfm = 0.107;
		//contact[i].surface.slip1 = 0;
		//contact[i].surface.slip2 = 0;
    }

    if (int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact)))
    {
		if ((b1!=robot_collision->Object[3].Body)&&(b1!=robot_collision->Object[6].Body)&&(b1!=robot_collision->Object[9].Body)&&(b1!=robot_collision->Object[12].Body)&&(b1!=robot_collision->Object[15].Body)&&(b1!=robot_collision->Object[18].Body)&&(b2!=robot_collision->Object[3].Body)&&(b2!=robot_collision->Object[6].Body)&&(b2!=robot_collision->Object[9].Body)&&(b2!=robot_collision->Object[12].Body)&&(b2!=robot_collision->Object[15].Body)&&(b2!=robot_collision->Object[16].Body))
		{
			for (int it=0;it<6;it++)
			{
				if (b1==robot_collision->Object[19+it].Body&&b2==g||b1==g&&b2==robot_collision->Object[19+it].Body)
				{
					if (it==3) robot_collision->contact[5]=1;
					else if (it==5) robot_collision->contact[3]=1;
					else robot_collision->contact[it]=1;
				}
			}
			for (i = 0; i < numc; i++)
			{
				dJointID c = dJointCreateContact(World, contactgroup, contact + i);
				dJointAttach(c, b1, b2);
			}
		}
    }
}

COdeWorld::COdeWorld(int sizex, int sizey)
{
	robotODE = createSimRobotHexapod();

	ground = new CGround(5.99,5.99,sizex,sizey,0.36);
	rec_robot_platform.setDelay(1);//czas co jaki ma byc wywolywane nagrywanie
	rec_robot_orientation.setDelay(1);//czas co jaki ma byc wywolywane nagrywanie
	for (int i=0;i<6;i++)
		rec_robot_leg[i].setDelay(500);//czas co jaki ma byc wywolywane nagrywanie

		for (int i=0;i<6;i++) {
			foot_groundx[i]=0;
			foot_groundy[i]=0;
		}
		tx=0;
		ty=0;
	show_geoms=false; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	actual_time = 0;
}

COdeWorld::~COdeWorld(void)
{
	dJointGroupDestroy(jointgroup);
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(Space);
    dWorldDestroy(World);
	rec_robot_platform.savePosition2file("snn_pos.m");
	rec_robot_leg[0].savePosition2file("leg1.m");
	rec_robot_leg[1].savePosition2file("leg2.m");
	rec_robot_leg[2].savePosition2file("leg3.m");
	rec_robot_leg[3].savePosition2file("leg4.m");
	rec_robot_leg[4].savePosition2file("leg5.m");
	rec_robot_leg[5].savePosition2file("leg6.m");
	rec_robot_orientation.savePosition2file("snn_rot.m");
	delete ground;
}

// Draw a geom
void COdeWorld::DrawGeom(dGeomID g, const dReal *pos, const dReal *R, char corp, char leg)
{
    if (!g)
        return;

    if (!pos) pos = dGeomGetPosition(g);//pobiera pozycje obiektu

    if (!R)   R = dGeomGetRotation(g); //pobiera orientacje obiektu

    int type = dGeomGetClass(g); //pobiera klase obiektu
    
    if (type == dSphereClass)//rysuje sfere
    {
        dReal radius;
        radius = dGeomSphereGetRadius(g);
        geometry.DrawSphere(radius, (const float*)pos, (const float*)R); //rysuje sfere
    }

    if (type == dCCylinderClass)//rysuje pigule
    {
		dReal radius;
		dReal length;
		dGeomCCylinderGetParams (g, &radius, &length);
		if (leg){			
			geometry.DrawCCylinder(radius*1.5,length, (const float*)pos, (const float*)R);
		}
		else {
			geometry.DrawCappedCylinder((const float*)pos,(const float*) R, radius, length);//rysuje pigule
		}
    }

    if (type == dBoxClass) //rysuje szescian
    {
		dReal sides[3];
		if (corp) {
			dGeomBoxGetLengths(g, sides);	
			geometry.DrawKorpus(sides, (const float*)pos, (const float*)R);
		}
		else if (leg){
			dGeomBoxGetLengths(g, sides);
			float len=1;
			if (leg==1) len=0.055;
			else if (leg==2) len=0.16;
			else if (leg==3) len=0.230-0.02;
			if (leg==4)
				geometry.DrawCylinder((const float *)pos, (const float*)R, 0.013, 0.021, 0.04);
			else
				geometry.DrawCCylinderBox(0.012,len, (const float*)pos, (const float*)R);
		}
		else {
			dGeomBoxGetLengths(g, sides);
			geometry.DrawBox(sides, (const float*)pos, (const float*)R); //rysuje szescian
		}
    }
	robsim::float_type angles[3];
	robotODE->getRPY(angles);
	robsim::float_type robot_position[3];
	robotODE->getPosition(robot_position);
	//geometry.DrawCoordinateSystem(rad2deg(angles[0]),rad2deg(angles[1]),rad2deg(angles[2]),robot_position[0]*10,robot_position[1]*10,robot_position[2]*10);
}

// Simulation loop
void COdeWorld::SimStep()
{
	for (int i=0;i<6;i++)
		((HexRobot*)robotODE)->contact[i] = false;
    dSpaceCollide(Space, 0, &nearCallback);//ustawia sprawdzanie kolizji
	//if (time<100) //kopniecie robota
	//	dBodySetForce  (robotODE.Object[1].Body, 2, 1, 0);
	//time++;
    dWorldQuickStep(World, stepDT); //wykonuje krok symulacji
    dJointGroupEmpty(contactgroup);
	actual_time += stepDT;//uplywajacy czas symulacji

	//if (((actual_time>0.1))&&(actual_time<0.2))
	//	dBodySetForce(robotODE.Object[0].Body, 0, 100, -50);
	// ustawia wszystkie serwa zgodnie z wartosciami zadanymi
	robotODE->setAllServos();
	//nagranie pozycji platformy
	robsim::float_type robot_position[3];
	robotODE->getPosition(robot_position);
	rec_robot_platform.savePosition(robot_position[0],robot_position[1],robot_position[2]);
	//nagranie orientacji platformy
	robsim::float_type robot_orientation[3];
	robotODE->getRPY(robot_orientation);
	rec_robot_orientation.savePosition(robot_orientation[0],robot_orientation[1],robot_orientation[2]);
	//pobranie pozycji stopy
	CPunctum feet_pose;
	for (int i=0;i<6;i++) {
		robotODE->getFootPosition(i, feet_pose);
		rec_robot_leg[i].savePosition(feet_pose.getElement(1,4),feet_pose.getElement(2,4),feet_pose.getElement(3,4)); // nagranie pozycji stopy
	}
	checkFootContatcs();
	
//	if ((actual_time>0.1)&&(actual_time<0.11))
//		dBodyAddTorque(robotODE.Object[0].Body,50,0,0);
//	if ((actual_time>0.11)&&(actual_time<0.12))
//		dBodyAddTorque(robotODE.Object[0].Body,0,50,0);
}

void COdeWorld::DrawObjects()
{ //rysuje wszystkie figury na scenie
	if (show_geoms)
	{
		DrawGeom(robotODE->getGeomId(0), 0, 0, 1); //rysuje obiekt
		for (int i=0;i<6;i++) 
		{
			DrawGeom(robotODE->getGeomId(i*3+1), 0, 0, 0, 1); //ogniwo1
			DrawGeom(robotODE->getGeomId(i*3+2), 0, 0, 0, 2); //ogniwo2
			DrawGeom(robotODE->getGeomId(i*3+1), 0, 0, 0, 3); //ogniwo3
		}
		for (int bodies = 19;bodies<25;bodies++)
		{//rysowanie obiektow
			DrawGeom(robotODE->getGeomId(bodies), 0, 0, 0, 4); //prawa stopa
		}
	}
	else 
	{
		for (int bodies = 19;bodies<25;bodies++)//rysowanie obiektow
			DrawGeom(robotODE->getGeomId(bodies), 0, 0); //rysuje obiekt
	}

	ground->DrawMesh(foot_groundx,foot_groundy,foot_groundx_def, foot_groundy_def,tx,ty); //rysuje ziemie
	
	rec_robot_platform.plot(0,1,0,4,'o');
	for (int i=0;i<6;i++) {
		if (robotODE->getContact(i)==1)
			rec_robot_leg[i].plot(1,0,0,5,'o');
		else
			rec_robot_leg[i].plot(0,1,0,5,'o');
	}

	glColor3f(1, 1, 0);
	float thickness = 3;
	glPushMatrix();
			glTranslatef(body.getElement(1,4)*10, body.getElement(3,4)*10, -body.getElement(2,4)*10);
			glutSolidSphere(.03*thickness,5,5);
	glPopMatrix();
	for (int i=0;i<6;i++){
		glPushMatrix();
			glTranslatef(feet[i].getElement(1,4)*10, feet[i].getElement(3,4)*10, -feet[i].getElement(2,4)*10);
			glutSolidSphere(.03*thickness,5,5);
		glPopMatrix();
	}
}

void COdeWorld::InitODE(double dt, bool ct, int sizex, int sizey) /// inicjalizacja ODE
{
	stepDT = dt;
    World = dWorldCreate();//tworzenie 
    Space = dHashSpaceCreate(0);//swiata
    contactgroup = dJointGroupCreate(0);//tworzenie punktow styku
    jointgroup = dJointGroupCreate(0);// i zlacz
    dWorldSetGravity(World, 0, -9.81, 0);//ustawienie grawitacji
    dWorldSetCFM(World, 1e-5);//ustawienie parametrow
    dWorldSetERP(World, 0.2);// silnika
    dWorldSetContactMaxCorrectingVel(World, 0.9);//open
    dWorldSetContactSurfaceLayer(World, 0);//dynamics
    dWorldSetAutoDisableFlag(World, 0);//engine
 		
 	if (!ct)	
	ground->randMesh(); //losujemy trudny teren
	else ground->readCustMap(sizex,sizey);
	indexes = new int[3*2*(ground->getYnumPoints()-1)*(ground->getXnumPoints()-1)];//liczba trojkatow
	ground->returnIndexes(indexes);
	triVert = new dVector3[ground->getYnumPoints()*ground->getXnumPoints()];
	ground->returnTriVert(triVert);
	triMesh = dGeomTriMeshDataCreate();
 	dGeomTriMeshDataBuildSimple(triMesh, (dReal*)triVert, (ground->getYnumPoints())*(ground->getXnumPoints()), indexes, 3*2*(ground->getYnumPoints()-1)*(ground->getXnumPoints()-1));
	plane = dCreateTriMesh(Space, triMesh, NULL, NULL, NULL);
	dGeomSetData(plane, "Plane");
	dGeomSetPosition(plane, 0, 0, 0);
	plane_c=&plane;

	// definicja robota w ode
	robotODE->ODEcreateRobot(World, Space, jointgroup, stepDT);
	robot_collision = (HexRobot*) robotODE;
}

/// zamyka i niszczy swiat ODE
void COdeWorld::CloseODE()
{
    dJointGroupDestroy(jointgroup);
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(Space);
    dWorldDestroy(World);
}

/// odczytuje pozycje robota
void COdeWorld::showRobotPosition()
{
	robsim::float_type robot_position[3];//pozycja platformy
	robotODE->getPosition(robot_position);
	robsim::float_type robot_orientation[3];//orientacja platformy
	robotODE->getRPY(robot_orientation);
	robot_position[0]=0.0;
	robot_position[1]=0.0;
	robot_position[2]=0.0;
	robot_orientation[0]=0.0;
	robot_orientation[1]=0.0;
	robot_orientation[2]=0.0;
}

void COdeWorld::checkFootContatcs(void) {
	float max;
	CPunctum pos;
	int pos_int[3];
	for (int leg_no=0;leg_no<6;leg_no++) {
		this->robotODE->getFootPosition(leg_no, pos);
		ground->CalculateGroundCoordinates(pos.getElement(1,4),pos.getElement(2,4),pos_int);
		max=-10;
		int i=0,j=0;
		//for (int i=-1;i<2;i++){
	//		for (int j=-1;j<2;j++){
				if ((ground->points[pos_int[0]+i][pos_int[1]+j][2]+0.003)>max)
					max=ground->points[pos_int[0]+i][pos_int[1]+j][2]+0.003;
			//}
		//}
		if (max+0.00>pos.getElement(3,4)||((HexRobot*)robotODE)->contact[leg_no])
			robotODE->setContact(leg_no,true);
		else
			robotODE->setContact(leg_no,false);
	}
}