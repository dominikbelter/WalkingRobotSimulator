#include "StarlETHStructure.h"
#include "objects3DS.h"

	#define STARLETH_WIDTH 0.185
	#define STARLETH_LENGTH 0.2525
	#define STARLETH_SEGMENT1 0.0685
	#define STARLETH_SEGMENT2 0.2
	#define STARLETH_SEGMENT3 0.2350
	#define STARLETH_FOOT_LENGTH 0.02

/// A single instance of StarlETHStructure
StarlETHStructure::Ptr starlETHStructure;

StarlETHStructure::StarlETHStructure(void) : RobotStructure("StarlETH Robot Structure")
{
	//for (int i=0;i<18;i++)
	//	angles[i]=0;
	//angles[0]=45*3.14/180;
	//angles[1]=-45*3.14/180;
	robot_model.ObjLoad("resources/StarlETH_model/StarlETH_base.3ds");
	//robot_model.ObjLoad("resources/StarlETH_model/StarlETH_hip.3ds");
	robot_model.ObjLoad("resources/StarlETH_model/StarlETH_thigh.3ds");
	robot_model.ObjLoad("resources/StarlETH_model/StarlETH_shank.3ds");
	
	for (int i=0;i<13;i++) {
		CollisionModel3D* tmp = newCollisionModel3D();
		meshModel.push_back(tmp);
	}
//	InitializeTerrain();
	CollisionModels();	// Init Collision Models
//	robot_model.TerrainCollisionModels();	// Init Collision Models
	initStructures();
}

StarlETHStructure::~StarlETHStructure(void)
{
}

void StarlETHStructure::initCollisionModel(uint_fast8_t objectNo, CollisionModel3D& model) {
	cout << "model size: " << robot_model.object[objectNo].polygons_qty << " triangles" <<endl;
	for (int j=0;j<robot_model.object[objectNo].polygons_qty;j++) {
		model.addTriangle(	robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].a ].x*0.254, robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].a ].y*0.254, robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].a ].z*0.254, 
							robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].b ].x*0.254, robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].b ].y*0.254,	robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].b ].z*0.254,
							robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].c ].x*0.254, robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].c ].y*0.254,	robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].c ].z*0.254);
	}
	model.finalize();
}

void StarlETHStructure::CollisionModels(void)
{
	initCollisionModel(0, *meshModel[PLATFORM]); // robot's platform

	/*initCollisionModel(6, *meshModel[HIP1]); 
	initCollisionModel(6, *meshModel[HIP2]); 
	initCollisionModel(6, *meshModel[HIP3]); 
	initCollisionModel(6, *meshModel[HIP4]); */

	initCollisionModel(1, *meshModel[THIGH1]); 
	initCollisionModel(1, *meshModel[THIGH2]);
	initCollisionModel(1, *meshModel[THIGH3]);
	initCollisionModel(1, *meshModel[THIGH4]); 

	initCollisionModel(2, *meshModel[SHANK1]); 
	initCollisionModel(2, *meshModel[SHANK2]); 
	initCollisionModel(2, *meshModel[SHANK3]); 
	initCollisionModel(2, *meshModel[SHANK4]); 
}

void StarlETHStructure::initStructures(void)
{
	structPlatform();
	//structLegHip();
	structLegThigh();
	structLegShank();
}

void StarlETHStructure::structPlatform(void)
{
	glNewList(GL_PLATFORM, GL_COMPILE);
	glColor3f(0.5,0.5,0.5);
	//glRotatef(90,0,0,1);
	robot_model.Object3DS(0,0.001/0.0254);
	glEndList();
}

/*void StarlETHStructure::structHip(void)
{
	glNewList(GL_HIP, GL_COMPILE);
	glColor3f(0.6,0.6,0.6);
	glScalef(0.0254,0.0254,0.0254);
	robot_model.Object3DS(1);
	glEndList();
}*/

void StarlETHStructure::structLegThigh(void)
{
	glNewList(GL_THIGH, GL_COMPILE);
	glColor3f(0.6,0.6,0.6);
	//glScalef(0.0254,0.0254,0.0254);
	robot_model.Object3DS(1,0.001/0.0254);
	glEndList();
}

void StarlETHStructure::structLegShank(void)
{
	glNewList(GL_SHANK, GL_COMPILE);
	glColor3f(0.6,0.6,0.6);
	//glScalef(0.0254,0.0254,0.0254);
	robot_model.Object3DS(2,0.001/0.0254);
	glEndList();
}

void StarlETHStructure::drawCoordinateSystem(void)
{
	glLineWidth(3);
    glBegin(GL_LINES);
        glColor3f(1, 0, 0);
        glVertex3f(0, 0, 0);
        glVertex3f(0.5, 0, 0);
        
		glColor3f(0, 1, 0);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0.5, 0);
        
		glColor3f(0, 0, 1);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 0.5);
    glEnd();
    glLineWidth(1);

								
    glPointSize(5);
    glBegin(GL_POINTS);
        glColor3f(1, 0, 0);
        glVertex3f(0.5, 0, 0);
        glColor3f(0, 1, 0);
        glVertex3f(0, 0.5, 0);
        glColor3f(0, 0, 1);
        glVertex3f(0, 0, 0.5);
    glEnd();
    glPointSize(1);
	glColor3f(1, 1, 1);
}

void StarlETHStructure::copyTable(CPunctum * src, float * dest) const{
	for (int i=0;i<4;i++){
		for (int j=0;j<4;j++){
			dest[i+4*j]=src->getElement(i+1,j+1);
		}
	}
}

void StarlETHStructure::Leg1(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const {
	float przegub_c1[16];
	CPunctum m_noga1,tmp;
	m_noga1 = (*m_noga)*tmp.makeTransformMatrix("gamma", Qn_1*3.14/180);
	copyTable(&m_noga1,przegub_c1);
	//meshModel[20]->setTransform (przegub_c1);
					
	float czlon1_m1[16];
	CPunctum m_noga2;
	m_noga2 = m_noga1 * tmp.makeTransformMatrix("alpha", -3.14/2)*tmp.makeTransformMatrix("x", -2.15*0.254)*tmp.makeTransformMatrix("y", -0.49*0.254)*tmp.makeTransformMatrix("z", 0.92*0.254)*tmp.makeTransformMatrix("gamma", Qn_2*3.14/180);
	copyTable(&m_noga2,czlon1_m1);
	meshModel[THIGH1]->setTransform (czlon1_m1);
										
	float segment1[16];
	CPunctum m_noga3;
	m_noga3 = m_noga2 * tmp.makeTransformMatrix("x", -6.28*0.254)*tmp.makeTransformMatrix("z", -1.69*0.254)*tmp.makeTransformMatrix("gamma", Qn_3*3.14/180);
	copyTable(&m_noga3,segment1);
	meshModel[SHANK1]->setTransform (segment1);
	
	/*float stopka_1[16];
	CPunctum m_noga4;
	m_noga4 = m_noga3 * tmp.makeTransformMatrix("x", -7.9*0.254)*tmp.makeTransformMatrix("y", -0.03*0.254)*tmp.makeTransformMatrix("z", 0.8*0.254)*tmp.makeTransformMatrix("beta", 90*3.14/180)*tmp.makeTransformMatrix("alpha", -28.6*3.14/180)*tmp.makeTransformMatrix("z", -0.45*0.254);
	copyTable(&m_noga4,stopka_1);
	meshModel[2]->setTransform (stopka_1);*/
}

void StarlETHStructure::Leg2(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const {
	float przegub_c2[16];
	CPunctum m_noga1,tmp;
	m_noga1 = (*m_noga)*tmp.makeTransformMatrix("gamma", Qn_1*3.14/180);
	copyTable(&m_noga1,przegub_c2);
	//meshModel[21]->setTransform (przegub_c2);

	float czlon1_m2[16];
	CPunctum m_noga2;
	m_noga2 = m_noga1 * tmp.makeTransformMatrix("alpha", -3.14/2)*tmp.makeTransformMatrix("x", -2.15*0.254)*tmp.makeTransformMatrix("y", -0.49*0.254)*tmp.makeTransformMatrix("z", 0.92*0.254)*tmp.makeTransformMatrix("gamma", Qn_2*3.14/180);
	copyTable(&m_noga2,czlon1_m2);
	meshModel[THIGH2]->setTransform (czlon1_m2);
										
	float segment2[16];
	CPunctum m_noga3;
	m_noga3 = m_noga2 * tmp.makeTransformMatrix("x", -6.28*0.254)*tmp.makeTransformMatrix("z", -1.69*0.254)*tmp.makeTransformMatrix("gamma", Qn_3*3.14/180);
	copyTable(&m_noga3,segment2);
	meshModel[SHANK2]->setTransform (segment2);

	/*float stopka_2[16];
	CPunctum m_noga4;
	m_noga4 = m_noga3 * tmp.makeTransformMatrix("x", -7.9*0.254)*tmp.makeTransformMatrix("y", -0.03*0.254)*tmp.makeTransformMatrix("z", 0.8*0.254)*tmp.makeTransformMatrix("beta", 90*3.14/180)*tmp.makeTransformMatrix("alpha", -28.6*3.14/180)*tmp.makeTransformMatrix("z", -0.45*0.254);
	copyTable(&m_noga4,stopka_2);
	meshModel[3]->setTransform (stopka_2);*/
}

void StarlETHStructure::Leg3(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const {
	float przegub_c3[16];
	CPunctum m_noga1,tmp;
	m_noga1 = (*m_noga)*tmp.makeTransformMatrix("gamma", Qn_1*3.14/180);
	copyTable(&m_noga1,przegub_c3);
	//meshModel[22]->setTransform (przegub_c3);

	float czlon1_m3[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, czlon1_m3);
	CPunctum m_noga2;
	m_noga2 = m_noga1 * tmp.makeTransformMatrix("alpha", -3.14/2)*tmp.makeTransformMatrix("x", -2.15*0.254)*tmp.makeTransformMatrix("y", -0.49*0.254)*tmp.makeTransformMatrix("z", 0.92*0.254)*tmp.makeTransformMatrix("gamma", Qn_2*3.14/180);
	copyTable(&m_noga2,czlon1_m3);
	meshModel[THIGH3]->setTransform (czlon1_m3);

	float segment3[16];
	CPunctum m_noga3;
	m_noga3 = m_noga2 * tmp.makeTransformMatrix("x", -6.28*0.254)*tmp.makeTransformMatrix("z", -1.69*0.254)*tmp.makeTransformMatrix("gamma", Qn_3*3.14/180);
	copyTable(&m_noga3,segment3);
	meshModel[SHANK3]->setTransform (segment3);

	/*CPunctum m_noga4;
	m_noga4 = m_noga3 * tmp.makeTransformMatrix("x", -7.9*0.254)*tmp.makeTransformMatrix("y", -0.03*0.254)*tmp.makeTransformMatrix("z", 0.76*0.254)*tmp.makeTransformMatrix("beta", 90*3.14/180)*tmp.makeTransformMatrix("alpha", -28.6*3.14/180)*tmp.makeTransformMatrix("z", -0.45*0.254);
	float stopka_3[16];
	copyTable(&m_noga4,stopka_3);
	meshModel[4]->setTransform (stopka_3);*/
}

void StarlETHStructure::Leg4(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const {
	float przegub_c4[16];
	CPunctum m_noga1, tmp;
	m_noga1 = (*m_noga)*tmp.makeTransformMatrix("gamma", Qn_1*3.14/180);
	copyTable(&m_noga1,przegub_c4);
	//meshModel[23]->setTransform (przegub_c4);
								
	float czlon1_m4[16];
	CPunctum m_noga2;
	m_noga2 = m_noga1 * tmp.makeTransformMatrix("alpha", -3.14/2)*tmp.makeTransformMatrix("x", -2.15*0.254)*tmp.makeTransformMatrix("y", -0.49*0.254)*tmp.makeTransformMatrix("z", 0.92*0.254)*tmp.makeTransformMatrix("gamma", Qn_2*3.14/180);
	copyTable(&m_noga2,czlon1_m4);
	meshModel[THIGH4]->setTransform (czlon1_m4);

	float segment4[16];
	CPunctum m_noga3;
	m_noga3 = m_noga2 * tmp.makeTransformMatrix("x", -6.28*0.254)*tmp.makeTransformMatrix("z", -1.69*0.254)*tmp.makeTransformMatrix("gamma", Qn_3*3.14/180);
	copyTable(&m_noga3,segment4);
	meshModel[SHANK4]->setTransform (segment4);

	/*float stopka_4[16];
	CPunctum m_noga4;
	m_noga4 = m_noga3 * tmp.makeTransformMatrix("x", -7.9*0.254)*tmp.makeTransformMatrix("y", -0.03*0.254)*tmp.makeTransformMatrix("z", 0.8*0.254)*tmp.makeTransformMatrix("beta", 90*3.14/180)*tmp.makeTransformMatrix("alpha", -28.6*3.14/180)*tmp.makeTransformMatrix("z", -0.45*0.254);
	copyTable(&m_noga4,stopka_4);
	meshModel[5]->setTransform (stopka_4);*/
}

void StarlETHStructure::GLLeg1(float Qn_1, float Qn_2, float Qn_3) const {
	glRotatef(Qn_1,1,0,0);
	/*glPushMatrix();
		glCallList(GL_LEG_SHEET1);
		glTranslatef(-2.14*0.254,0,0.49*0.254);
		glRotatef(90,1,0,0);
		glRotatef(180,0,1,0);
		glRotatef(90,0,0,1);
		glCallList(GL_LEG_SHEET2);
	glPopMatrix();*/

	glRotatef(90,1,0,0);
	glRotatef(90,0,0,1);
	glTranslatef(STARLETH_SEGMENT1*10,0,0);
	glRotatef(Qn_2,0,0,1);
	glPushMatrix();
		glCallList(GL_THIGH);
		glTranslatef(STARLETH_SEGMENT2*10,0,-0);
		glRotatef(Qn_3-90,0,0,1);
		glPushMatrix();
			glCallList(GL_SHANK);
			/*glTranslatef(0,0,1.61*0.254);
			glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
			glRotatef(90,0,1,0);
			glRotatef(-28.6,1,0,0);
			glCallList(GL_BASE);
			glTranslatef(0,0,-0.45*0.254);
			glCallList(GL_FOOT);*/
		glPopMatrix();
	glPopMatrix();
}

void StarlETHStructure::GLLeg2(float Qn_1, float Qn_2, float Qn_3) const {
	glRotatef(180,0,0,1);
	glRotatef(Qn_1,1,0,0);
	/*glPushMatrix();
		glCallList(GL_LEG_SHEET1);
		glTranslatef(-2.14*0.254,0,0.49*0.254);
		glRotatef(90,1,0,0);
		glRotatef(180,0,1,0);
		glRotatef(90,0,0,1);
		glCallList(GL_LEG_SHEET2);
	glPopMatrix();*/

	glRotatef(90,1,0,0);
	glRotatef(90,0,0,1);
	glTranslatef(STARLETH_SEGMENT1*10,0,0);
	glRotatef(Qn_2,0,0,1);
	glPushMatrix();
		glCallList(GL_THIGH);
		glTranslatef(STARLETH_SEGMENT2*10,0,-0);
		glRotatef(Qn_3-90,0,0,1);
		glPushMatrix();
			glCallList(GL_SHANK);
			/*glTranslatef(0,0,1.61*0.254);	
			glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
			glRotatef(90,0,1,0);
			glRotatef(-28.6,1,0,0);
			glCallList(GL_BASE);
			glTranslatef(0,0,-0.45*0.254);
			glCallList(GL_FOOT);*/
		glPopMatrix();
	glPopMatrix();
}

void StarlETHStructure::GLLeg3(float Qn_1, float Qn_2, float Qn_3) const {
	glRotatef(180,0,0,1);
	glRotatef(Qn_1,1,0,0);
	/*glPushMatrix();
		glCallList(GL_LEG_SHEET1);
		glTranslatef(-2.14*0.254,0,0.49*0.254);
		glRotatef(90,1,0,0);
		glRotatef(180,0,1,0);
		glRotatef(90,0,0,1);
		glCallList(GL_LEG_SHEET2);
	glPopMatrix();*/
	
	glRotatef(90,1,0,0);
	glRotatef(90,0,0,1);
	glTranslatef(STARLETH_SEGMENT1*10,0,0);
	glRotatef(Qn_2,0,0,1);
	glPushMatrix();
		glCallList(GL_THIGH);
		glTranslatef(STARLETH_SEGMENT2*10,0,-0);
		glRotatef(Qn_3-90,0,0,1);
		glPushMatrix();
			glCallList(GL_SHANK);
			/*glTranslatef(0,0,1.57*0.254);
			glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
			glRotatef(90,0,1,0);
			glRotatef(-28.6,1,0,0);
			glCallList(GL_BASE);
			glTranslatef(0,0,-0.45*0.254);
			glCallList(GL_FOOT);*/
		glPopMatrix();
	glPopMatrix();
}

void StarlETHStructure::GLLeg4(float Qn_1, float Qn_2, float Qn_3) const {
	glRotatef(Qn_1,1,0,0);
	/*glPushMatrix();
		glCallList(GL_LEG_SHEET1);
		glTranslatef(-2.14*0.254,0,0.49*0.254);
		glRotatef(90,1,0,0);
		glRotatef(180,0,1,0);
		glRotatef(90,0,0,1);
		glCallList(GL_LEG_SHEET2);
	glPopMatrix();*/

	glRotatef(90,1,0,0);
	glRotatef(90,0,0,1);
	glTranslatef(STARLETH_SEGMENT1*10,0,0);
	glRotatef(Qn_2,0,0,1);
	glPushMatrix();
		glCallList(GL_THIGH);
		glTranslatef(STARLETH_SEGMENT2*10,0,-0);
		glRotatef(Qn_3-90,0,0,1);
		glPushMatrix();
			glCallList(GL_SHANK);
			/*glTranslatef(0,0,1.61*0.254);
			glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
			glRotatef(90,0,1,0);
			glRotatef(-28.6,1,0,0);
			glCallList(GL_BASE);
			glTranslatef(0,0,-0.45*0.254);
			glCallList(GL_FOOT);*/
		glPopMatrix();
	glPopMatrix();
}

void StarlETHStructure::DrawRobot(robsim::float_type* pos, robsim::float_type* rot, robsim::float_type * angles) const
{
	CPunctum m4,tmp;
	m4.setEye();
	m4=tmp.makeTransformMatrix("x", pos[0]*10)*tmp.makeTransformMatrix("y", pos[2]*10-0.1)*tmp.makeTransformMatrix("z", -pos[1]*10)*tmp.makeTransformMatrix("alpha", 3.14/2+rot[0])*tmp.makeTransformMatrix("beta", -rot[1])*tmp.makeTransformMatrix("gamma", 3.14-rot[2]);
	float korpus_dol_m[16];
	copyTable(&m4,korpus_dol_m);
	meshModel[0]->setTransform (korpus_dol_m);
	
	m4=m4*tmp.makeTransformMatrix("z", -4.24*0.254);
	float korpus_gora_m[16];
	copyTable(&m4,korpus_gora_m);
	meshModel[1]->setTransform (korpus_gora_m);

	CPunctum m_noga;
	//===============LEG_1=================================
	m_noga = m4*tmp.makeTransformMatrix("x", -2.56*0.254)*tmp.makeTransformMatrix("y", 6.06*0.254)*tmp.makeTransformMatrix("z", 3.33*0.254);
	Leg1(-angles[0]*180/3.14,-angles[1]*180/3.14,-angles[2]*180/3.14,&m_noga);

	//===============LEG_2=================================				
	m_noga = m4*tmp.makeTransformMatrix("x", -5.1*0.254)*tmp.makeTransformMatrix("z", 3.33*0.254);
	Leg2(-angles[3]*180/3.14,-angles[4]*180/3.14,-angles[5]*180/3.14,&m_noga); 

//===============NOGA_3=================================

	m_noga = m4*tmp.makeTransformMatrix("x", -2.56*0.254)*tmp.makeTransformMatrix("y", -6.06*0.254)*tmp.makeTransformMatrix("z", 3.33*0.254);
	Leg3(-angles[6]*180/3.14,-angles[7]*180/3.14,-angles[8]*180/3.14,&m_noga); 

//===============NOGA_4=================================
	m_noga = m4*tmp.makeTransformMatrix("x", 2.56*0.254)*tmp.makeTransformMatrix("y", -6.06*0.254)*tmp.makeTransformMatrix("z", 3.33*0.254)*tmp.makeTransformMatrix("gamma", 3.14);
	Leg4(angles[9]*180/3.14,-angles[10]*180/3.14,-angles[11]*180/3.14,&m_noga);	

}

void StarlETHStructure::GLDrawRobot(robsim::float_type *pos, robsim::float_type * rot, std::vector<robsim::float_type> config) const {

	float GLmat[16]={rot[0], rot[4], rot[8], 0, rot[1], rot[5], rot[9], 0, rot[2], rot[6], rot[10], 0, pos[0], pos[1], pos[2], 1}; //macierz do przeksztalcen

	glPushMatrix();
		glMultMatrixf(GLmat);
		glRotatef(90,1,0,0);
		glRotatef(180,0,0,1);
		glPushMatrix();
			glCallList(GL_PLATFORM);
		glPopMatrix();
		//glTranslatef(0.0f,0.0f,-4.24*0.254);
		//glPushMatrix();
//			glCallList(GL_PLATFORM_TOP);
	//	glPopMatrix();
			
		//===============LEG_1=================================
		glPushMatrix();
			glTranslatef(-STARLETH_LENGTH*10, -STARLETH_WIDTH*10, 0);
			GLLeg1(config[0]*180/3.14,config[1]*180/3.14,config[2]*180/3.14);	
		glPopMatrix();

		//===============LEG_2=================================				
		glPushMatrix();
			glTranslatef(STARLETH_LENGTH*10, -STARLETH_WIDTH*10, 0);
			GLLeg2(-config[3]*180/3.14,-config[4]*180/3.14,-config[5]*180/3.14); 
		glPopMatrix();

//===============LEG_3=================================
		glPushMatrix();
			glTranslatef(STARLETH_LENGTH*10, STARLETH_WIDTH*10, 0);
			GLLeg3(-config[6]*180/3.14,-config[7]*180/3.14,-config[8]*180/3.14); 
		glPopMatrix();

//===============LEG_4=================================
		glPushMatrix();
			glTranslatef(-STARLETH_LENGTH*10, STARLETH_WIDTH*10, 0);
			//glRotatef(180,0,0,1);
			GLLeg4(config[9]*180/3.14,config[10]*180/3.14,config[11]*180/3.14);	
		glPopMatrix();

	glPopMatrix();
}

bool StarlETHStructure::checkCollision(robsim::float_type* pos, robsim::float_type* rot, robsim::float_type * angles, bool * collision_table) const {

	DrawRobot(pos, rot, angles);
	for (int i=0;i<44;i++){
		collision_table[i]=false;
	}

	//*******KOLIZJE KOÑCZYN ROBOTA******************************************************************
	//=========KOLIZJE pierwszym, a drugim ogniwem od korpusu  ========================
	//collision_table[0-5] pierwszy czlon koliduje
	//collision_table[6-11] drugi czlon koliduje
	//collision_table[12-17] trzeci czlon koliduje
	//collision_table[18-23] trzeci czlon koliduje z terenem
	//collision_table[24] teren koliduje
	//collision_table[25] korpus koliduje
	//simplified method
	/*for (int i=0;i<4;i++) {
		if ((angles[i*3+1]>(24*3.14/180+1.1))){
			collision_table[i]=true;
			collision_table[i+6]=true;
		}
	}
	for (int i=0;i<6;i++) {
		if (abs(angles[i*3])>1.57){
			collision_table[i]=true;
		}
	}*/
	/*
	collision_table[0]=robot_model.segment_II_model_1->collision(robot_model.przegub_typu_C_1);
	if (collision_table[0]) collision_table[6]=true;
	collision_table[1]=robot_model.segment_II_model_2->collision(robot_model.przegub_typu_C_2);
	if (collision_table[1]) collision_table[7]=true;
	collision_table[2]=robot_model.segment_II_model_3->collision(robot_model.przegub_typu_C_3);
	if (collision_table[2]) collision_table[8]=true;
	collision_table[3]=robot_model.segment_II_model_4->collision(robot_model.przegub_typu_C_4);
	if (collision_table[3]) collision_table[9]=true;
	collision_table[4]=robot_model.segment_II_model_5->collision(robot_model.przegub_typu_C_5);
	if (collision_table[4]) collision_table[10]=true;
	collision_table[5]=robot_model.segment_II_model_6->collision(robot_model.przegub_typu_C_6);
	if (collision_table[5]) collision_table[11]=true;
	*/
	//=========KOLIZJE drugimi ogniwami od korpusu roznymi nogami
	/*if (meshModel[SEGMENT_II1]->collision(meshModel[SEGMENT_II2])) {
		collision_table[6]=true; collision_table[7]=true;
	}
	if (meshModel[SEGMENT_II2]->collision(meshModel[SEGMENT_II3])) {
		collision_table[7]=true; collision_table[8]=true;
	}
	if (meshModel[SEGMENT_II4]->collision(meshModel[SEGMENT_II5])) {
		collision_table[9]=true; collision_table[10]=true;
	}
	if (meshModel[SEGMENT_II5]->collision(meshModel[SEGMENT_II6])) {
		collision_table[10]=true; collision_table[11]=true;
	}
	//=========KOLIZJE trzecimi ogniwami od korpusu roznymi nogami
	if (meshModel[SEGMENT_I1]->collision(meshModel[SEGMENT_I2])) {
		collision_table[12]=true; collision_table[13]=true;
	}
	if (meshModel[SEGMENT_I2]->collision(meshModel[SEGMENT_I3])) {
		collision_table[13]=true; collision_table[14]=true;
	}
	if (meshModel[SEGMENT_I4]->collision(meshModel[SEGMENT_I5])) {
		collision_table[15]=true; collision_table[16]=true;
	}
	if (meshModel[SEGMENT_I5]->collision(meshModel[SEGMENT_I6])) {
		collision_table[16]=true; collision_table[17]=true;
	}
	//=========KOLIZJE drugimi, a trzecimi ogniwami od korpusu miedzy roznymi nogami
	if (meshModel[SEGMENT_I1]->collision(meshModel[SEGMENT_II2])) {
		collision_table[7]=true; collision_table[12]=true;
	}
	if (meshModel[SEGMENT_I2]->collision(meshModel[SEGMENT_II1])) {
		collision_table[6]=true; collision_table[13]=true;
	}
	if (meshModel[SEGMENT_I2]->collision(meshModel[SEGMENT_II3])) {
		collision_table[8]=true; collision_table[13]=true;
	}
	if (meshModel[SEGMENT_I3]->collision(meshModel[SEGMENT_II2])) {
		collision_table[7]=true; collision_table[14]=true;
	}
	if (meshModel[SEGMENT_I4]->collision(meshModel[SEGMENT_II5])) {
		collision_table[10]=true; collision_table[15]=true;
	}
	if (meshModel[SEGMENT_I5]->collision(meshModel[SEGMENT_II4])) {
		collision_table[9]=true; collision_table[16]=true;
	}
	if (meshModel[SEGMENT_I5]->collision(meshModel[SEGMENT_II6])) {
		collision_table[11]=true; collision_table[16]=true;
	}
	if (meshModel[SEGMENT_I6]->collision(meshModel[SEGMENT_II5])) {
		collision_table[10]=true; collision_table[17]=true;
	}
	for (int i=0;i<44;i++){
		if (collision_table[i]==true) 
			return true;
	}*/
	return false;
}

robsim::RobotStructure* robsim::createStarlETHRobotStructure(void) {
	starlETHStructure.reset(new StarlETHStructure());
	return starlETHStructure.get();
}