#include "MessorStructure.h"
#include "objects3DS.h"

/// A single instance of MessorStructure
MessorStructure::Ptr messorStructure;

MessorStructure::MessorStructure(void) : RobotStructure("Messor Robot Structure")
{
	for (int i=0;i<18;i++)
		angles[i]=0;
	angles[0]=45*3.14/180;
	angles[1]=-45*3.14/180;
	robot_model.ObjLoad("resources/Messor_model/korpus_dol.3ds");
	robot_model.ObjLoad("resources/Messor_model/korpus_gora.3ds");
	robot_model.ObjLoad("resources/Messor_model/noga_blacha_1.3ds"); //zawias (ceownik)
	robot_model.ObjLoad("resources/Messor_model/noga_blacha_2.3ds"); //³¹cznik
	robot_model.ObjLoad("resources/Messor_model/noga_1_3ds_poprzeczka.3ds");//udo
	robot_model.ObjLoad("resources/Messor_model/noga_3ds_poprzeczka.3ds");//goleñ
	robot_model.ObjLoad("resources/Messor_model/podstawka.3ds");
	robot_model.ObjLoad("resources/Messor_model/stopka.3ds");
	
	for (int i=0;i<26;i++) {
		CollisionModel3D* tmp = newCollisionModel3D();
		meshModel.push_back(tmp);
	}
//	InitializeTerrain();
	CollisionModels();	// Init Collision Models
//	robot_model.TerrainCollisionModels();	// Init Collision Models
	init_structures();
}

MessorStructure::~MessorStructure(void)
{
}

void MessorStructure::initCollisionModel(uint_fast8_t objectNo, CollisionModel3D& model) {
	for (int j=0;j<robot_model.object[objectNo].polygons_qty;j++) {
		model.addTriangle(	robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].a ].x*0.254, robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].a ].y*0.254, robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].a ].z*0.254, 
							robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].b ].x*0.254, robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].b ].y*0.254,	robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].b ].z*0.254,
							robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].c ].x*0.254, robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].c ].y*0.254,	robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].c ].z*0.254);
	}
	model.finalize();
}

void MessorStructure::CollisionModels(void)
{
	initCollisionModel(0, *meshModel[PLATFORM_BOTTOM]); // korpus_dol_model
	initCollisionModel(1, *meshModel[PLATFORM_TOP]); // korpus_gora_model

	initCollisionModel(7, *meshModel[FOOT1]); // stopka_1_model
	initCollisionModel(7, *meshModel[FOOT2]); // stopka_2_model
	initCollisionModel(7, *meshModel[FOOT3]); // stopka_3_model
	initCollisionModel(7, *meshModel[FOOT4]); // stopka_4_model
	initCollisionModel(7, *meshModel[FOOT5]); // stopka_5_model
	initCollisionModel(7, *meshModel[FOOT6]); // stopka_6_model

	initCollisionModel(5, *meshModel[SEGMENT_I1]); // segment_I_model_1
	initCollisionModel(5, *meshModel[SEGMENT_I2]); // segment_I_model_2
	initCollisionModel(5, *meshModel[SEGMENT_I3]); // segment_I_model_3
	initCollisionModel(5, *meshModel[SEGMENT_I4]); // segment_I_model_4
	initCollisionModel(5, *meshModel[SEGMENT_I5]); // segment_I_model_5
	initCollisionModel(5, *meshModel[SEGMENT_I6]); // segment_I_model_6

	initCollisionModel(4, *meshModel[SEGMENT_II1]); // segment_II_model_1
	initCollisionModel(4, *meshModel[SEGMENT_II2]); // segment_II_model_2
	initCollisionModel(4, *meshModel[SEGMENT_II3]); // segment_II_model_3
	initCollisionModel(4, *meshModel[SEGMENT_II4]); // segment_II_model_4
	initCollisionModel(4, *meshModel[SEGMENT_II5]); // segment_II_model_5
	initCollisionModel(4, *meshModel[SEGMENT_II6]); // segment_II_model_6

	initCollisionModel(2, *meshModel[LINK_C1]); // przegub_typu_C_1
	initCollisionModel(2, *meshModel[LINK_C2]); // przegub_typu_C_2
	initCollisionModel(2, *meshModel[LINK_C3]); // przegub_typu_C_3
	initCollisionModel(2, *meshModel[LINK_C4]); // przegub_typu_C_4
	initCollisionModel(2, *meshModel[LINK_C5]); // przegub_typu_C_5
	initCollisionModel(2, *meshModel[LINK_C6]); // przegub_typu_C_6
}

void MessorStructure::init_structures(void)
{
	structPlatformBottom();
	structPlatformTop();
	structLegSheet1();
	structLegSheet2();
	structLegPart1();
	structLegPart2();
	structBase();
	structFoot();
}

void MessorStructure::structPlatformBottom(void)
{
	glNewList(GL_PLATFORM_BOTTOM, GL_COMPILE);
	glColor3f(0.5,0.5,0.5);
	robot_model.Object3DS(0);
	glEndList();
}

void MessorStructure::structPlatformTop(void)
{
	glNewList(GL_PLATFORM_TOP, GL_COMPILE);
	glColor3f(0.5,0.5,0.5);
	robot_model.Object3DS(1);
	glEndList();
}

void MessorStructure::structLegSheet1(void)
{
	glNewList(GL_LEG_SHEET1, GL_COMPILE);
	glColor3f(0.6,0.6,0.6);
	robot_model.Object3DS(2);
	glEndList();
}

void MessorStructure::structLegSheet2(void)
{
	glNewList(GL_LEG_SHEET2, GL_COMPILE);
	glColor3f(0.6,0.6,0.6);
	robot_model.Object3DS(3);
	glEndList();
}

void MessorStructure::structLegPart1(void)
{
	glNewList(GL_LEG_PART1, GL_COMPILE);
	glColor3f(0.6,0.6,0.6);
	robot_model.Object3DS(4);
	glEndList();
}

void MessorStructure::structLegPart2(void)
{
	glNewList(GL_LEG_PART2, GL_COMPILE);
	glColor3f(0.6,0.6,0.6);
	robot_model.Object3DS(5);
	glEndList();
}

void MessorStructure::structBase(void)
{
	glNewList(GL_BASE, GL_COMPILE);
	glColor3f(0.6,0.6,0.6);
	robot_model.Object3DS(6);
	glEndList();
}

void MessorStructure::structFoot(void)
{
	glNewList(GL_FOOT, GL_COMPILE);
	glColor3f(0.6,0.6,0.6);
	robot_model.Object3DS(7);
	glEndList();
	
}

void MessorStructure::drawCoordinateSystem(void)
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

void MessorStructure::copyTable(CPunctum * src, float * dest) const{
	for (int i=0;i<4;i++){
		for (int j=0;j<4;j++){
			dest[i+4*j]=src->getElement(i+1,j+1);
		}
	}
}

void MessorStructure::Leg3(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const {
	float przegub_c3[16];
	CPunctum m_noga1,tmp;
	m_noga1 = (*m_noga)*tmp.makeTransformMatrix("gamma", Qn_1*3.14/180);
	copyTable(&m_noga1,przegub_c3);
	meshModel[22]->setTransform (przegub_c3);

	float czlon1_m3[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, czlon1_m3);
	CPunctum m_noga2;
	m_noga2 = m_noga1 * tmp.makeTransformMatrix("alpha", -3.14/2)*tmp.makeTransformMatrix("x", -2.15*0.254)*tmp.makeTransformMatrix("y", -0.49*0.254)*tmp.makeTransformMatrix("z", 0.92*0.254)*tmp.makeTransformMatrix("gamma", Qn_2*3.14/180);
	copyTable(&m_noga2,czlon1_m3);
	meshModel[16]->setTransform (czlon1_m3);

	float segment3[16];
	CPunctum m_noga3;
	m_noga3 = m_noga2 * tmp.makeTransformMatrix("x", -6.28*0.254)*tmp.makeTransformMatrix("z", -1.69*0.254)*tmp.makeTransformMatrix("gamma", Qn_3*3.14/180);
	copyTable(&m_noga3,segment3);
	meshModel[10]->setTransform (segment3);

	CPunctum m_noga4;
	m_noga4 = m_noga3 * tmp.makeTransformMatrix("x", -7.9*0.254)*tmp.makeTransformMatrix("y", -0.03*0.254)*tmp.makeTransformMatrix("z", 0.76*0.254)*tmp.makeTransformMatrix("beta", 90*3.14/180)*tmp.makeTransformMatrix("alpha", -28.6*3.14/180)*tmp.makeTransformMatrix("z", -0.45*0.254);
	float stopka_3[16];
	copyTable(&m_noga4,stopka_3);
	meshModel[4]->setTransform (stopka_3);
}

void MessorStructure::Leg4(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const {
	float przegub_c4[16];
	CPunctum m_noga1, tmp;
	m_noga1 = (*m_noga)*tmp.makeTransformMatrix("gamma", Qn_1*3.14/180);
	copyTable(&m_noga1,przegub_c4);
	meshModel[23]->setTransform (przegub_c4);
								
	float czlon1_m4[16];
	CPunctum m_noga2;
	m_noga2 = m_noga1 * tmp.makeTransformMatrix("alpha", -3.14/2)*tmp.makeTransformMatrix("x", -2.15*0.254)*tmp.makeTransformMatrix("y", -0.49*0.254)*tmp.makeTransformMatrix("z", 0.92*0.254)*tmp.makeTransformMatrix("gamma", Qn_2*3.14/180);
	copyTable(&m_noga2,czlon1_m4);
	meshModel[17]->setTransform (czlon1_m4);

	float segment4[16];
	CPunctum m_noga3;
	m_noga3 = m_noga2 * tmp.makeTransformMatrix("x", -6.28*0.254)*tmp.makeTransformMatrix("z", -1.69*0.254)*tmp.makeTransformMatrix("gamma", Qn_3*3.14/180);
	copyTable(&m_noga3,segment4);
	meshModel[11]->setTransform (segment4);

	float stopka_4[16];
	CPunctum m_noga4;
	m_noga4 = m_noga3 * tmp.makeTransformMatrix("x", -7.9*0.254)*tmp.makeTransformMatrix("y", -0.03*0.254)*tmp.makeTransformMatrix("z", 0.8*0.254)*tmp.makeTransformMatrix("beta", 90*3.14/180)*tmp.makeTransformMatrix("alpha", -28.6*3.14/180)*tmp.makeTransformMatrix("z", -0.45*0.254);
	copyTable(&m_noga4,stopka_4);
	meshModel[5]->setTransform (stopka_4);
}

void MessorStructure::Leg2(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const {
	float przegub_c2[16];
	CPunctum m_noga1,tmp;
	m_noga1 = (*m_noga)*tmp.makeTransformMatrix("gamma", Qn_1*3.14/180);
	copyTable(&m_noga1,przegub_c2);
	meshModel[21]->setTransform (przegub_c2);

	float czlon1_m2[16];
	CPunctum m_noga2;
	m_noga2 = m_noga1 * tmp.makeTransformMatrix("alpha", -3.14/2)*tmp.makeTransformMatrix("x", -2.15*0.254)*tmp.makeTransformMatrix("y", -0.49*0.254)*tmp.makeTransformMatrix("z", 0.92*0.254)*tmp.makeTransformMatrix("gamma", Qn_2*3.14/180);
	copyTable(&m_noga2,czlon1_m2);
	meshModel[15]->setTransform (czlon1_m2);
										
	float segment2[16];
	CPunctum m_noga3;
	m_noga3 = m_noga2 * tmp.makeTransformMatrix("x", -6.28*0.254)*tmp.makeTransformMatrix("z", -1.69*0.254)*tmp.makeTransformMatrix("gamma", Qn_3*3.14/180);
	copyTable(&m_noga3,segment2);
	meshModel[9]->setTransform (segment2);

	float stopka_2[16];
	CPunctum m_noga4;
	m_noga4 = m_noga3 * tmp.makeTransformMatrix("x", -7.9*0.254)*tmp.makeTransformMatrix("y", -0.03*0.254)*tmp.makeTransformMatrix("z", 0.8*0.254)*tmp.makeTransformMatrix("beta", 90*3.14/180)*tmp.makeTransformMatrix("alpha", -28.6*3.14/180)*tmp.makeTransformMatrix("z", -0.45*0.254);
	copyTable(&m_noga4,stopka_2);
	meshModel[3]->setTransform (stopka_2);
}

void MessorStructure::Leg5(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const {
	float przegub_c5[16];
	CPunctum m_noga1,tmp;
	m_noga1 = (*m_noga)*tmp.makeTransformMatrix("gamma", Qn_1*3.14/180);
	copyTable(&m_noga1,przegub_c5);
	meshModel[24]->setTransform (przegub_c5);
					
	float czlon1_m5[16];
	CPunctum m_noga2;
	m_noga2 = m_noga1 * tmp.makeTransformMatrix("alpha", -3.14/2)*tmp.makeTransformMatrix("x", -2.15*0.254)*tmp.makeTransformMatrix("y", -0.49*0.254)*tmp.makeTransformMatrix("z", 0.92*0.254)*tmp.makeTransformMatrix("gamma", Qn_2*3.14/180);
	copyTable(&m_noga2,czlon1_m5);
	meshModel[18]->setTransform (czlon1_m5);
										
	float segment5[16];
	CPunctum m_noga3;
	m_noga3 = m_noga2 * tmp.makeTransformMatrix("x", -6.28*0.254)*tmp.makeTransformMatrix("z", -1.69*0.254)*tmp.makeTransformMatrix("gamma", Qn_3*3.14/180);
	copyTable(&m_noga3,segment5);
	meshModel[12]->setTransform (segment5);
	
	float stopka_5[16];
	CPunctum m_noga4;
	m_noga4 = m_noga3 * tmp.makeTransformMatrix("x", -7.9*0.254)*tmp.makeTransformMatrix("y", -0.03*0.254)*tmp.makeTransformMatrix("z", 0.8*0.254)*tmp.makeTransformMatrix("beta", 90*3.14/180)*tmp.makeTransformMatrix("alpha", -28.6*3.14/180)*tmp.makeTransformMatrix("z", -0.45*0.254);
	copyTable(&m_noga4,stopka_5);
	meshModel[6]->setTransform (stopka_5);
}

void MessorStructure::Leg1(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const {
	float przegub_c1[16];
	CPunctum m_noga1,tmp;
	m_noga1 = (*m_noga)*tmp.makeTransformMatrix("gamma", Qn_1*3.14/180);
	copyTable(&m_noga1,przegub_c1);
	meshModel[20]->setTransform (przegub_c1);
					
	float czlon1_m1[16];
	CPunctum m_noga2;
	m_noga2 = m_noga1 * tmp.makeTransformMatrix("alpha", -3.14/2)*tmp.makeTransformMatrix("x", -2.15*0.254)*tmp.makeTransformMatrix("y", -0.49*0.254)*tmp.makeTransformMatrix("z", 0.92*0.254)*tmp.makeTransformMatrix("gamma", Qn_2*3.14/180);
	copyTable(&m_noga2,czlon1_m1);
	meshModel[14]->setTransform (czlon1_m1);
										
	float segment1[16];
	CPunctum m_noga3;
	m_noga3 = m_noga2 * tmp.makeTransformMatrix("x", -6.28*0.254)*tmp.makeTransformMatrix("z", -1.69*0.254)*tmp.makeTransformMatrix("gamma", Qn_3*3.14/180);
	copyTable(&m_noga3,segment1);
	meshModel[8]->setTransform (segment1);
	
	float stopka_1[16];
	CPunctum m_noga4;
	m_noga4 = m_noga3 * tmp.makeTransformMatrix("x", -7.9*0.254)*tmp.makeTransformMatrix("y", -0.03*0.254)*tmp.makeTransformMatrix("z", 0.8*0.254)*tmp.makeTransformMatrix("beta", 90*3.14/180)*tmp.makeTransformMatrix("alpha", -28.6*3.14/180)*tmp.makeTransformMatrix("z", -0.45*0.254);
	copyTable(&m_noga4,stopka_1);
	meshModel[2]->setTransform (stopka_1);
}

void MessorStructure::Leg6(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const {
	float przegub_c6[16];
	CPunctum m_noga1,tmp;
	m_noga1 = (*m_noga)*tmp.makeTransformMatrix("gamma", Qn_1*3.14/180);
	copyTable(&m_noga1,przegub_c6);
	meshModel[25]->setTransform (przegub_c6);
					
	float czlon1_m6[16];
	CPunctum m_noga2;
	m_noga2 = m_noga1 * tmp.makeTransformMatrix("alpha", -3.14/2)*tmp.makeTransformMatrix("x", -2.15*0.254)*tmp.makeTransformMatrix("y", -0.49*0.254)*tmp.makeTransformMatrix("z", 0.92*0.254)*tmp.makeTransformMatrix("gamma", Qn_2*3.14/180);
	copyTable(&m_noga2,czlon1_m6);
	meshModel[19]->setTransform (czlon1_m6);
										
	float segment6[16];
	CPunctum m_noga3;
	m_noga3 = m_noga2 * tmp.makeTransformMatrix("x", -6.28*0.254)*tmp.makeTransformMatrix("z", -1.69*0.254)*tmp.makeTransformMatrix("gamma", Qn_3*3.14/180);
	copyTable(&m_noga3,segment6);
	meshModel[13]->setTransform (segment6);
	
	float stopka_6[16];
	CPunctum m_noga4;
	m_noga4 = m_noga3 * tmp.makeTransformMatrix("x", -7.9*0.254)*tmp.makeTransformMatrix("y", -0.03*0.254)*tmp.makeTransformMatrix("z", 0.8*0.254)*tmp.makeTransformMatrix("beta", 90*3.14/180)*tmp.makeTransformMatrix("alpha", -28.6*3.14/180)*tmp.makeTransformMatrix("z", -0.45*0.254);
	copyTable(&m_noga4,stopka_6);
	meshModel[7]->setTransform (stopka_6);
}

void MessorStructure::GLLeg3(float Qn_1, float Qn_2, float Qn_3) const {
	glRotatef(Qn_1,0,0,1);
	glPushMatrix();
		glCallList(GL_LEG_SHEET1);
		glTranslatef(-2.14*0.254,0,0.49*0.254);
		glRotatef(90,1,0,0);
		glRotatef(180,0,1,0);
		glRotatef(90,0,0,1);
		glCallList(GL_LEG_SHEET2);
	glPopMatrix();
	
	glRotatef(-90,1,0,0);
	glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
	glRotatef(0.0,0,0,1);
	glRotatef(Qn_2,0,0,1);		
	glPushMatrix();
		glCallList(GL_LEG_PART1);
		glTranslatef(0,0,-1.91*0.254);
		glTranslatef(-6.28*0.254,0,0.22*0.254);
		glRotatef(0.0,0,0,1);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			glCallList(GL_LEG_PART2);
			glTranslatef(0,0,1.57*0.254);
			glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
			glRotatef(90,0,1,0);
			glRotatef(-28.6,1,0,0);
			glCallList(GL_BASE);
			glTranslatef(0,0,-0.45*0.254);
			glCallList(GL_FOOT);
		glPopMatrix();
	glPopMatrix();
}

void MessorStructure::GLLeg4(float Qn_1, float Qn_2, float Qn_3) const {
	glRotatef(Qn_1,0,0,1);
	glPushMatrix();
		glCallList(GL_LEG_SHEET1);
		glTranslatef(-2.14*0.254,0,0.49*0.254);
		glRotatef(90,1,0,0);
		glRotatef(180,0,1,0);
		glRotatef(90,0,0,1);
		glCallList(GL_LEG_SHEET2);
	glPopMatrix();
	
	glRotatef(-90,1,0,0);
	glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
	glRotatef(Qn_2,0,0,1);
	glPushMatrix();
		glCallList(GL_LEG_PART1);
		glTranslatef(0,0,-1.91*0.254);
		glTranslatef(-6.28*0.254,0,0.22*0.254);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			glCallList(GL_LEG_PART2);
			glTranslatef(0,0,1.61*0.254);
			glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
			glRotatef(90,0,1,0);
			glRotatef(-28.6,1,0,0);
			glCallList(GL_BASE);
			glTranslatef(0,0,-0.45*0.254);
			glCallList(GL_FOOT);
		glPopMatrix();
	glPopMatrix();
}

void MessorStructure::GLLeg2(float Qn_1, float Qn_2, float Qn_3) const {
	glRotatef(Qn_1,0,0,1);
	glPushMatrix();
		glCallList(GL_LEG_SHEET1);
		glTranslatef(-2.14*0.254,0,0.49*0.254);
		glRotatef(90,1,0,0);
		glRotatef(180,0,1,0);
		glRotatef(90,0,0,1);
		glCallList(GL_LEG_SHEET2);
	glPopMatrix();

	glRotatef(-90,1,0,0);
	glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
	glRotatef(Qn_2,0,0,1);
	glPushMatrix();
		glCallList(GL_LEG_PART1);
		glTranslatef(0,0,-1.91*0.254);
		glTranslatef(-6.28*0.254,0,0.22*0.254);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			glCallList(GL_LEG_PART2);
			glTranslatef(0,0,1.61*0.254);	
			glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
			glRotatef(90,0,1,0);
			glRotatef(-28.6,1,0,0);
			glCallList(GL_BASE);
			glTranslatef(0,0,-0.45*0.254);
			glCallList(GL_FOOT);
		glPopMatrix();
	glPopMatrix();
}

void MessorStructure::GLLeg5(float Qn_1, float Qn_2, float Qn_3) const {
	glRotatef(Qn_1,0,0,1);
	glPushMatrix();
		glCallList(GL_LEG_SHEET1);
		glTranslatef(-2.14*0.254,0,0.49*0.254);
		glRotatef(90,1,0,0);
		glRotatef(180,0,1,0);
		glRotatef(90,0,0,1);
		glCallList(GL_LEG_SHEET2);
	glPopMatrix();
	
	glRotatef(-90,1,0,0);
	glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
	glRotatef(Qn_2,0,0,1);
	glPushMatrix();
		glCallList(GL_LEG_PART1);
		glTranslatef(0,0,-1.91*0.254);
		glTranslatef(-6.28*0.254,0,0.22*0.254);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			glCallList(GL_LEG_PART2);
			glTranslatef(0,0,1.61*0.254);	
			glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
			glRotatef(90,0,1,0);
			glRotatef(-28.6,1,0,0);
			glCallList(GL_BASE);
			glTranslatef(0,0,-0.45*0.254);
			glCallList(GL_FOOT);
		glPopMatrix();
	glPopMatrix();
}

void MessorStructure::GLLeg1(float Qn_1, float Qn_2, float Qn_3) const {
	glRotatef(Qn_1,0,0,1);
	glPushMatrix();
		glCallList(GL_LEG_SHEET1);
		glTranslatef(-2.14*0.254,0,0.49*0.254);
		glRotatef(90,1,0,0);
		glRotatef(180,0,1,0);
		glRotatef(90,0,0,1);
		glCallList(GL_LEG_SHEET2);
	glPopMatrix();

	glRotatef(-90,1,0,0);
	glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
	glRotatef(Qn_2,0,0,1);
	glPushMatrix();
		glCallList(GL_LEG_PART1);
		glTranslatef(0,0,-1.91*0.254);					
		glTranslatef(-6.28*0.254,0,0.22*0.254);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			glCallList(GL_LEG_PART2);
			glTranslatef(0,0,1.61*0.254);
			glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
			glRotatef(90,0,1,0);
			glRotatef(-28.6,1,0,0);
			glCallList(GL_BASE);
			glTranslatef(0,0,-0.45*0.254);
			glCallList(GL_FOOT);
		glPopMatrix();
	glPopMatrix();
}

void MessorStructure::GLLeg6(float Qn_1, float Qn_2, float Qn_3) const {
	glRotatef(Qn_1,0,0,1);
	glPushMatrix();
		glCallList(GL_LEG_SHEET1);
		glTranslatef(-2.14*0.254,0,0.49*0.254);
		glRotatef(90,1,0,0);
		glRotatef(180,0,1,0);
		glRotatef(90,0,0,1);
		glCallList(GL_LEG_SHEET2);
	glPopMatrix();

	glRotatef(-90,1,0,0);
	glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
	glRotatef(Qn_2,0,0,1);
	glPushMatrix();
		glCallList(GL_LEG_PART1);
		glTranslatef(0,0,-1.91*0.254);
		glTranslatef(-6.28*0.254,0,0.22*0.254);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			glCallList(GL_LEG_PART2);
			glTranslatef(0,0,1.61*0.254);
			glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
			glRotatef(90,0,1,0);
			glRotatef(-28.6,1,0,0);
			glCallList(GL_BASE);
			glTranslatef(0,0,-0.45*0.254);
			glCallList(GL_FOOT);
		glPopMatrix();
	glPopMatrix();
}

void MessorStructure::DrawRobot(robsim::float_type* pos, robsim::float_type* rot, robsim::float_type * angles) const
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
		
//===============NOGA_3=================================

	CPunctum m_noga;
	m_noga = m4*tmp.makeTransformMatrix("x", -2.56*0.254)*tmp.makeTransformMatrix("y", -6.06*0.254)*tmp.makeTransformMatrix("z", 3.33*0.254);
	Leg3(-angles[6]*180/3.14,-angles[7]*180/3.14,-angles[8]*180/3.14,&m_noga); 

//===============NOGA_4=================================
	m_noga = m4*tmp.makeTransformMatrix("x", 2.56*0.254)*tmp.makeTransformMatrix("y", -6.06*0.254)*tmp.makeTransformMatrix("z", 3.33*0.254)*tmp.makeTransformMatrix("gamma", 3.14);
	Leg4(angles[9]*180/3.14,-angles[10]*180/3.14,-angles[11]*180/3.14,&m_noga);	

//===============NOGA_2=================================				
	m_noga = m4*tmp.makeTransformMatrix("x", -5.1*0.254)*tmp.makeTransformMatrix("z", 3.33*0.254);
	Leg2(-angles[3]*180/3.14,-angles[4]*180/3.14,-angles[5]*180/3.14,&m_noga); 

//===============NOGA_5=================================
	m_noga = m4*tmp.makeTransformMatrix("x", 5.1*0.254)*tmp.makeTransformMatrix("z", 3.33*0.254)*tmp.makeTransformMatrix("gamma", 3.14);
	Leg5(angles[12]*180/3.14,-angles[13]*180/3.14,-angles[14]*180/3.14, &m_noga);	

//===============NOGA_1=================================
	m_noga = m4*tmp.makeTransformMatrix("x", -2.56*0.254)*tmp.makeTransformMatrix("y", 6.06*0.254)*tmp.makeTransformMatrix("z", 3.33*0.254);
	Leg1(-angles[0]*180/3.14,-angles[1]*180/3.14,-angles[2]*180/3.14,&m_noga);	

//===============NOGA_6=================================
	m_noga = m4*tmp.makeTransformMatrix("x", 2.56*0.254)*tmp.makeTransformMatrix("y", 6.06*0.254)*tmp.makeTransformMatrix("z", 3.33*0.254)*tmp.makeTransformMatrix("gamma", 3.14);
	Leg6(angles[15]*180/3.14,-angles[16]*180/3.14,-angles[17]*180/3.14, &m_noga);	
}

void MessorStructure::GLDrawRobot(robsim::float_type *pos, robsim::float_type * rot, std::vector<robsim::float_type> config) const {

	float GLmat[16]={rot[0], rot[4], rot[8], 0, rot[1], rot[5], rot[9], 0, rot[2], rot[6], rot[10], 0, pos[0], pos[1], pos[2], 1}; //macierz do przeksztalcen

	glPushMatrix();
		glMultMatrixf(GLmat);
		glRotatef(90,1,0,0);
		glRotatef(180,0,0,1);
		glPushMatrix();
			glCallList(GL_PLATFORM_BOTTOM);
		glPopMatrix();
		glTranslatef(0.0f,0.0f,-4.24*0.254);
		glPushMatrix();
			glCallList(GL_PLATFORM_TOP);
		glPopMatrix();
					
//===============LEG_3=================================
		glPushMatrix();
			glTranslatef(-2.56*0.254,-6.06*0.254,3.33*0.254);
			GLLeg3(-config[6]*180/3.14,-config[7]*180/3.14,-config[8]*180/3.14); 
		glPopMatrix();

//===============LEG_4=================================
		glPushMatrix();
			glTranslatef(2.56*0.254,-6.06*0.254,3.33*0.254);
			glRotatef(180,0,0,1);
			GLLeg4(config[9]*180/3.14,-config[10]*180/3.14,-config[11]*180/3.14);	
		glPopMatrix();

//===============LEG_1=================================				
		glPushMatrix();
			glTranslatef(-5.1*0.254,0.0,3.33*0.254);
			GLLeg2(-config[3]*180/3.14,-config[4]*180/3.14,-config[5]*180/3.14); 
		glPopMatrix();

//===============LEG_5=================================
		glPushMatrix();
			glTranslatef(5.1*0.254,0,3.33*0.254);
			glRotatef(180,0,0,1);
			GLLeg5(config[12]*180/3.14,-config[13]*180/3.14,-config[14]*180/3.14);	
		glPopMatrix();

//===============LEG_1=================================
		glPushMatrix();
			glTranslatef(-2.56*0.254,6.06*0.254,3.33*0.254);
			GLLeg1(-config[0]*180/3.14,-config[1]*180/3.14,-config[2]*180/3.14);	
		glPopMatrix();

//===============LEG_6=================================
		glPushMatrix();
			glTranslatef(2.56*0.254,6.06*0.254,3.33*0.254);
			glRotatef(180,0,0,1);
			GLLeg6(config[15]*180/3.14,-config[16]*180/3.14,-config[17]*180/3.14);	
		glPopMatrix();
	glPopMatrix();
}

bool MessorStructure::checkCollision(robsim::float_type* pos, robsim::float_type* rot, robsim::float_type * angles, bool * collision_table) const {

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
	//uproszczony sposob
	for (int i=0;i<6;i++) {
		if ((angles[i*3+1]>(24*3.14/180+1.1))){
			collision_table[i]=true;
			collision_table[i+6]=true;
		}
	}
	for (int i=0;i<6;i++) {
		if (abs(angles[i*3])>1.57){
			collision_table[i]=true;
		}
	}
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
	if (meshModel[SEGMENT_II1]->collision(meshModel[SEGMENT_II2])) {
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
	}
	return false;
}

robsim::RobotStructure* robsim::createMessorRobotStructure(void) {
	messorStructure.reset(new MessorStructure());
	return messorStructure.get();
}