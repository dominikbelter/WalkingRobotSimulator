#include "RobotStructure.h"
#include "objects3DS.h"


CRobotStructure::CRobotStructure(void)
{
	for (int i=0;i<18;i++)
		angles[i]=0;
	angles[0]=45*3.14/180;
	angles[1]=-45*3.14/180;
	robot_model.ObjLoad("model/korpus_dol.3ds");
	robot_model.ObjLoad("model/korpus_gora.3ds");
	robot_model.ObjLoad("model/noga_blacha_1.3ds"); //zawias (ceownik)
	robot_model.ObjLoad("model/noga_blacha_2.3ds"); //³¹cznik
	robot_model.ObjLoad("model/noga_1_3ds_poprzeczka.3ds");//udo
	robot_model.ObjLoad("model/noga_3ds_poprzeczka.3ds");//goleñ
	robot_model.ObjLoad("model/podstawka.3ds");
	robot_model.ObjLoad("model/stopka.3ds");
	
//	InitializeTerrain();
	robot_model.CollisionModels();	// Init Collision Models
	robot_model.TerrainCollisionModels();	// Init Collision Models
}

CRobotStructure::~CRobotStructure(void)
{
}

void CRobotStructure::init_structures(void)
{
	struct_korpus_dol();
	struct_korpus_gora();
	struct_noga_blacha_1();
	struct_noga_blacha_2();
	struct_noga_czlon_1();
	struct_noga_czlon_2a();
	struct_podstawka();
	struct_stopka();
}

void CRobotStructure::struct_korpus_dol(void)
{
	glNewList(korpus_dol, GL_COMPILE);
	glColor3f(0.5,0.5,0.5);
	robot_model.Object3DS(0);
	glEndList();
}

void CRobotStructure::struct_korpus_gora(void)
{
	glNewList(korpus_gora, GL_COMPILE);
	glColor3f(0.5,0.5,0.5);
	robot_model.Object3DS(1);
	glEndList();
}

void CRobotStructure::struct_noga_blacha_1(void)
{
	glNewList(noga_blacha_1, GL_COMPILE);
	glColor3f(0.6,0.6,0.6);
	robot_model.Object3DS(2);
	glEndList();
}

void CRobotStructure::struct_noga_blacha_2(void)
{
	glNewList(noga_blacha_2, GL_COMPILE);
	glColor3f(0.6,0.6,0.6);
	robot_model.Object3DS(3);
	glEndList();
}

void CRobotStructure::struct_noga_czlon_1(void)
{
	glNewList(noga_czlon_1, GL_COMPILE);
	glColor3f(0.6,0.6,0.6);
	robot_model.Object3DS(4);
	glEndList();
}

void CRobotStructure::struct_noga_czlon_2a(void)
{
	glNewList(noga_czlon_2a, GL_COMPILE);
	glColor3f(0.6,0.6,0.6);
	robot_model.Object3DS(5);
	glEndList();
}

void CRobotStructure::struct_podstawka(void)
{
	glNewList(podstawka, GL_COMPILE);
	glColor3f(0.6,0.6,0.6);
	robot_model.Object3DS(6);
	glEndList();
}

void CRobotStructure::struct_stopka(void)
{
	glNewList(stopka, GL_COMPILE);
	glColor3f(0.6,0.6,0.6);
	robot_model.Object3DS(7);
	glEndList();
	
}

void CRobotStructure::CoordinateSystem_Local(void)
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

/*!\brief makes 4x4 transform matrix
  */
Matrix4f CRobotStructure::makeTransformMatrix(const char * type, float value){
  Matrix4f tmp = Matrix4f::Identity();
  float x,y,z;
  if (!strcmp(type,"x")){
     tmp(0,3)=value;
  }
  else if (!strcmp(type,"y")){
     tmp(1,3)=value;
  }
  else if (!strcmp(type,"z")){
     tmp(2,3)=value;
  }
  else if (!strcmp(type,"alpha")){
	  x=1;
	  y=0;
	  z=0;
     /*tmp(1,1)=cos(value);
     tmp(1,2)=-sin(value);
     tmp(2,1)=sin(value);
     tmp(2,2)=cos(value);*/
	  tmp(0,0)=x*x*(1-cos(value))+cos(value);
	  tmp(0,1)=x*y*(1-cos(value))-z*sin(value);
	  tmp(0,2)=x*z*(1-cos(value))+y*sin(value);

	  tmp(1,0)=y*x*(1-cos(value))+z*sin(value);
	  tmp(1,1)=y*y*(1-cos(value))+cos(value);
	  tmp(1,2)=y*z*(1-cos(value))-x*sin(value);

	  tmp(2,0)=x*z*(1-cos(value))-y*sin(value);
	  tmp(2,1)=y*z*(1-cos(value))+x*sin(value);
	  tmp(2,2)=z*z*(1-cos(value))+cos(value);
  }
  else if (!strcmp(type,"beta")){
	  y=1; z=0; x=0;
     /*tmp(0,0)=cos(value);
     tmp(0,2)=sin(value);
     tmp(2,0)=-sin(value);
     tmp(2,2)=cos(value);*/
	  tmp(0,0)=x*x*(1-cos(value))+cos(value);
	  tmp(0,1)=x*y*(1-cos(value))-z*sin(value);
	  tmp(0,2)=x*z*(1-cos(value))+y*sin(value);

	  tmp(1,0)=y*x*(1-cos(value))+z*sin(value);
	  tmp(1,1)=y*y*(1-cos(value))+cos(value);
	  tmp(1,2)=y*z*(1-cos(value))-x*sin(value);

	  tmp(2,0)=x*z*(1-cos(value))-y*sin(value);
	  tmp(2,1)=y*z*(1-cos(value))+x*sin(value);
	  tmp(2,2)=z*z*(1-cos(value))+cos(value);
  }
  else if (!strcmp(type,"gamma")){
	  z=1; x=0; y=0;
     /*tmp(0,0)=cos(value);
     tmp(0,1)=-sin(value);
     tmp(1,0)=sin(value);
     tmp(1,1)=cos(value);*/
	  tmp(0,0)=x*x*(1-cos(value))+cos(value);
	  tmp(0,1)=x*y*(1-cos(value))-z*sin(value);
	  tmp(0,2)=x*z*(1-cos(value))+y*sin(value);

	  tmp(1,0)=y*x*(1-cos(value))+z*sin(value);
	  tmp(1,1)=y*y*(1-cos(value))+cos(value);
	  tmp(1,2)=y*z*(1-cos(value))-x*sin(value);

	  tmp(2,0)=x*z*(1-cos(value))-y*sin(value);
	  tmp(2,1)=y*z*(1-cos(value))+x*sin(value);
	  tmp(2,2)=z*z*(1-cos(value))+cos(value);
  }
  return tmp;
}

void CRobotStructure::copyTable(Matrix4f * src, float * dest){
	for (int i=0;i<4;i++){
		for (int j=0;j<4;j++){
			dest[i+4*j]=(*src)(i,j);
		}
	}
}

void CRobotStructure::Noga_3(float Qn_1, float Qn_2, float Qn_3, Matrix4f * m_noga)
{
//=======================================================================		
//float test[16];
//				glRotatef(Qn_1,0,0,1);
			
//					glPushMatrix();
		//				CoordinateSystem_Local();
//						glCallList(noga_blacha_1);
									float przegub_c3[16];
									Matrix4f m_noga1 = (*m_noga)*makeTransformMatrix("gamma", Qn_1*3.14/180);
									//glGetFloatv(GL_MODELVIEW_MATRIX, przegub_c3);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									copyTable(&m_noga1,przegub_c3);
									robot_model.przegub_typu_C_3->setTransform (przegub_c3);
//						glTranslatef(-2.14*0.254,0,0.49*0.254);
//						glRotatef(90,1,0,0);
//						glRotatef(180,0,1,0);
//						glRotatef(90,0,0,1);
		//				CoordinateSystem_Local();
//						glCallList(noga_blacha_2);
//					glPopMatrix();
//=======================================================================		
//				glRotatef(-90,1,0,0);
//				glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
//				glRotatef(Qn_2,0,0,1);
			
//					glPushMatrix();
	//					CoordinateSystem_Local();
//						glCallList(noga_czlon_1);
									float czlon1_m3[16];
									glGetFloatv(GL_MODELVIEW_MATRIX, czlon1_m3);
									Matrix4f m_noga2 = m_noga1 * makeTransformMatrix("alpha", -3.14/2)*makeTransformMatrix("x", -2.15*0.254)*makeTransformMatrix("y", -0.49*0.254)*makeTransformMatrix("z", 0.92*0.254)*makeTransformMatrix("gamma", Qn_2*3.14/180);
									copyTable(&m_noga2,czlon1_m3);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.segment_II_model_3->setTransform (czlon1_m3);
						
//=======================================================================					
//						glTranslatef(0,0,-1.91*0.254);
//						glTranslatef(-6.28*0.254,0,0.22*0.254);
//						glRotatef(Qn_3,0,0,1);
						
//							glPushMatrix();
	//							CoordinateSystem_Local();
//								glCallList(noga_czlon_2a);
									float segment3[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, segment3);
									Matrix4f m_noga3 = m_noga2 * makeTransformMatrix("x", -6.28*0.254)*makeTransformMatrix("z", -1.69*0.254)*makeTransformMatrix("gamma", Qn_3*3.14/180);
									copyTable(&m_noga3,segment3);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.segment_I_model_3->setTransform (segment3);

//								glTranslatef(0,0,1.57*0.254);
	//							CoordinateSystem_Local();
												
//								glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
//								glRotatef(90,0,1,0);
//								glRotatef(-28.6,1,0,0);

	//							CoordinateSystem_Local();
//								glCallList(podstawka);
										
//								glTranslatef(0,0,-0.45*0.254);
	//							CoordinateSystem_Local();
//								glCallList(stopka);
								Matrix4f m_noga4 = m_noga3 * makeTransformMatrix("x", -7.9*0.254)*makeTransformMatrix("y", -0.03*0.254)*makeTransformMatrix("z", 0.76*0.254)*makeTransformMatrix("beta", 90*3.14/180)*makeTransformMatrix("alpha", -28.6*3.14/180)*makeTransformMatrix("z", -0.45*0.254);
									float stopka_3[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, stopka_3);
									copyTable(&m_noga4,stopka_3);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.stopka_3_model->setTransform (stopka_3);
							
//								glPopMatrix();
							
//=======================================================================			
//					glPopMatrix();
}

void CRobotStructure::Noga_4(float Qn_1, float Qn_2, float Qn_3, Matrix4f * m_noga)
{
//=======================================================================		
//float test[16];
//				glRotatef(Qn_1,0,0,1);
			
//					glPushMatrix();
	//					CoordinateSystem_Local();
//						glCallList(noga_blacha_1);
						float przegub_c4[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, przegub_c4);
									Matrix4f m_noga1 = (*m_noga)*makeTransformMatrix("gamma", Qn_1*3.14/180);
									copyTable(&m_noga1,przegub_c4);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.przegub_typu_C_4->setTransform (przegub_c4);
			
//						glTranslatef(-2.14*0.254,0,0.49*0.254);
//						glRotatef(90,1,0,0);
//						glRotatef(180,0,1,0);
//						glRotatef(90,0,0,1);
		//				CoordinateSystem_Local();
//						glCallList(noga_blacha_2);
//					glPopMatrix();
//=======================================================================		
//				glRotatef(-90,1,0,0);
//				glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
//				glRotatef(Qn_2,0,0,1);
			
//					glPushMatrix();
//						CoordinateSystem_Local();
//						glCallList(noga_czlon_1);
					
						float czlon1_m4[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, czlon1_m4);
									Matrix4f m_noga2 = m_noga1 * makeTransformMatrix("alpha", -3.14/2)*makeTransformMatrix("x", -2.15*0.254)*makeTransformMatrix("y", -0.49*0.254)*makeTransformMatrix("z", 0.92*0.254)*makeTransformMatrix("gamma", Qn_2*3.14/180);
									copyTable(&m_noga2,czlon1_m4);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.segment_II_model_4->setTransform (czlon1_m4);
//						glTranslatef(0,0,-1.91*0.254);
						//=======================================================================					
//						glTranslatef(-6.28*0.254,0,0.22*0.254);
//						glRotatef(Qn_3,0,0,1);
						
//							glPushMatrix();
		//						CoordinateSystem_Local();
//								glCallList(noga_czlon_2a);
									float segment4[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, segment4);
									Matrix4f m_noga3 = m_noga2 * makeTransformMatrix("x", -6.28*0.254)*makeTransformMatrix("z", -1.69*0.254)*makeTransformMatrix("gamma", Qn_3*3.14/180);
									copyTable(&m_noga3,segment4);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.segment_I_model_4->setTransform (segment4);

//								glTranslatef(0,0,1.61*0.254);
	//							CoordinateSystem_Local();
												
//								glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
//								glRotatef(90,0,1,0);
//								glRotatef(-28.6,1,0,0);
							
			//					CoordinateSystem_Local();
//								glCallList(podstawka);
										
//								glTranslatef(0,0,-0.45*0.254);
			//					CoordinateSystem_Local();
//								glCallList(stopka);
								float stopka_4[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, stopka_4);
									Matrix4f m_noga4 = m_noga3 * makeTransformMatrix("x", -7.9*0.254)*makeTransformMatrix("y", -0.03*0.254)*makeTransformMatrix("z", 0.8*0.254)*makeTransformMatrix("beta", 90*3.14/180)*makeTransformMatrix("alpha", -28.6*3.14/180)*makeTransformMatrix("z", -0.45*0.254);
									copyTable(&m_noga4,stopka_4);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.stopka_4_model->setTransform (stopka_4);
							
//								glPopMatrix();
							
//=======================================================================			
//					glPopMatrix();
}

void CRobotStructure::Noga_2(float Qn_1, float Qn_2, float Qn_3, Matrix4f * m_noga)
{
//=======================================================================		
//float test[16];
//				glRotatef(Qn_1,0,0,1);
			
//					glPushMatrix();
		//				CoordinateSystem_Local();
//						glCallList(noga_blacha_1);
						float przegub_c2[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, przegub_c2);
									Matrix4f m_noga1 = (*m_noga)*makeTransformMatrix("gamma", Qn_1*3.14/180);
									copyTable(&m_noga1,przegub_c2);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.przegub_typu_C_2->setTransform (przegub_c2);
			
//						glTranslatef(-2.14*0.254,0,0.49*0.254);
//						glRotatef(90,1,0,0);
//						glRotatef(180,0,1,0);
//						glRotatef(90,0,0,1);
					
		//				CoordinateSystem_Local();
//						glCallList(noga_blacha_2);
//					glPopMatrix();
//=======================================================================		
//				glRotatef(-90,1,0,0);
//				glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
//				glRotatef(Qn_2,0,0,1);
			
//					glPushMatrix();
	//					CoordinateSystem_Local();
//						glCallList(noga_czlon_1);
					
						float czlon1_m2[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, czlon1_m2);
									Matrix4f m_noga2 = m_noga1 * makeTransformMatrix("alpha", -3.14/2)*makeTransformMatrix("x", -2.15*0.254)*makeTransformMatrix("y", -0.49*0.254)*makeTransformMatrix("z", 0.92*0.254)*makeTransformMatrix("gamma", Qn_2*3.14/180);
									copyTable(&m_noga2,czlon1_m2);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.segment_II_model_2->setTransform (czlon1_m2);
//						glTranslatef(0,0,-1.91*0.254);
						
									
//=======================================================================					
//						glTranslatef(-6.28*0.254,0,0.22*0.254);
//						glRotatef(Qn_3,0,0,1);
						
//							glPushMatrix();
							
	//							CoordinateSystem_Local();
//								glCallList(noga_czlon_2a);
										
								float segment2[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, segment2);
									Matrix4f m_noga3 = m_noga2 * makeTransformMatrix("x", -6.28*0.254)*makeTransformMatrix("z", -1.69*0.254)*makeTransformMatrix("gamma", Qn_3*3.14/180);
									copyTable(&m_noga3,segment2);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.segment_I_model_2->setTransform (segment2);

//								glTranslatef(0,0,1.61*0.254);	
					
//								glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
//								glRotatef(90,0,1,0);
//								glRotatef(-28.6,1,0,0);
								
	//							CoordinateSystem_Local();
//								glCallList(podstawka);
										
//								glTranslatef(0,0,-0.45*0.254);
	//							CoordinateSystem_Local();
//								glCallList(stopka);
								float stopka_2[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, stopka_2);
									Matrix4f m_noga4 = m_noga3 * makeTransformMatrix("x", -7.9*0.254)*makeTransformMatrix("y", -0.03*0.254)*makeTransformMatrix("z", 0.8*0.254)*makeTransformMatrix("beta", 90*3.14/180)*makeTransformMatrix("alpha", -28.6*3.14/180)*makeTransformMatrix("z", -0.45*0.254);
									copyTable(&m_noga4,stopka_2);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.stopka_2_model->setTransform (stopka_2);
							
//								glPopMatrix();
							
//=======================================================================			
//					glPopMatrix();
}

void CRobotStructure::Noga_5(float Qn_1, float Qn_2, float Qn_3, Matrix4f * m_noga)
{
//=======================================================================		
//float test[16];
//				glRotatef(Qn_1,0,0,1);
			
//					glPushMatrix();
//						CoordinateSystem_Local();
//						glCallList(noga_blacha_1);
						float przegub_c5[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, przegub_c5);
									Matrix4f m_noga1 = (*m_noga)*makeTransformMatrix("gamma", Qn_1*3.14/180);
									copyTable(&m_noga1,przegub_c5);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.przegub_typu_C_5->setTransform (przegub_c5);
			
//						glTranslatef(-2.14*0.254,0,0.49*0.254);
//						glRotatef(90,1,0,0);
//						glRotatef(180,0,1,0);
//						glRotatef(90,0,0,1);
					
	//					CoordinateSystem_Local();
//						glCallList(noga_blacha_2);
//					glPopMatrix();
//=======================================================================		
//				glRotatef(-90,1,0,0);
//				glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
//				glRotatef(Qn_2,0,0,1);
			
//					glPushMatrix();
					
	//					CoordinateSystem_Local();
//						glCallList(noga_czlon_1);
					
									float czlon1_m5[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, czlon1_m5);
									Matrix4f m_noga2 = m_noga1 * makeTransformMatrix("alpha", -3.14/2)*makeTransformMatrix("x", -2.15*0.254)*makeTransformMatrix("y", -0.49*0.254)*makeTransformMatrix("z", 0.92*0.254)*makeTransformMatrix("gamma", Qn_2*3.14/180);
									copyTable(&m_noga2,czlon1_m5);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.segment_II_model_5->setTransform (czlon1_m5);
//						glTranslatef(0,0,-1.91*0.254);
						
									
//=======================================================================					
//						glTranslatef(-6.28*0.254,0,0.22*0.254);
//						glRotatef(Qn_3,0,0,1);
						
//							glPushMatrix();
							
	//							CoordinateSystem_Local();
//								glCallList(noga_czlon_2a);
										
								float segment5[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, segment5);
									Matrix4f m_noga3 = m_noga2 * makeTransformMatrix("x", -6.28*0.254)*makeTransformMatrix("z", -1.69*0.254)*makeTransformMatrix("gamma", Qn_3*3.14/180);
									copyTable(&m_noga3,segment5);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.segment_I_model_5->setTransform (segment5);

//								glTranslatef(0,0,1.61*0.254);
									
//								glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
//								glRotatef(90,0,1,0);
//								glRotatef(-28.6,1,0,0);

		//						CoordinateSystem_Local();
//								glCallList(podstawka);
										
//								glTranslatef(0,0,-0.45*0.254);
	//							CoordinateSystem_Local();
//								glCallList(stopka);
								float stopka_5[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, stopka_5);
									Matrix4f m_noga4 = m_noga3 * makeTransformMatrix("x", -7.9*0.254)*makeTransformMatrix("y", -0.03*0.254)*makeTransformMatrix("z", 0.8*0.254)*makeTransformMatrix("beta", 90*3.14/180)*makeTransformMatrix("alpha", -28.6*3.14/180)*makeTransformMatrix("z", -0.45*0.254);
									copyTable(&m_noga4,stopka_5);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.stopka_5_model->setTransform (stopka_5);
							
//								glPopMatrix();
							
//=======================================================================			
//					glPopMatrix();
}

void CRobotStructure::Noga_1(float Qn_1, float Qn_2, float Qn_3, Matrix4f * m_noga)
{
//=======================================================================		
//float test[16];
//				glRotatef(Qn_1,0,0,1);
			
//					glPushMatrix();
					
		//				CoordinateSystem_Local();
//						glCallList(noga_blacha_1);
						float przegub_c1[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, przegub_c1);
									Matrix4f m_noga1 = (*m_noga)*makeTransformMatrix("gamma", Qn_1*3.14/180);
									copyTable(&m_noga1,przegub_c1);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.przegub_typu_C_1->setTransform (przegub_c1);
			
//						glTranslatef(-2.14*0.254,0,0.49*0.254);
//						glRotatef(90,1,0,0);
//						glRotatef(180,0,1,0);
//						glRotatef(90,0,0,1);
					
	//					CoordinateSystem_Local();
//						glCallList(noga_blacha_2);
//					glPopMatrix();
//=======================================================================		
//				glRotatef(-90,1,0,0);
//				glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
//				glRotatef(Qn_2,0,0,1);
			
//					glPushMatrix();
					
	//					CoordinateSystem_Local();
//						glCallList(noga_czlon_1);
					
						float czlon1_m1[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, czlon1_m1);
									Matrix4f m_noga2 = m_noga1 * makeTransformMatrix("alpha", -3.14/2)*makeTransformMatrix("x", -2.15*0.254)*makeTransformMatrix("y", -0.49*0.254)*makeTransformMatrix("z", 0.92*0.254)*makeTransformMatrix("gamma", Qn_2*3.14/180);
									copyTable(&m_noga2,czlon1_m1);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.segment_II_model_1->setTransform (czlon1_m1);
//						glTranslatef(0,0,-1.91*0.254);
						
									
//=======================================================================					
//						glTranslatef(-6.28*0.254,0,0.22*0.254);
//						glRotatef(Qn_3,0,0,1);
						
//							glPushMatrix();
							
			//					CoordinateSystem_Local();
//								glCallList(noga_czlon_2a);
										
								float segment1[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, segment1);
									Matrix4f m_noga3 = m_noga2 * makeTransformMatrix("x", -6.28*0.254)*makeTransformMatrix("z", -1.69*0.254)*makeTransformMatrix("gamma", Qn_3*3.14/180);
									copyTable(&m_noga3,segment1);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.segment_I_model_1->setTransform (segment1);

//								glTranslatef(0,0,1.61*0.254);
					
//								glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
//								glRotatef(90,0,1,0);
//								glRotatef(-28.6,1,0,0);

	//							CoordinateSystem_Local();
//								glCallList(podstawka);
										
//								glTranslatef(0,0,-0.45*0.254);
	//							CoordinateSystem_Local();
//								glCallList(stopka);
								float stopka_1[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, stopka_1);
									Matrix4f m_noga4 = m_noga3 * makeTransformMatrix("x", -7.9*0.254)*makeTransformMatrix("y", -0.03*0.254)*makeTransformMatrix("z", 0.8*0.254)*makeTransformMatrix("beta", 90*3.14/180)*makeTransformMatrix("alpha", -28.6*3.14/180)*makeTransformMatrix("z", -0.45*0.254);
									copyTable(&m_noga4,stopka_1);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.stopka_1_model->setTransform (stopka_1);
							
//								glPopMatrix();
							
//=======================================================================			
//					glPopMatrix();
}

void CRobotStructure::Noga_6(float Qn_1, float Qn_2, float Qn_3, Matrix4f * m_noga)
{
//=======================================================================		
//float test[16];
//				glRotatef(Qn_1,0,0,1);
			
//					glPushMatrix();
					
//						CoordinateSystem_Local();
//						glCallList(noga_blacha_1);
						float przegub_c6[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, przegub_c6);
									Matrix4f m_noga1 = (*m_noga)*makeTransformMatrix("gamma", Qn_1*3.14/180);
									copyTable(&m_noga1,przegub_c6);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.przegub_typu_C_6->setTransform (przegub_c6);
			
//						glTranslatef(-2.14*0.254,0,0.49*0.254);
//						glRotatef(90,1,0,0);
//						glRotatef(180,0,1,0);
//						glRotatef(90,0,0,1);
					
//						CoordinateSystem_Local();
//						glCallList(noga_blacha_2);
//					glPopMatrix();
//=======================================================================		
//				glRotatef(-90,1,0,0);
//				glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
//				glRotatef(Qn_2,0,0,1);
			
//					glPushMatrix();
					
		//				CoordinateSystem_Local();
//						glCallList(noga_czlon_1);
					
						float czlon1_m6[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, czlon1_m6);
									Matrix4f m_noga2 = m_noga1 * makeTransformMatrix("alpha", -3.14/2)*makeTransformMatrix("x", -2.15*0.254)*makeTransformMatrix("y", -0.49*0.254)*makeTransformMatrix("z", 0.92*0.254)*makeTransformMatrix("gamma", Qn_2*3.14/180);
									copyTable(&m_noga2,czlon1_m6);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.segment_II_model_6->setTransform (czlon1_m6);
//						glTranslatef(0,0,-1.91*0.254);
						
									
//=======================================================================					
//						glTranslatef(-6.28*0.254,0,0.22*0.254);
//						glRotatef(Qn_3,0,0,1);
						
//							glPushMatrix();
							
			//					CoordinateSystem_Local();
//								glCallList(noga_czlon_2a);
										
								float segment6[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, segment6);
									Matrix4f m_noga3 = m_noga2 * makeTransformMatrix("x", -6.28*0.254)*makeTransformMatrix("z", -1.69*0.254)*makeTransformMatrix("gamma", Qn_3*3.14/180);
									copyTable(&m_noga3,segment6);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.segment_I_model_6->setTransform (segment6);

//								glTranslatef(0,0,1.61*0.254);
								
//								glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
//								glRotatef(90,0,1,0);
//								glRotatef(-28.6,1,0,0);
								
		//						CoordinateSystem_Local();
//								glCallList(podstawka);
										
//								glTranslatef(0,0,-0.45*0.254);
		//						CoordinateSystem_Local();
//								glCallList(stopka);
								float stopka_6[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, stopka_6);
									Matrix4f m_noga4 = m_noga3 * makeTransformMatrix("x", -7.9*0.254)*makeTransformMatrix("y", -0.03*0.254)*makeTransformMatrix("z", 0.8*0.254)*makeTransformMatrix("beta", 90*3.14/180)*makeTransformMatrix("alpha", -28.6*3.14/180)*makeTransformMatrix("z", -0.45*0.254);
									copyTable(&m_noga4,stopka_6);
//glGetFloatv(GL_MODELVIEW_MATRIX, test);
									robot_model.stopka_6_model->setTransform (stopka_6);
							
//								glPopMatrix();
							
//=======================================================================			
//					glPopMatrix();
}

void CRobotStructure::GLNoga_3(float Qn_1, float Qn_2, float Qn_3)
{
//=======================================================================		
				
				glRotatef(Qn_1,0,0,1);
			
					glPushMatrix();
					//	CoordinateSystem_Local();
						glCallList(noga_blacha_1);
	//								float przegub_c3[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, przegub_c3);
	//								Matrix4f m_noga1 = (*m_noga)*makeTransformMatrix("gamma", Qn_1*3.14/180);
	//								copyTable(&m_noga1,przegub_c3);
	//								robot_model.przegub_typu_C_3->setTransform (przegub_c3);
						glTranslatef(-2.14*0.254,0,0.49*0.254);
						glRotatef(90,1,0,0);
						glRotatef(180,0,1,0);
						glRotatef(90,0,0,1);
					//	CoordinateSystem_Local();
						glCallList(noga_blacha_2);
					glPopMatrix();
//=======================================================================		
				glRotatef(-90,1,0,0);
				glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
				glRotatef(0.0,0,0,1);
				glRotatef(Qn_2,0,0,1);
			
					glPushMatrix();
						//CoordinateSystem_Local();
						glCallList(noga_czlon_1);
			//						float czlon1_m3[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, czlon1_m3);
			//						Matrix4f m_noga2 = m_noga1 * makeTransformMatrix("alpha", -3.14/2)*makeTransformMatrix("x", -2.15)*makeTransformMatrix("y", -0.49)*makeTransformMatrix("z", 0.92)*makeTransformMatrix("gamma", -32*3.14/180)*makeTransformMatrix("gamma", Qn_2*3.14/180);
			//						copyTable(&m_noga2,czlon1_m3);
			//						robot_model.segment_II_model_3->setTransform (czlon1_m3);
						
//=======================================================================					
						glTranslatef(0,0,-1.91*0.254);
						glTranslatef(-6.28*0.254,0,0.22*0.254);
						glRotatef(0.0,0,0,1);
						glRotatef(Qn_3,0,0,1);
						
							glPushMatrix();
						//		CoordinateSystem_Local();
								glCallList(noga_czlon_2a);
									float segment3[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, segment3);
				//					Matrix4f m_noga3 = m_noga2 * makeTransformMatrix("x", -6.28)*makeTransformMatrix("y", 0)*makeTransformMatrix("z", -1.91+0.22)*makeTransformMatrix("gamma", 92*3.14/180)*makeTransformMatrix("gamma", Qn_3*3.14/180);
				//					copyTable(&m_noga3,segment3);
				//					robot_model.segment_I_model_3->setTransform (segment3);

								glTranslatef(0,0,1.57*0.254);
				//				CoordinateSystem_Local();
												
								glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
								glRotatef(90,0,1,0);
								glRotatef(-28.6,1,0,0);

				//				CoordinateSystem_Local();
								glCallList(podstawka);
										
								glTranslatef(0,0,-0.45*0.254);
				//				CoordinateSystem_Local();
								glCallList(stopka);
//								Matrix4f m_noga4 = m_noga3 * makeTransformMatrix("x", -7.9)*makeTransformMatrix("y", -0.03)*makeTransformMatrix("z", 1.57-0.81)*makeTransformMatrix("beta", 90*3.14/180)*makeTransformMatrix("alpha", -28.6*3.14/180)*makeTransformMatrix("z", -0.45);
//									float stopka_3[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, stopka_3);
//									copyTable(&m_noga4,stopka_3);
//									robot_model.stopka_3_model->setTransform (stopka_3);
							
								glPopMatrix();
							
//=======================================================================			
					glPopMatrix();
}

void CRobotStructure::GLNoga_4(float Qn_1, float Qn_2, float Qn_3)
{
//=======================================================================		
				
				glRotatef(Qn_1,0,0,1);
			
					glPushMatrix();
	//					CoordinateSystem_Local();
						glCallList(noga_blacha_1);
						float przegub_c4[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, przegub_c4);
	//								Matrix4f m_noga1 = (*m_noga)*makeTransformMatrix("gamma", Qn_1*3.14/180);
	//								copyTable(&m_noga1,przegub_c4);
	//								robot_model.przegub_typu_C_4->setTransform (przegub_c4);
			
						glTranslatef(-2.14*0.254,0,0.49*0.254);
						glRotatef(90,1,0,0);
						glRotatef(180,0,1,0);
						glRotatef(90,0,0,1);
		//				CoordinateSystem_Local();
						glCallList(noga_blacha_2);
					glPopMatrix();
//=======================================================================		
				glRotatef(-90,1,0,0);
				glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
				//glRotatef(-32.0,0,0,1);
				glRotatef(Qn_2,0,0,1);
			
					glPushMatrix();
//						CoordinateSystem_Local();
						glCallList(noga_czlon_1);
					
						float czlon1_m4[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, czlon1_m4);
	//								Matrix4f m_noga2 = m_noga1 * makeTransformMatrix("alpha", -3.14/2)*makeTransformMatrix("x", -2.15)*makeTransformMatrix("y", -0.49)*makeTransformMatrix("z", 0.92)*makeTransformMatrix("gamma", -32*3.14/180)*makeTransformMatrix("gamma", Qn_2*3.14/180);
	//								copyTable(&m_noga2,czlon1_m4);
	//								robot_model.segment_II_model_4->setTransform (czlon1_m4);
						glTranslatef(0,0,-1.91*0.254);
						//=======================================================================					
						glTranslatef(-6.28*0.254,0,0.22*0.254);
						//glRotatef(92.0,0,0,1);
						glRotatef(Qn_3,0,0,1);
						
							glPushMatrix();
		//						CoordinateSystem_Local();
								glCallList(noga_czlon_2a);
									float segment4[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, segment4);
			//						Matrix4f m_noga3 = m_noga2 * makeTransformMatrix("x", -6.28)*makeTransformMatrix("y", 0)*makeTransformMatrix("z", -1.91+0.22)*makeTransformMatrix("gamma", 92*3.14/180)*makeTransformMatrix("gamma", Qn_3*3.14/180);
			//						copyTable(&m_noga3,segment4);
			//						robot_model.segment_I_model_4->setTransform (segment4);

								glTranslatef(0,0,1.61*0.254);
	//							CoordinateSystem_Local();
												
								glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
								glRotatef(90,0,1,0);
								glRotatef(-28.6,1,0,0);
							
			//					CoordinateSystem_Local();
								glCallList(podstawka);
										
								glTranslatef(0,0,-0.45*0.254);
			//					CoordinateSystem_Local();
								glCallList(stopka);
								float stopka_4[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, stopka_4);
			//						Matrix4f m_noga4 = m_noga3 * makeTransformMatrix("x", -7.9)*makeTransformMatrix("y", -0.03)*makeTransformMatrix("z", 1.61-0.81)*makeTransformMatrix("beta", 90*3.14/180)*makeTransformMatrix("alpha", -28.6*3.14/180)*makeTransformMatrix("z", -0.45);
			//						copyTable(&m_noga4,stopka_4);
			//						robot_model.stopka_4_model->setTransform (stopka_4);
							
								glPopMatrix();
							
//=======================================================================			
					glPopMatrix();
}

void CRobotStructure::GLNoga_2(float Qn_1, float Qn_2, float Qn_3)
{
//=======================================================================		
				
				glRotatef(Qn_1,0,0,1);
			
					glPushMatrix();
		//				CoordinateSystem_Local();
						glCallList(noga_blacha_1);
						float przegub_c2[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, przegub_c2);
	//								Matrix4f m_noga1 = (*m_noga)*makeTransformMatrix("gamma", Qn_1*3.14/180);
	//								copyTable(&m_noga1,przegub_c2);
	//								robot_model.przegub_typu_C_2->setTransform (przegub_c2);
			
						glTranslatef(-2.14*0.254,0,0.49*0.254);
						glRotatef(90,1,0,0);
						glRotatef(180,0,1,0);
						glRotatef(90,0,0,1);
					
		//				CoordinateSystem_Local();
						glCallList(noga_blacha_2);
					glPopMatrix();
//=======================================================================		
				glRotatef(-90,1,0,0);
				glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
				//glRotatef(-32.0,0,0,1);
				glRotatef(Qn_2,0,0,1);
			
					glPushMatrix();
	//					CoordinateSystem_Local();
						glCallList(noga_czlon_1);
					
	//					float czlon1_m2[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, czlon1_m2);
	//								Matrix4f m_noga2 = m_noga1 * makeTransformMatrix("alpha", -3.14/2)*makeTransformMatrix("x", -2.15)*makeTransformMatrix("y", -0.49)*makeTransformMatrix("z", 0.92)*makeTransformMatrix("gamma", -32*3.14/180)*makeTransformMatrix("gamma", Qn_2*3.14/180);
	//								copyTable(&m_noga2,czlon1_m2);
	//								robot_model.segment_II_model_2->setTransform (czlon1_m2);
						glTranslatef(0,0,-1.91*0.254);
						
									
//=======================================================================					
						glTranslatef(-6.28*0.254,0,0.22*0.254);
						//glRotatef(92.0,0,0,1);
						glRotatef(Qn_3,0,0,1);
						
							glPushMatrix();
							
	//							CoordinateSystem_Local();
								glCallList(noga_czlon_2a);
										
	//							float segment2[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, segment2);
	//								Matrix4f m_noga3 = m_noga2 * makeTransformMatrix("x", -6.28)*makeTransformMatrix("y", 0)*makeTransformMatrix("z", -1.91+0.22)*makeTransformMatrix("gamma", 92*3.14/180)*makeTransformMatrix("gamma", Qn_3*3.14/180);
	//								copyTable(&m_noga3,segment2);
	//								robot_model.segment_I_model_2->setTransform (segment2);

								glTranslatef(0,0,1.61*0.254);	
					
								glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
								glRotatef(90,0,1,0);
								glRotatef(-28.6,1,0,0);
								
	//							CoordinateSystem_Local();
								glCallList(podstawka);
										
								glTranslatef(0,0,-0.45*0.254);
	//							CoordinateSystem_Local();
								glCallList(stopka);
	//							float stopka_2[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, stopka_2);
	//								Matrix4f m_noga4 = m_noga3 * makeTransformMatrix("x", -7.9)*makeTransformMatrix("y", -0.03)*makeTransformMatrix("z", 1.61-0.81)*makeTransformMatrix("beta", 90*3.14/180)*makeTransformMatrix("alpha", -28.6*3.14/180)*makeTransformMatrix("z", -0.45);
	//								copyTable(&m_noga4,stopka_2);
	//								robot_model.stopka_2_model->setTransform (stopka_2);
							
								glPopMatrix();
							
//=======================================================================			
					glPopMatrix();
}

void CRobotStructure::GLNoga_5(float Qn_1, float Qn_2, float Qn_3)
{
//=======================================================================		
				
				glRotatef(Qn_1,0,0,1);
			
					glPushMatrix();
//						CoordinateSystem_Local();
						glCallList(noga_blacha_1);
//						float przegub_c5[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, przegub_c5);
//									Matrix4f m_noga1 = (*m_noga)*makeTransformMatrix("gamma", Qn_1*3.14/180);
//									copyTable(&m_noga1,przegub_c5);
//									robot_model.przegub_typu_C_5->setTransform (przegub_c5);
			
						glTranslatef(-2.14*0.254,0,0.49*0.254);
						glRotatef(90,1,0,0);
						glRotatef(180,0,1,0);
						glRotatef(90,0,0,1);
					
	//					CoordinateSystem_Local();
						glCallList(noga_blacha_2);
					glPopMatrix();
//=======================================================================		
				glRotatef(-90,1,0,0);
				glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
				//glRotatef(-32.0,0,0,1);
				glRotatef(Qn_2,0,0,1);
			
					glPushMatrix();
					
	//					CoordinateSystem_Local();
						glCallList(noga_czlon_1);
					
	//								float czlon1_m5[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, czlon1_m5);
	//								Matrix4f m_noga2 = m_noga1 * makeTransformMatrix("alpha", -3.14/2)*makeTransformMatrix("x", -2.15)*makeTransformMatrix("y", -0.49)*makeTransformMatrix("z", 0.92)*makeTransformMatrix("gamma", -32*3.14/180)*makeTransformMatrix("gamma", Qn_2*3.14/180);
	//								copyTable(&m_noga2,czlon1_m5);
	//								robot_model.segment_II_model_5->setTransform (czlon1_m5);
						glTranslatef(0,0,-1.91*0.254);
						
									
//=======================================================================					
						glTranslatef(-6.28*0.254,0,0.22*0.254);
						//glRotatef(92.0,0,0,1);
						glRotatef(Qn_3,0,0,1);
						
							glPushMatrix();
							
	//							CoordinateSystem_Local();
								glCallList(noga_czlon_2a);
										
	//							float segment5[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, segment5);
	//								Matrix4f m_noga3 = m_noga2 * makeTransformMatrix("x", -6.28)*makeTransformMatrix("y", 0)*makeTransformMatrix("z", -1.91+0.22)*makeTransformMatrix("gamma", 92*3.14/180)*makeTransformMatrix("gamma", Qn_3*3.14/180);
	//								copyTable(&m_noga3,segment5);
	//								robot_model.segment_I_model_5->setTransform (segment5);

								glTranslatef(0,0,1.61*0.254);	
									
								glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
								glRotatef(90,0,1,0);
								glRotatef(-28.6,1,0,0);

		//						CoordinateSystem_Local();
								glCallList(podstawka);
										
								glTranslatef(0,0,-0.45*0.254);
	//							CoordinateSystem_Local();
								glCallList(stopka);
	//							float stopka_5[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, stopka_5);
	//								Matrix4f m_noga4 = m_noga3 * makeTransformMatrix("x", -7.9)*makeTransformMatrix("y", -0.03)*makeTransformMatrix("z", 1.61-0.81)*makeTransformMatrix("beta", 90*3.14/180)*makeTransformMatrix("alpha", -28.6*3.14/180)*makeTransformMatrix("z", -0.45);
	//								copyTable(&m_noga4,stopka_5);
	//								robot_model.stopka_5_model->setTransform (stopka_5);
							
								glPopMatrix();
							
//=======================================================================			
					glPopMatrix();
}

void CRobotStructure::GLNoga_1(float Qn_1, float Qn_2, float Qn_3)
{
//=======================================================================		
				
				glRotatef(Qn_1,0,0,1);
			
					glPushMatrix();
					
		//				CoordinateSystem_Local();
						glCallList(noga_blacha_1);
		//				float przegub_c1[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, przegub_c1);
		//							Matrix4f m_noga1 = (*m_noga)*makeTransformMatrix("gamma", Qn_1*3.14/180);
		//							copyTable(&m_noga1,przegub_c1);
		//							robot_model.przegub_typu_C_1->setTransform (przegub_c1);
			
						glTranslatef(-2.14*0.254,0,0.49*0.254);
						glRotatef(90,1,0,0);
						glRotatef(180,0,1,0);
						glRotatef(90,0,0,1);
					
	//					CoordinateSystem_Local();
						glCallList(noga_blacha_2);
					glPopMatrix();
//=======================================================================		
				glRotatef(-90,1,0,0);
				glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
				//glRotatef(-32.0,0,0,1);
				glRotatef(Qn_2,0,0,1);
			
					glPushMatrix();
					
	//					CoordinateSystem_Local();
						glCallList(noga_czlon_1);
					
			//			float czlon1_m1[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, czlon1_m1);
		//							Matrix4f m_noga2 = m_noga1 * makeTransformMatrix("alpha", -3.14/2)*makeTransformMatrix("x", -2.15)*makeTransformMatrix("y", -0.49)*makeTransformMatrix("z", 0.92)*makeTransformMatrix("gamma", -32*3.14/180)*makeTransformMatrix("gamma", Qn_2*3.14/180);
		//							copyTable(&m_noga2,czlon1_m1);
		//							robot_model.segment_II_model_1->setTransform (czlon1_m1);
						glTranslatef(0,0,-1.91*0.254);
						
									
//=======================================================================					
						glTranslatef(-6.28*0.254,0,0.22*0.254);
						//glRotatef(92.0,0,0,1);
						glRotatef(Qn_3,0,0,1);
						
							glPushMatrix();
							
			//					CoordinateSystem_Local();
								glCallList(noga_czlon_2a);
										
			//					float segment1[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, segment1);
			//						Matrix4f m_noga3 = m_noga2 * makeTransformMatrix("x", -6.28)*makeTransformMatrix("y", 0)*makeTransformMatrix("z", -1.91+0.22)*makeTransformMatrix("gamma", 92*3.14/180)*makeTransformMatrix("gamma", Qn_3*3.14/180);
			//						copyTable(&m_noga3,segment1);
			//						robot_model.segment_I_model_1->setTransform (segment1);

								glTranslatef(0,0,1.61*0.254);
					
								glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
								glRotatef(90,0,1,0);
								glRotatef(-28.6,1,0,0);

	//							CoordinateSystem_Local();
								glCallList(podstawka);
										
								glTranslatef(0,0,-0.45*0.254);
						//		CoordinateSystem_Local();
								glCallList(stopka);
	//							float stopka_1[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, stopka_1);
	//								Matrix4f m_noga4 = m_noga3 * makeTransformMatrix("x", -7.9)*makeTransformMatrix("y", -0.03)*makeTransformMatrix("z", 1.61-0.81)*makeTransformMatrix("beta", 90*3.14/180)*makeTransformMatrix("alpha", -28.6*3.14/180)*makeTransformMatrix("z", -0.45);
	//								copyTable(&m_noga4,stopka_1);
	//								robot_model.stopka_1_model->setTransform (stopka_1);
							
								glPopMatrix();
							
//=======================================================================			
					glPopMatrix();
}

void CRobotStructure::GLNoga_6(float Qn_1, float Qn_2, float Qn_3)
{
//=======================================================================		
				
				glRotatef(Qn_1,0,0,1);
			
					glPushMatrix();
					
//						CoordinateSystem_Local();
						glCallList(noga_blacha_1);
			//			float przegub_c6[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, przegub_c6);
			//						Matrix4f m_noga1 = (*m_noga)*makeTransformMatrix("gamma", Qn_1*3.14/180);
			//						copyTable(&m_noga1,przegub_c6);
			//						robot_model.przegub_typu_C_6->setTransform (przegub_c6);
			
						glTranslatef(-2.14*0.254,0,0.49*0.254);
						glRotatef(90,1,0,0);
						glRotatef(180,0,1,0);
						glRotatef(90,0,0,1);
					
//						CoordinateSystem_Local();
						glCallList(noga_blacha_2);
					glPopMatrix();
//=======================================================================		
				glRotatef(-90,1,0,0);
				glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
				//glRotatef(-32.0,0,0,1);
				glRotatef(Qn_2,0,0,1);
			
					glPushMatrix();
					
		//				CoordinateSystem_Local();
						glCallList(noga_czlon_1);
					
		//				float czlon1_m6[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, czlon1_m6);
		//							Matrix4f m_noga2 = m_noga1 * makeTransformMatrix("alpha", -3.14/2)*makeTransformMatrix("x", -2.15)*makeTransformMatrix("y", -0.49)*makeTransformMatrix("z", 0.92)*makeTransformMatrix("gamma", -32*3.14/180)*makeTransformMatrix("gamma", Qn_2*3.14/180);
		//							copyTable(&m_noga2,czlon1_m6);
		//							robot_model.segment_II_model_6->setTransform (czlon1_m6);
						glTranslatef(0,0,-1.91*0.254);
						
									
//=======================================================================					
						glTranslatef(-6.28*0.254,0,0.22*0.254);
						//glRotatef(92.0,0,0,1);
						glRotatef(Qn_3,0,0,1);
						
							glPushMatrix();
							
			//					CoordinateSystem_Local();
								glCallList(noga_czlon_2a);
										
		//						float segment6[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, segment6);
		//							Matrix4f m_noga3 = m_noga2 * makeTransformMatrix("x", -6.28)*makeTransformMatrix("y", 0)*makeTransformMatrix("z", -1.91+0.22)*makeTransformMatrix("gamma", 92*3.14/180)*makeTransformMatrix("gamma", Qn_3*3.14/180);
		//							copyTable(&m_noga3,segment6);
		//							robot_model.segment_I_model_6->setTransform (segment6);

								glTranslatef(0,0,1.61*0.254);
								
								glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
								glRotatef(90,0,1,0);
								glRotatef(-28.6,1,0,0);
								
		//						CoordinateSystem_Local();
								glCallList(podstawka);
										
								glTranslatef(0,0,-0.45*0.254);
		//						CoordinateSystem_Local();
								glCallList(stopka);
		//						float stopka_6[16];
									//glGetFloatv(GL_MODELVIEW_MATRIX, stopka_6);
		//							Matrix4f m_noga4 = m_noga3 * makeTransformMatrix("x", -7.9)*makeTransformMatrix("y", -0.03)*makeTransformMatrix("z", 1.61-0.81)*makeTransformMatrix("beta", 90*3.14/180)*makeTransformMatrix("alpha", -28.6*3.14/180)*makeTransformMatrix("z", -0.45);
		//							copyTable(&m_noga4,stopka_6);
		//							robot_model.stopka_6_model->setTransform (stopka_6);
							
								glPopMatrix();
							
//=======================================================================			
					glPopMatrix();
}

void CRobotStructure::DrawRobot(float x, float y, float z, float alpha, float beta, float gamma, float Q1,  float Q2,  float Q3,  float Q4,  float Q5,  float Q6, 
			   float Q7,  float Q8,  float Q9,  float Q10, float Q11, float Q12, 
			   float Q13, float Q14, float Q15, float Q16, float Q17, float Q18)
{
	//CoordinateSystem_World();

/*		glPushMatrix();
		glTranslatef(x*10,z*10-0.1,-y*10);
		glRotatef(90+alpha*180/3.14,1,0,0);
		glRotatef(-beta*180/3.14,0,1,0);
		glRotatef(180-gamma*180/3.14,0,0,1);
*/
		Matrix4f m4 = Matrix4f::Identity();
		m4=makeTransformMatrix("x", x*10)*makeTransformMatrix("y", z*10-0.1)*makeTransformMatrix("z", -y*10)*makeTransformMatrix("alpha", 3.14/2+alpha)*makeTransformMatrix("beta", -beta)*makeTransformMatrix("gamma", 3.14-gamma);
//			glPushMatrix();
	//			CoordinateSystem_Local();
//				glCallList(korpus_dol);
				float korpus_dol_m[16];
				//glGetFloatv(GL_MODELVIEW_MATRIX, korpus_dol_m);
				copyTable(&m4,korpus_dol_m);
				robot_model.korpus_dol_model->setTransform (korpus_dol_m);
//			glPopMatrix();
				
//			glTranslatef(0.0f,0.0f,-4.24*0.254);
				m4=m4*makeTransformMatrix("z", -4.24*0.254);
//			glPushMatrix();
//				glCallList(korpus_gora);
				float korpus_gora_m[16];
				//glGetFloatv(GL_MODELVIEW_MATRIX, korpus_gora_m);
				copyTable(&m4,korpus_gora_m);
				robot_model.korpus_gora_model->setTransform (korpus_gora_m);
//			glPopMatrix();

			
//===============NOGA_3=================================
//				glPushMatrix();
//				glTranslatef(-2.56*0.254,-6.06*0.254,3.33*0.254);
				Matrix4f m_noga = Matrix4f::Identity();
				m_noga = m4*makeTransformMatrix("x", -2.56*0.254)*makeTransformMatrix("y", -6.06*0.254)*makeTransformMatrix("z", 3.33*0.254);
				Noga_3(-Q7*180/3.14,-Q8*180/3.14,-Q9*180/3.14,&m_noga); 
//				glPopMatrix();

//===============NOGA_4=================================
//				glPushMatrix();
//				glTranslatef(2.56*0.254,-6.06*0.254,3.33*0.254);
//				glRotatef(180,0,0,1);
				m_noga = m4*makeTransformMatrix("x", 2.56*0.254)*makeTransformMatrix("y", -6.06*0.254)*makeTransformMatrix("z", 3.33*0.254)*makeTransformMatrix("gamma", 3.14);
				Noga_4(Q10*180/3.14,-Q11*180/3.14,-Q12*180/3.14,&m_noga);	
//				glPopMatrix();

//===============NOGA_2=================================				
//				glPushMatrix();
//				glTranslatef(-5.1*0.254,0.0,3.33*0.254);
				m_noga = m4*makeTransformMatrix("x", -5.1*0.254)*makeTransformMatrix("z", 3.33*0.254);
				Noga_2(-Q4*180/3.14,-Q5*180/3.14,-Q6*180/3.14,&m_noga); 
//				glPopMatrix();

//===============NOGA_5=================================
//				glPushMatrix();
//				glTranslatef(5.1*0.254,0,3.33*0.254);
//				glRotatef(180,0,0,1);
				m_noga = m4*makeTransformMatrix("x", 5.1*0.254)*makeTransformMatrix("z", 3.33*0.254)*makeTransformMatrix("gamma", 3.14);
				Noga_5(Q13*180/3.14,-Q14*180/3.14,-Q15*180/3.14, &m_noga);	
//				glPopMatrix();

//===============NOGA_1=================================
//				glPushMatrix();
//				glTranslatef(-2.56*0.254,6.06*0.254,3.33*0.254);
				m_noga = m4*makeTransformMatrix("x", -2.56*0.254)*makeTransformMatrix("y", 6.06*0.254)*makeTransformMatrix("z", 3.33*0.254);
				Noga_1(-Q1*180/3.14,-Q2*180/3.14,-Q3*180/3.14,&m_noga);	
//				glPopMatrix();

//===============NOGA_6=================================
//				glPushMatrix();
//				glTranslatef(2.56*0.254,6.06*0.254,3.33*0.254);
//				glRotatef(180,0,0,1);
				m_noga = m4*makeTransformMatrix("x", 2.56*0.254)*makeTransformMatrix("y", 6.06*0.254)*makeTransformMatrix("z", 3.33*0.254)*makeTransformMatrix("gamma", 3.14);
				Noga_6(Q16*180/3.14,-Q17*180/3.14,-Q18*180/3.14, &m_noga);	
//				glPopMatrix();

//	glPopMatrix();
}

void CRobotStructure::GLDrawRobot(float x, float y, float z, float alpha, float beta, float gamma, float Q1,  float Q2,  float Q3,  float Q4,  float Q5,  float Q6, 
			   float Q7,  float Q8,  float Q9,  float Q10, float Q11, float Q12, 
			   float Q13, float Q14, float Q15, float Q16, float Q17, float Q18)
{
	//CoordinateSystem_World();

		glPushMatrix();
		glTranslatef(x*10,z*10-0.1,-y*10);
		glRotatef(90+alpha*180/3.14,1,0,0);
		glRotatef(-beta*180/3.14,0,1,0);
		glRotatef(180-gamma*180/3.14,0,0,1);

		//Matrix4f m4 = Matrix4f::Identity();
		//m4=makeTransformMatrix("alpha", 3.14/2+alpha)*makeTransformMatrix("beta", -beta)*makeTransformMatrix("gamma", 3.14-gamma)*makeTransformMatrix("x", x)*makeTransformMatrix("y", -z)*makeTransformMatrix("z", y);
			glPushMatrix();
		//		CoordinateSystem_Local();
				glCallList(korpus_dol);
		//		float korpus_dol_m[16];
				//glGetFloatv(GL_MODELVIEW_MATRIX, korpus_dol_m);
				//copyTable(&m4,korpus_dol_m);
				//robot_model.korpus_dol_model->setTransform (korpus_dol_m);
			glPopMatrix();
				
			glTranslatef(0.0f,0.0f,-4.24*0.254);
				
			glPushMatrix();
				glCallList(korpus_gora);
				float korpus_gora_m[16];
				//glGetFloatv(GL_MODELVIEW_MATRIX, korpus_gora_m);
		//		copyTable(&m4,korpus_gora_m);
		//		robot_model.korpus_gora_model->setTransform (korpus_gora_m);
			glPopMatrix();

			
//===============NOGA_3=================================
				glPushMatrix();
				glTranslatef(-2.56*0.254,-6.06*0.254,3.33*0.254);
				//Matrix4f m_noga = Matrix4f::Identity();
				//m_noga = m4*makeTransformMatrix("x", -2.56)*makeTransformMatrix("y", -6.06)*makeTransformMatrix("z", 3.33);
				GLNoga_3(-Q7*180/3.14,-Q8*180/3.14,-Q9*180/3.14); 
				glPopMatrix();

//===============NOGA_4=================================
				glPushMatrix();
				glTranslatef(2.56*0.254,-6.06*0.254,3.33*0.254);
				glRotatef(180,0,0,1);
//				m_noga = m4*makeTransformMatrix("x", 2.56)*makeTransformMatrix("y", -6.06)*makeTransformMatrix("z", 3.33)*makeTransformMatrix("gamma", 3.14);
				GLNoga_4(Q10*180/3.14,-Q11*180/3.14,-Q12*180/3.14);	
				glPopMatrix();

//===============NOGA_1=================================				
				glPushMatrix();
				glTranslatef(-5.1*0.254,0.0,3.33*0.254);
//				m_noga = m4*makeTransformMatrix("x", -5.1)*makeTransformMatrix("y", 0.0)*makeTransformMatrix("z", 3.33);
				GLNoga_2(-Q4*180/3.14,-Q5*180/3.14,-Q6*180/3.14); 
				glPopMatrix();

//===============NOGA_5=================================
				glPushMatrix();
				glTranslatef(5.1*0.254,0,3.33*0.254);
				glRotatef(180,0,0,1);
//				m_noga = m4*makeTransformMatrix("x", 5.1)*makeTransformMatrix("y", 0.0)*makeTransformMatrix("z", 3.33)*makeTransformMatrix("gamma", 3.14);
				GLNoga_5(Q13*180/3.14,-Q14*180/3.14,-Q15*180/3.14);	
				glPopMatrix();

//===============NOGA_1=================================
				glPushMatrix();
				glTranslatef(-2.56*0.254,6.06*0.254,3.33*0.254);
//				m_noga = m4*makeTransformMatrix("x", -2.56)*makeTransformMatrix("y", 6.06)*makeTransformMatrix("z", 3.33);
				GLNoga_1(-Q1*180/3.14,-Q2*180/3.14,-Q3*180/3.14);	
				glPopMatrix();

//===============NOGA_6=================================
				glPushMatrix();
				glTranslatef(2.56*0.254,6.06*0.254,3.33*0.254);
				glRotatef(180,0,0,1);
				//m_noga = m4*makeTransformMatrix("x", 2.56)*makeTransformMatrix("y", 6.06)*makeTransformMatrix("z", 3.33)*makeTransformMatrix("gamma", 3.14);
				GLNoga_6(Q16*180/3.14,-Q17*180/3.14,-Q18*180/3.14);	
				glPopMatrix();

	glPopMatrix();
}

void CRobotStructure::GLDrawRobot(float *pos, float * rot, float Q1,  float Q2,  float Q3,  float Q4,  float Q5,  float Q6, 
			   float Q7,  float Q8,  float Q9,  float Q10, float Q11, float Q12, 
			   float Q13, float Q14, float Q15, float Q16, float Q17, float Q18)
{
	//CoordinateSystem_World();

	float GLmat[16]={rot[0], rot[4], rot[8], 0, rot[1], rot[5], rot[9], 0, rot[2], rot[6], rot[10], 0, pos[0], pos[1], pos[2], 1}; //macierz do przeksztalcen

		glPushMatrix();
		/*glTranslatef(x*10,z*10-0.1,-y*10);
		glRotatef(90+alpha*180/3.14,1,0,0);
		glRotatef(-beta*180/3.14,0,1,0);
		glRotatef(180-gamma*180/3.14,0,0,1);*/
		glMultMatrixf(GLmat);
		glRotatef(90,1,0,0);
		glRotatef(180,0,0,1);

		//Matrix4f m4 = Matrix4f::Identity();
		//m4=makeTransformMatrix("alpha", 3.14/2+alpha)*makeTransformMatrix("beta", -beta)*makeTransformMatrix("gamma", 3.14-gamma)*makeTransformMatrix("x", x)*makeTransformMatrix("y", -z)*makeTransformMatrix("z", y);
			glPushMatrix();
		//		CoordinateSystem_Local();
				glCallList(korpus_dol);
		//		float korpus_dol_m[16];
				//glGetFloatv(GL_MODELVIEW_MATRIX, korpus_dol_m);
				//copyTable(&m4,korpus_dol_m);
				//robot_model.korpus_dol_model->setTransform (korpus_dol_m);
			glPopMatrix();
				
			glTranslatef(0.0f,0.0f,-4.24*0.254);
				
			glPushMatrix();
				glCallList(korpus_gora);
				float korpus_gora_m[16];
				//glGetFloatv(GL_MODELVIEW_MATRIX, korpus_gora_m);
		//		copyTable(&m4,korpus_gora_m);
		//		robot_model.korpus_gora_model->setTransform (korpus_gora_m);
			glPopMatrix();

			
//===============NOGA_3=================================
				glPushMatrix();
				glTranslatef(-2.56*0.254,-6.06*0.254,3.33*0.254);
				//Matrix4f m_noga = Matrix4f::Identity();
				//m_noga = m4*makeTransformMatrix("x", -2.56)*makeTransformMatrix("y", -6.06)*makeTransformMatrix("z", 3.33);
				GLNoga_3(-Q7*180/3.14,-Q8*180/3.14,-Q9*180/3.14); 
				glPopMatrix();

//===============NOGA_4=================================
				glPushMatrix();
				glTranslatef(2.56*0.254,-6.06*0.254,3.33*0.254);
				glRotatef(180,0,0,1);
//				m_noga = m4*makeTransformMatrix("x", 2.56)*makeTransformMatrix("y", -6.06)*makeTransformMatrix("z", 3.33)*makeTransformMatrix("gamma", 3.14);
				GLNoga_4(Q10*180/3.14,-Q11*180/3.14,-Q12*180/3.14);	
				glPopMatrix();

//===============NOGA_1=================================				
				glPushMatrix();
				glTranslatef(-5.1*0.254,0.0,3.33*0.254);
//				m_noga = m4*makeTransformMatrix("x", -5.1)*makeTransformMatrix("y", 0.0)*makeTransformMatrix("z", 3.33);
				GLNoga_2(-Q4*180/3.14,-Q5*180/3.14,-Q6*180/3.14); 
				glPopMatrix();

//===============NOGA_5=================================
				glPushMatrix();
				glTranslatef(5.1*0.254,0,3.33*0.254);
				glRotatef(180,0,0,1);
//				m_noga = m4*makeTransformMatrix("x", 5.1)*makeTransformMatrix("y", 0.0)*makeTransformMatrix("z", 3.33)*makeTransformMatrix("gamma", 3.14);
				GLNoga_5(Q13*180/3.14,-Q14*180/3.14,-Q15*180/3.14);	
				glPopMatrix();

//===============NOGA_1=================================
				glPushMatrix();
				glTranslatef(-2.56*0.254,6.06*0.254,3.33*0.254);
//				m_noga = m4*makeTransformMatrix("x", -2.56)*makeTransformMatrix("y", 6.06)*makeTransformMatrix("z", 3.33);
				GLNoga_1(-Q1*180/3.14,-Q2*180/3.14,-Q3*180/3.14);	
				glPopMatrix();

//===============NOGA_6=================================
				glPushMatrix();
				glTranslatef(2.56*0.254,6.06*0.254,3.33*0.254);
				glRotatef(180,0,0,1);
				//m_noga = m4*makeTransformMatrix("x", 2.56)*makeTransformMatrix("y", 6.06)*makeTransformMatrix("z", 3.33)*makeTransformMatrix("gamma", 3.14);
				GLNoga_6(Q16*180/3.14,-Q17*180/3.14,-Q18*180/3.14);	
				glPopMatrix();

	glPopMatrix();
}

void CRobotStructure::drawTerrain(float x, float y)
{
	robot_model.ground.RenderTerrain_1();
	float terrain_1_matrix[16];
	//glGetFloatv(GL_MODELVIEW_MATRIX, terrain_1_matrix);
	Matrix4f m4 = Matrix4f::Identity();
		m4=makeTransformMatrix("x", x)*makeTransformMatrix("z", y);
		copyTable(&m4, terrain_1_matrix);
	robot_model.teren_1_model->setTransform (terrain_1_matrix);
	robot_model.ground.RenderTerrain_2();
	float terrain_2_matrix[16];
	//glGetFloatv(GL_MODELVIEW_MATRIX, terrain_2_matrix);
	robot_model.teren_2_model->setTransform (terrain_1_matrix);
}

bool CRobotStructure::checkCollision(float x, float y, float z, float alpha, float beta, float gamma, float * angles, bool * collision_table,bool check_ground, double ***map, int offset_x, int offset_y)
{

	/*angles[0]-=0*3.14/180;		angles[1]-=24*3.14/180;		angles[2]-=-114*3.14/180;
	angles[3]-=0*3.14/180;		angles[4]-=24*3.14/180;		angles[5]-=-114*3.14/180;
	angles[6]-=-0*3.14/180;		angles[7]-=24*3.14/180;		angles[8]-=-114*3.14/180;
	angles[9]-=-0*3.14/180;		angles[10]-=24*3.14/180;	angles[11]-=-114*3.14/180;
	angles[12]-=0*3.14/180;		angles[13]-=24*3.14/180;	angles[14]-=-114*3.14/180;
	angles[15]-=0*3.14/180;		angles[16]-=24*3.14/180;	angles[17]-=-114*3.14/180;*/
		//int win = glutCreateWindow("dd");

			
		//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	/*x=x*10;
	float tmp = y;
	y=(z*10-0.1);
	z=tmp*10;*/
	if (check_ground) 
	{
		robot_model.ground.InitializeTerrain(map,offset_x,offset_y);
		robot_model.TerrainCollisionModels();	// Init Collision Models
		drawTerrain(0*10,0*10);
	}
		DrawRobot(x,y,z,alpha,beta,gamma,angles[0],angles[1],angles[2],angles[3],angles[4],angles[5],angles[6],angles[7],angles[8],angles[9],angles[10],angles[11],angles[12],angles[13],angles[14],angles[15],angles[16],angles[17]);
		//glutDestroyWindow(win);

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
		if (robot_model.segment_II_model_1->collision(robot_model.segment_II_model_2)) {
			collision_table[6]=true; collision_table[7]=true;
		}
		if (robot_model.segment_II_model_2->collision(robot_model.segment_II_model_3)) {
			collision_table[7]=true; collision_table[8]=true;
		}
		if (robot_model.segment_II_model_4->collision(robot_model.segment_II_model_5)) {
			collision_table[9]=true; collision_table[10]=true;
		}
		if (robot_model.segment_II_model_5->collision(robot_model.segment_II_model_6)) {
			collision_table[10]=true; collision_table[11]=true;
		}
		//=========KOLIZJE trzecimi ogniwami od korpusu roznymi nogami
		if (robot_model.segment_I_model_1->collision(robot_model.segment_I_model_2)) {
			collision_table[12]=true; collision_table[13]=true;
		}
		if (robot_model.segment_I_model_2->collision(robot_model.segment_I_model_3)) {
			collision_table[13]=true; collision_table[14]=true;
		}
		if (robot_model.segment_I_model_4->collision(robot_model.segment_I_model_5)) {
			collision_table[15]=true; collision_table[16]=true;
		}
		if (robot_model.segment_I_model_5->collision(robot_model.segment_I_model_6)) {
			collision_table[16]=true; collision_table[17]=true;
		}
		//=========KOLIZJE drugimi, a trzecimi ogniwami od korpusu miedzy roznymi nogami
		if (robot_model.segment_I_model_1->collision(robot_model.segment_II_model_2)) {
			collision_table[7]=true; collision_table[12]=true;
		}
		if (robot_model.segment_I_model_2->collision(robot_model.segment_II_model_1)) {
			collision_table[6]=true; collision_table[13]=true;
		}
		if (robot_model.segment_I_model_2->collision(robot_model.segment_II_model_3)) {
			collision_table[8]=true; collision_table[13]=true;
		}
		if (robot_model.segment_I_model_3->collision(robot_model.segment_II_model_2)) {
			collision_table[7]=true; collision_table[14]=true;
		}
		if (robot_model.segment_I_model_4->collision(robot_model.segment_II_model_5)) {
			collision_table[10]=true; collision_table[15]=true;
		}
		if (robot_model.segment_I_model_5->collision(robot_model.segment_II_model_4)) {
			collision_table[9]=true; collision_table[16]=true;
		}
		if (robot_model.segment_I_model_5->collision(robot_model.segment_II_model_6)) {
			collision_table[11]=true; collision_table[16]=true;
		}
		if (robot_model.segment_I_model_6->collision(robot_model.segment_II_model_5)) {
			collision_table[10]=true; collision_table[17]=true;
		}
		//=========KOLIZJE trzecimi ogniwami od korpusu z terenem
		if (check_ground){
			if ((robot_model.segment_I_model_1->collision(robot_model.teren_1_model))||(robot_model.segment_I_model_1->collision(robot_model.teren_2_model))){
				collision_table[18]=true; collision_table[24]=true;
			}
			if ((robot_model.segment_I_model_2->collision(robot_model.teren_1_model))||(robot_model.segment_I_model_2->collision(robot_model.teren_2_model))){
				collision_table[19]=true; collision_table[24]=true;
			}
			if ((robot_model.segment_I_model_3->collision(robot_model.teren_1_model))||(robot_model.segment_I_model_3->collision(robot_model.teren_2_model))){
				collision_table[20]=true; collision_table[24]=true;
			}
			if ((robot_model.segment_I_model_4->collision(robot_model.teren_1_model))||(robot_model.segment_I_model_4->collision(robot_model.teren_2_model))){
				collision_table[21]=true; collision_table[24]=true;
			}
			if ((robot_model.segment_I_model_5->collision(robot_model.teren_1_model))||(robot_model.segment_I_model_5->collision(robot_model.teren_2_model))){
				collision_table[22]=true; collision_table[24]=true;
			}
			if ((robot_model.segment_I_model_6->collision(robot_model.teren_1_model))||(robot_model.segment_I_model_6->collision(robot_model.teren_2_model))){
				collision_table[23]=true; collision_table[24]=true;
			}
			//=========KOLIZJA korpusu z podlozem
			if ((robot_model.korpus_dol_model->collision(robot_model.teren_1_model))||(robot_model.korpus_dol_model->collision(robot_model.teren_2_model))||(robot_model.korpus_gora_model->collision(robot_model.teren_1_model))||(robot_model.korpus_gora_model->collision(robot_model.teren_2_model))){
				collision_table[25]=true; collision_table[24]=true;
			}
			if ((robot_model.przegub_typu_C_1->collision(robot_model.teren_1_model))||(robot_model.przegub_typu_C_1->collision(robot_model.teren_2_model))||(robot_model.przegub_typu_C_2->collision(robot_model.teren_1_model))||(robot_model.przegub_typu_C_2->collision(robot_model.teren_2_model))){
				collision_table[25]=true; collision_table[24]=true;
			}
			if ((robot_model.przegub_typu_C_3->collision(robot_model.teren_1_model))||(robot_model.przegub_typu_C_3->collision(robot_model.teren_2_model))||(robot_model.przegub_typu_C_4->collision(robot_model.teren_1_model))||(robot_model.przegub_typu_C_4->collision(robot_model.teren_2_model))){
				collision_table[25]=true; collision_table[24]=true;
			}
			if ((robot_model.przegub_typu_C_5->collision(robot_model.teren_1_model))||(robot_model.przegub_typu_C_5->collision(robot_model.teren_2_model))||(robot_model.przegub_typu_C_5->collision(robot_model.teren_1_model))||(robot_model.przegub_typu_C_5->collision(robot_model.teren_2_model))){
				collision_table[25]=true; collision_table[24]=true;
			}
		}

		for (int i=0;i<44;i++){
			if (collision_table[i]==true) return true;
		}
		return false;
}
