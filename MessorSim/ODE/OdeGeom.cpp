#include "OdeGeom.h"
#include <math.h>

//green
//const float leg_color[3]={0.6, 0.7, 0.1};
//const float body_color[3]={0.3, 0.4, 0.0};
const float leg_color[3]={0.3, 0.6, 0.7};
const float body_color[3]={0.2, 0.4, 0.5};

COdeGeom::COdeGeom(void)
{
}

COdeGeom::~COdeGeom(void)
{
}

//rysuje szescian
void COdeGeom::DrawBox(dReal * sides, const float * pos, const float * R)
{
    double side[3];
    side[0] = sides[0] / 2*10;
    side[1] = sides[1] / 2*10;
    side[2] = sides[2] / 2*10;

	float pos1[3];
	for (int i=0;i<3;i++) {
		pos1[i]=pos[i]*10;
	}
    float mat_ambient[] = { 0.0, 0.0, 1.0, 1.0 };
    float mat_diffuse[] = { 0.0, 0.0, 1.0, 1.0 };
    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
            
    glPushMatrix();
    GeomMatrix.ODEtoOGL(pos1, R);
    glMultMatrixf(GeomMatrix.Element);

	

	float colors_rgb[12][3] =
	{
		{0.5f,0.1f,0.1f }, {1.0f,0.1f,0.1f }, // Red
		{0.5f,0.1f,0.1f }, {1.0f,0.1f,0.1f }, // Yellow
		{0.5f,0.1f,0.1f }, {1.0f,0.1f,0.1f }, // Green
		{0.5f,0.1f,0.1f }, {1.0f,0.1f,0.1f }, // Cyan
		{0.5f,0.1f,0.1f }, {1.0f,0.1f,0.1f }, // Blue
		{0.5f,0.1f,0.1f }, {1.0f,0.1f,0.1f } // Magenta
	};

    glBegin(GL_QUADS);
        // Front Face
        glNormal3f(0.0, 0.0, 1.0);
		glColor3fv( colors_rgb[1] );
        glVertex3f(-side[0], -side[1], side[2]);
        glColor3fv( colors_rgb[2] );
		glVertex3f(side[0], -side[1], side[2]);
		glColor3fv( colors_rgb[3] );
        glVertex3f(side[0], side[1], side[2]);
		glColor3fv( colors_rgb[4] );
        glVertex3f(-side[0], side[1], side[2]);

        // Back Face
        glNormal3f(0.0, 0.0, -1.0);
		glColor3fv( colors_rgb[1] );
        glVertex3f(side[0], -side[1], -side[2]);
		glColor3fv( colors_rgb[2] );
        glVertex3f(-side[0], -side[1], -side[2]);
		glColor3fv( colors_rgb[3] );
        glVertex3f(-side[0], side[1], -side[2]);
		glColor3fv( colors_rgb[4] );
        glVertex3f(side[0], side[1], -side[2]);

        // Top Face
        glNormal3f(0.0, 1.0, 0.0);
		glColor3fv( colors_rgb[1] );
        glVertex3f(-side[0], side[1], side[2]);
		glColor3fv( colors_rgb[2] );
        glVertex3f(side[0], side[1], side[2]);
		glColor3fv( colors_rgb[3] );
        glVertex3f(side[0], side[1], -side[2]);
		glColor3fv( colors_rgb[4] );
        glVertex3f(-side[0], side[1], -side[2]);

        // Bottom Face
        glNormal3f(0.0, -1.0, 0.0);
		glColor3fv( colors_rgb[1] );
        glVertex3f(-side[0], -side[1], side[2]);
		glColor3fv( colors_rgb[2] );
        glVertex3f(-side[0], -side[1], -side[2]);
		glColor3fv( colors_rgb[3] );
        glVertex3f(side[0], -side[1], -side[2]);
		glColor3fv( colors_rgb[4] );
        glVertex3f(side[0], -side[1], side[2]);

        // Right face
        glNormal3f(1.0, 0.0, 0.0);
		glColor3fv( colors_rgb[1] );
        glVertex3f(side[0], -side[1], side[2]);
		glColor3fv( colors_rgb[2] );
        glVertex3f(side[0], -side[1], -side[2]);
		glColor3fv( colors_rgb[3] );
        glVertex3f(side[0], side[1], -side[2]);
		glColor3fv( colors_rgb[4] );
        glVertex3f(side[0], side[1], side[2]);

        // Left Face
        glNormal3f(-1.0, 0.0, 0.0);
		glColor3fv( colors_rgb[1] );
        glVertex3f(-side[0], -side[1], -side[2]);
		glColor3fv( colors_rgb[2] );
        glVertex3f(-side[0], -side[1], side[2]);
		glColor3fv( colors_rgb[3] );
        glVertex3f(-side[0], side[1], side[2]);
		glColor3fv( colors_rgb[4] );
        glVertex3f(-side[0], side[1], -side[2]);
    glEnd();
    glPopMatrix();
}
//rysuje sfere
void COdeGeom::DrawSphere(float radius, const float *pos, const float *R)
{
	float pos1[3];
	for (int i=0;i<3;i++) {
		pos1[i]=pos[i]*10;
	}

	GLUquadricObj *sphere;
    sphere = gluNewQuadric();
	
	gluQuadricDrawStyle(sphere,GLU_LINE);

    float mat_ambient[] = { 0.8, 0.8, 0.8, 1.0 };
    float mat_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);

    glPushMatrix();
    GeomMatrix.ODEtoOGL(pos1, R);
    glMultMatrixf(GeomMatrix.Element);
	    //DrawCoordinateSystem();
    gluSphere(sphere, radius*10, 32, 32);
    glPopMatrix();
}

//rysuje pigule
void COdeGeom::DrawCappedCylinder(const float * pos, const float *R, float radius, float length)
{
   glColor3f(1.0, 1.0, 0.0);
	GLUquadricObj *sphere;
	float pos1[3];
	for (int i=0;i<3;i++) {
		pos1[i]=pos[i]*10;
	}

	sphere = gluNewQuadric();

	gluQuadricTexture(sphere, GL_TRUE);
    gluQuadricDrawStyle(sphere,GLU_LINE);


    float mat_ambient[] = { 0.8, 0.8, 0.8, 1.0 };
    float mat_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);

     glPushMatrix();
    GeomMatrix.ODEtoOGL(pos1, R);
    glMultMatrixf(GeomMatrix.Element);
	
	//DrawCoordinateSystem();

	gluCylinder( sphere, radius*10, radius*10, length*10/2, 32, 32);
	glTranslatef(0,0,length*10/2);
	gluSphere(sphere, radius*10, 32, 32);
	glTranslatef(0,0,-length*10);
	gluCylinder( sphere, radius*10, radius*10, length*10/2, 32, 32);
	gluSphere(sphere, radius*10, 32, 32);
    glPopMatrix();
}

//rysuje korpus
void COdeGeom::DrawKorpus(dReal * sides, const float * pos, const float * R)
{
    double skala[3];
    skala[0] = sides[0] / 3*10;
    skala[1] = sides[1] / 3*10;
    skala[2] = sides[2] / 3*10;

	float pos1[3];
	for (int i=0;i<3;i++) 
		pos1[i]=pos[i]*10;

glPushMatrix();
	glDisable(GL_TEXTURE_2D);
	glColor3d(body_color[0],body_color[1],body_color[2]);
	GeomMatrix.ODEtoOGL(pos1, R);
	glMultMatrixf(GeomMatrix.Element);

	//GLUquadric *korpus;
	//korpus=gluNewQuadric();
	//gluQuadricDrawStyle(korpus,GLU_FILL);
	//gluQuadricNormals(korpus,GLU_SMOOTH);
	//gluCylinder(korpus,M_PI*2,M_PI*2,10.0f,20.0f,20.0f);

	glScalef(0.1,0.1,0.1);
	glRotatef(90,0,1,0);
	
	glScalef(1,0.5,1);

	//trzy ³¹cza 
	drawCapsule(skala[1]*40.0f+0.5f,skala[1]*100.0f,5);
	glTranslatef(10,0,0);
	drawCapsule(skala[1]*5.0f,skala[1]*80.0f,5);
	glTranslatef(-20,0,0);
	drawCapsule(skala[1]*5.0f,skala[1]*80.0f,5);

	glTranslatef(10,0,0);
	glScalef(2.5,1,0.5);
	
	glRotatef(90,1,0,0);
	glutSolidTorus(2,2,80.0f,40.0f);
	glutSolidTorus(3,4,80.0f,40.0f);

	glScalef(0.4*0.6,2.5,1);
	glutSolidTorus(2,5,80.0f,40.0f);
	//glutSolidTorus(2,5,80.0f,40.0f);
	
	glTranslatef(2.7*10,0.27*10,-0.2*10);
	glColor3f(1.0, 1.0, 1.0);
	GLUquadricObj *sphere;
	sphere = gluNewQuadric();
	glScalef(1,0.5,1.2);
	gluSphere(sphere, 0.2*10, 32, 32);

	glTranslatef(0,-1.08*10,0);
	gluSphere(sphere, 0.2*10, 32, 32);

	glTranslatef(0.1*10,-0.05*10,0);
	glColor3f(0.0, 0.0, 0.0);
	glScalef(0.5,0.5,0.6);
	gluSphere(sphere, 0.2*10, 32, 32);

	glTranslatef(0,24.08,0);
	gluSphere(sphere, 0.18*10, 32, 32);
glPopMatrix();
//gluDeleteQuadric(korpus);
}

//rysuje walec
void COdeGeom::DrawCylinder(const float * pos, const float *R, float base, float top, float length){
	glColor3f(1.0, 1.0, 0.0);
	GLUquadricObj *sphere;
	float pos1[3];
	for (int i=0;i<3;i++) {
		pos1[i]=pos[i]*10;
	}

	sphere = gluNewQuadric();

	gluQuadricTexture(sphere, GL_TRUE);
    gluQuadricDrawStyle(sphere,GLU_LINE);

	glPushMatrix();
	glDisable(GL_TEXTURE_2D);
	glColor3d(body_color[0],body_color[1],body_color[2]);
	GeomMatrix.ODEtoOGL(pos1, R);
	glMultMatrixf(GeomMatrix.Element);

	gluCylinder( sphere, top*10, base*10, length*10/2, 16, 16);
	glPopMatrix();
}

// rysuje kapsule
void COdeGeom::drawCapsule (float l, float r,int quality)
{
 glPushMatrix();
  int i,j;
  float tmp,nx,ny,nz,start_nx,start_ny,a,ca,sa;
  // number of sides to the cylinder (divisible by 4):
  const int n = quality*4;

  l *= 0.5;
  a = float(M_PI*2.0)/float(n);
  sa = (float) sin(a);
  ca = (float) cos(a);

  // draw cylinder body
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_STRIP);
  for (i=0; i<=n; i++) {
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,l);
    glNormal3d (ny,nz,0);
    glVertex3d (ny*r,nz*r,-l);
    // rotate ny,nz
    tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw first cylinder cap
  start_nx = 0;
  start_ny = 1;
  for (j=0; j<(n/4); j++) {
    // get start_n2 = rotated start_n
    float start_nx2 =  ca*start_nx + sa*start_ny;
    float start_ny2 = -sa*start_nx + ca*start_ny;
    // get n=start_n and n2=start_n2
    nx = start_nx; ny = start_ny; nz = 0;
    float nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
    glBegin (GL_TRIANGLE_STRIP);
    for (i=0; i<=n; i++) {
      glNormal3d (ny2,nz2,nx2);
      glVertex3d (ny2*r,nz2*r,l+nx2*r);
      glNormal3d (ny,nz,nx);
      glVertex3d (ny*r,nz*r,l+nx*r);
      // rotate n,n2
      tmp = ca*ny - sa*nz;
      nz = sa*ny + ca*nz;
      ny = tmp;
      tmp = ca*ny2- sa*nz2;
      nz2 = sa*ny2 + ca*nz2;
      ny2 = tmp;
    }
    glEnd();
    start_nx = start_nx2;
    start_ny = start_ny2;
  }

  // draw second cylinder cap
  start_nx = 0;
  start_ny = 1;
  for (j=0; j<(n/4); j++) {
    // get start_n2 = rotated start_n
    float start_nx2 = ca*start_nx - sa*start_ny;
    float start_ny2 = sa*start_nx + ca*start_ny;
    // get n=start_n and n2=start_n2
    nx = start_nx; ny = start_ny; nz = 0;
    float nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
    glBegin (GL_TRIANGLE_STRIP);
    for (i=0; i<=n; i++) {
      glNormal3d (ny,nz,nx);
      glVertex3d (ny*r,nz*r,-l+nx*r);
      glNormal3d (ny2,nz2,nx2);
      glVertex3d (ny2*r,nz2*r,-l+nx2*r);
      // rotate n,n2
      tmp = ca*ny - sa*nz;
      nz = sa*ny + ca*nz;
      ny = tmp;
      tmp = ca*ny2- sa*nz2;
      nz2 = sa*ny2 + ca*nz2;
      ny2 = tmp;
    }
    glEnd();
    start_nx = start_nx2;
    start_ny = start_ny2;
  }

  glPopMatrix();
}

////rysuje kapsu³e - pigu³e + wiêksze kule, cz³ony nogi
void COdeGeom::DrawCCylinderBox(float radius,float dlugosc, const float* pos, const float* R)
{
	float pos1[3];
	for (int i=0;i<3;i++) {
		pos1[i]=pos[i]*10;
	}

	glPushMatrix();
	glDisable(GL_TEXTURE_2D);
	glColor3d(leg_color[0],leg_color[1],leg_color[2]);
	GeomMatrix.ODEtoOGL(pos1, R);
	glMultMatrixf(GeomMatrix.Element);

	glRotatef(90,0,1,0);

	GLUquadric *walec;
	walec=gluNewQuadric();
	gluQuadricDrawStyle(walec,GLU_FILL);
	gluQuadricNormals(walec,GLU_SMOOTH);   
	
	glColor3d(leg_color[0],leg_color[1],leg_color[2]);
    //glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, k2);
	radius*=0.75;//0.5;

	glTranslatef(0,0,-5.0f*dlugosc);
	gluCylinder(walec,10.0f*radius,10.0f*radius,10.0f*dlugosc,20.0f,20.0f);
	gluSphere(walec,10.0f*radius,20.0f,20.0f);

	gluSphere(walec,35.0f*radius*0.5,20.0f,20.0f);
	glRotatef(-90,0,1,0);
	drawCapsule(0.15,0.085,3);
	glRotatef(90,0,1,0);

	glTranslatef(0,0,2*5.0f*dlugosc);
	gluSphere(walec,10.0f*radius,20.0f,20.0f);
	
	glRotatef(-90,0,1,0);
	drawCapsule(0.15,0.085,3);
	gluSphere(walec,35.0f*radius*0.5,20.0f,20.0f);

	glPopMatrix();
}

///rysuje kapsu³e - pigu³e
void COdeGeom::DrawCCylinder(float radius,float dlugosc, const float* pos, const float* R)
{
	float pos1[3];
	for (int i=0;i<3;i++) {
		pos1[i]=pos[i]*10;
	}

	glPushMatrix();
	glDisable(GL_TEXTURE_2D);
	glColor3d(leg_color[0],leg_color[1],leg_color[2]);
	GeomMatrix.ODEtoOGL(pos1, R);
	glMultMatrixf(GeomMatrix.Element);
	drawCapsule(10.0f*dlugosc,10.0f*radius,3);
	glPopMatrix();

	
}
/*sterowanie chodem robota*/
//rysuje uklad wspolrzednych
void COdeGeom::DrawCoordinateSystem(float rotx,float roty,float rotz,float posx, float posy, float posz)
{
	glPushMatrix();
	glTranslatef(posx,posz,-posy);
	//glRotatef(rotx-57.295780,1.0f,0.0f,0.0f);
	//glRotatef(roty-57.295780,0.0f,0.0f,1.0f);
	glRotatef(rotz,0.0f,1.0f,0.0f);
	glRotatef(roty,0.0f,0.0f,-1.0f);
	glRotatef(rotx,1.0f,0.0f,0.0f);
	//glPopMatrix();
	glBegin(GL_LINES);
	//oœ y
	 glNormal3f(1.0, 0.0, 0.0);
	glColor3f(0.0f,1.0,0.0);
	glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
	glVertex3f(0.0f, 0.0f, -0.5f); // ending point of the line
	//oœ z
	 glNormal3f(1.0, 0.0, 0.0);
	glColor3f(0.0f,0.0,1.0);
	glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
	glVertex3f(0.0f, 0.5f, 0.0f); // ending point of the line
	//oœ x
	 glNormal3f(1.0, 0.0, 0.0);
	glColor3f(1.0f,0.0f,0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
	glVertex3f(0.5f, 0.0f, 0.0f); // ending point of the line
	glEnd();
	glPopMatrix();
}

void COdeGeom::DrawCoordinateSystem(CPunctum m)
{
	float tmp[16];

	m.getMatrix(tmp);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixf(tmp);
	glBegin(GL_LINES);
	//oœ y
	glNormal3f(1.0, 0.0, 0.0);
	glColor3f(0.0f,1.0,0.0);
	glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
	glVertex3f(0.0f, 0.0f, -0.5f); // ending point of the line
	//oœ z
	glNormal3f(1.0, 0.0, 0.0);
	glColor3f(0.0f,0.0,1.0);
	glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
	glVertex3f(0.0f, 0.5f, 0.0f); // ending point of the line
	//oœ x
	glNormal3f(1.0, 0.0, 0.0);
	glColor3f(1.0f,0.0f,0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
	glVertex3f(0.5f, 0.0f, 0.0f); // ending point of the line
	glEnd();
	glPopMatrix();
}
void COdeGeom::DrawCoordinateSystem()
{
	glPushMatrix();
	glBegin(GL_LINES);
	//oœ y
	 glNormal3f(1.0, 0.0, 0.0);
	glColor3f(0.0f,1.0,0.0);
	glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
	glVertex3f(0.0f, 0.0f, -0.5f); // ending point of the line
	//oœ z
	 glNormal3f(1.0, 0.0, 0.0);
	glColor3f(0.0f,0.0,1.0);
	glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
	glVertex3f(0.0f, 0.5f, 0.0f); // ending point of the line
	//oœ x
	 glNormal3f(1.0, 0.0, 0.0);
	glColor3f(1.0f,0.0f,0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f); // origin of the line
	glVertex3f(0.5f, 0.0f, 0.0f); // ending point of the line
	glEnd();
	glPopMatrix();
}
