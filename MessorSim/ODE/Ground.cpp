#include "Ground.h"
#include "../functions.h"

//#define mapa_adres "F:/uczelnia/badania/mapy_symulator/powiekszone300x300/mapa_big1.dat"
//#define mapa_adres "C:/NOWY_DYSK/uczelnia/badania/mapy_symulator/powiekszone300x300/mapa_big1.dat"
#define mapa_adres "C:/NOWY_DYSK/uczelnia/badania/mapy_symulator/offlineMap10kalman1.dat"
//#define mapa_adres ""
#define mapa2_adres ""//"D:/MessorSim/MessorSim/mapy_symulator/powiekszone300x300/mapa5-wiarygodnosc.dat"
#define mapa3_adres ""//"D:/MessorSim/MessorSim/mapy_symulator/powiekszone300x300/mapa5-wysokosc_pomiar.dat"

#pragma warning( disable : 4996 )

CGround::CGround(void)
{
	min_height = 1e10;
	max_height = -1e10;
}

/// length_x - surface length in x axis [m], length_y - surface length in y axis [m]
/// size_x - surface size in x axis (number of points), size_y - surface size in y axis (number of points)
CGround::CGround(double length_x, double length_y, int size_x, int size_y, double max_height)
{
	mesh_x_length=length_x; //surface length in x axis [m]
	mesh_y_length=length_y; //surface length in y axis [m]
	mesh_x_size=size_x; //surface size in x axis (number of points)
	mesh_y_size=size_y; //surface size in y axis (number of points)
    points = new double **[mesh_x_size];
	reliability_map = new double **[mesh_x_size];
	measured_map = new double **[mesh_x_size];
	min_height = 1e10;
	max_height = -1e10;
    for (int i=0;i<mesh_x_size;i++)
    {
        points[i]=new double *[mesh_y_size];
		reliability_map[i]=new double *[mesh_y_size];
		measured_map[i]=new double *[mesh_y_size];
        for (int j=0;j<mesh_y_size;j++)
        {
            points[i][j]=new double [3];// trojelementowa na x, y i z
			reliability_map[i][j]=new double [3];// trojelementowa na x, y i z
			measured_map[i][j]=new double [3];// trojelementowa na x, y i z
        }
    }
}

CGround::~CGround(void)
{
	for (int i=0;i<mesh_x_size;i++)
    {
		for (int j=0; j<mesh_y_size;j++)
		{
			delete points[i][j];
		}
		delete [] points[i];
	}
	delete [] points;
}



//draw lines - siatka
void CGround::DrawGrid(void)
{
    glPushMatrix();

    float Line = -10;
    int Grid;
    glBegin(GL_LINES);
    for(Grid = 0; Grid <= 20; Grid += 1)
    {
        glColor3f(0.0f,1.0f,0.0f);
        glVertex3f(Line + Grid, 0, -10);
        glVertex3f(Line + Grid, 0, 10);
        glVertex3f(-10, 0, Line + Grid);
        glVertex3f(10, 0, Line + Grid);
    }
    glEnd();
    glPopMatrix();
}

void CGround::initializeList(int * indexes, dVector3 * triVert)
{

	indexes = new int[3*2*(this->getYnumPoints()-1)*(this->getXnumPoints()-1)];//liczba trojkatow
	this->returnIndexes(indexes);
	triVert = new dVector3[this->getYnumPoints()*this->getXnumPoints()];
	this->returnTriVert(triVert);

	glNewList(ground_surface, GL_COMPILE);
	glPushMatrix();
	glBegin(GL_TRIANGLES);

	float normal[3];

	float ** vert = new float*[3];
	for(int i = 0; i < 3; i++)
		vert[i] = new float[3];
	unsigned char rgb[3];
	double k=-1;
		for (int i = 0; i < 3*2*(mesh_x_size-1)*(mesh_y_size-1); i=i+3) 
		{
			if (triVert[indexes[i]][1]>k) 
				k=triVert[indexes[i]][1];
			positiveColorMap(rgb,max_height-triVert[indexes[i]][1],1.6*min_height, 1.6*max_height);
			glColor3f(float(rgb[0])/255.0, float(rgb[1])/255.0, float(rgb[2])/255.0);
			for (int k1=0;k1<3;k1++){
				for (int l=0;l<3;l++){
					vert[k1][l]=triVert[indexes[i+k1]][l];
				}
			}
			calcNormal(vert, normal);
			glNormal3d (normal[0],normal[1],normal[2]);
			glVertex3f(triVert[indexes[i]][0]*10,triVert[indexes[i]][1]*10,triVert[indexes[i]][2]*10);
			positiveColorMap(rgb,max_height-triVert[indexes[i+1]][1],1.6*min_height, 1.6*max_height);
			glColor3f(float(rgb[0])/255.0, float(rgb[1])/255.0, float(rgb[2])/255.0);
			glVertex3f(triVert[indexes[i+1]][0]*10,triVert[indexes[i+1]][1]*10,triVert[indexes[i+1]][2]*10);
			positiveColorMap(rgb,max_height-triVert[indexes[i+2]][1],1.6*min_height, 1.6*max_height);
			glColor3f(float(rgb[0])/255.0, float(rgb[1])/255.0, float(rgb[2])/255.0);
			glVertex3f(triVert[indexes[i+2]][0]*10,triVert[indexes[i+2]][1]*10,triVert[indexes[i+2]][2]*10);
		}
	glEnd();
	glPopMatrix();

	glEndList();
	for(int i = 0; i < 3; i++)
		delete [] vert[i];
	delete [] vert;
}

/// draw mesh
void CGround::DrawMesh(int a[6], int b[6], int foot_groundx_def[6], int foot_groundy_def[6], int targetx, int targety)
{
	glCallList(ground_surface);
}
///losuje liczbe rzeczywista z zakresu <min,max>
double CGround::randDouble(double min, double max) {
	double value;
	value = min + rand() * (max - min) / RAND_MAX;
	return value;
}

/// losuje punkty siatki (wysokosc)
void CGround::randMesh(void) 
{
	min_height=10;
	max_height=-10;

    for (int i=0;i<mesh_x_size;i++)
    {
        for (int j=0;j<mesh_y_size;j++)
        {
			points[i][j][0]=(mesh_x_length/(mesh_x_size-1))*i-(mesh_x_length/2);// wspolrzedna x
			points[i][j][1]=(mesh_y_length/(mesh_y_size-1))*j-(mesh_y_length/2);// wspolrzedna y
			points[i][j][2]=0.0;
		}
    }
}

///get mesh_x_size
int CGround::getXnumPoints(){//get mesh_x_size
	return mesh_x_size;
}

///get mesh_y_size
int CGround::getYnumPoints(){//get mesh_y_size
	return mesh_y_size;
}
///get x size in meters
double CGround::getXsize()
{
return mesh_x_length;

}
///get y size in meters
double CGround::getYsize()
{
return mesh_y_length;

}
/// return table of indexes (triangle points)
void CGround::returnIndexes(int * indexes){
	int size=2*(mesh_x_size-1)*(mesh_y_size-1);//liczba trojkatow - prawdziwe tylko dla kwadratowej siatki
	int row=0;
	int row2=mesh_x_size;
	int num=0;
    for (int i=0;i<size*3;i=i+6)
    {
		//kwadrat
		//pierwszy trojkat
		indexes[i]=row2;		indexes[i+1]=row;		indexes[i+2]=row+1;
		//drugi trojkat
		indexes[i+3]=row2+1;		indexes[i+4]=row2;		indexes[i+5]=row+1;
		if ((row-num*mesh_x_size)==(mesh_x_size-2)) {
			num++;
			row=num*mesh_x_size;
			row2=(num+1)*mesh_x_size;
		}
		else {
			row++;	
			row2++;
		}
    }
}

/// return triangle coordinates
void CGround::returnTriVert(dVector3 * triVert){// return triangle coordinates
	int num_point=0;
    for (int i=0;i<mesh_x_size;i++)
    {
		//zmodyfikowane
        for (int j=0;j<mesh_y_size;j++)
        {
			triVert[num_point][0]=points[i][j][0];// wspolrzedna x
			triVert[num_point][1]=points[i][j][2];// wspolrzedna y
			triVert[num_point][2]=points[i][j][1];// wysokosc punktu
			num_point++;
        }
    }
}
//funkcja do zaokraglania
int CGround::my_round (double x) {
  int i = (int) x;
  if (x >= 0.0) {
    return ((x-i) >= 0.5) ? (i + 1) : (i);
  } else {
    return (-x+i >= 0.5) ? (i - 1) : (i);
  }

} 

/*kroczenie po trudnym terenie*/

///wyznacza wspó³czynnik skali dla X
void  CGround::getScaleFactor(float *factor)
{
	factor[0]=getXsize()/getXnumPoints();
	factor[1]=getYsize()/getYnumPoints();
}
///wyznacza wspó³czynnik skali dla Y
double CGround::getScaleFactorY()
{
	return getYsize()/getYnumPoints();
}
///wyznacza wspó³czynnik skali dla Y
double CGround::getScaleFactorX()
{
	return getXsize()/getXnumPoints();
}

///wyznacza wspó³rzêdn¹ X œrodka terenu w jego wspolrzednych
double CGround::getCenterX()
{
//	double centerX;
	return getXsize()/2;
}
///wyznacza wspó³rzêdn¹ Y œrodka terenu w jego wspolrzednych
double CGround::getCenterY()
{
//	double centerY;
	return getYsize()/2;
}

///przelicza wspolrzedne robota na wspolrzedne terenu
void CGround::CalculateGroundCoordinates(double x, double y, int * result)
{
	double centerX=getCenterX();
	double centerY=getCenterY();
	double factorX=getScaleFactorX();
	double factorY=getScaleFactorY();
	result[0]=my_round((x+centerX-(factorX/2))/factorX);
	result[1]=my_round((-y+centerY-factorY/2)/factorY);
}

//zwraca wysokoœæ "gruntu" w danym punkcie
double CGround::getReliability(int x, int y)
{
	if ((x<0)||(y<0)||(x>=mesh_x_size)||(y>=mesh_y_size)) return 0;
	else
		return reliability_map[x][y][2];
}

//zwraca maksymaln¹ wysokoœænorma
double CGround::getMaxHeight()
{
	return max_height;
}

//load *.dat map file
void CGround::loadDatData(FILE * file, double *** data_structure)
{
	float temp;
	for (int i=0;i<mesh_x_size;i++) {
		for (int j=0;j<mesh_y_size;j++) {
			data_structure[i][j][0]=(mesh_x_length/(mesh_x_size-1))*i-(mesh_x_length/2);// wspolrzedna x
			data_structure[i][j][1]=(mesh_y_length/(mesh_y_size-1))*j-(mesh_y_length/2);// wspolrzedna y
			fscanf(file,"%f ",&temp);
			data_structure[j][i][2]=temp;
		}
		fscanf(file,"\n");
	}
}

//wczytuje mapê z pliku
void CGround::readCustMap(int sizex, int sizey)
{	
	//mapa wysokoœci (idealna mapa terenu)
	FILE *mapa=fopen(mapa_adres,"r");
	if (mapa!=NULL)	{
		loadDatData(mapa, points);
		for (size_t i = 0; i<mesh_x_size;i++){
			for (size_t j = 0; j<mesh_x_size;j++){
				if (points[j][i][2]>max_height) 
					max_height=points[j][i][2];
				if (points[j][i][2]<min_height) 
					min_height=points[j][i][2];
			}
		}
		fclose(mapa);
	}
	else {
		for (int i=0;i<sizex;i++)
			for (int j=0;j<sizey;j++) {
				points[i][j][0]=(mesh_x_length/(mesh_x_size-1))*i-(mesh_x_length/2);// wspolrzedna x
				points[i][j][1]=(mesh_y_length/(mesh_y_size-1))*j-(mesh_y_length/2);// wspolrzedna y
				points[j][i][2]=0;
			}
		min_height = 0;
		max_height = 1;
	}

/*	for (int i=0;i<int(sizex/15);i++)
			for (int j=0;j<int(sizey/15);j++) {
				float maxm=-1000;
				for (int k=0;k<15;k++)
					for (int l=0;l<15;l++) {
						if (points[j*15+k][i*15+l][2]>maxm)
							maxm = points[j*15+k][i*15+l][2];
					}
				for (int k=0;k<15;k++)
					for (int l=0;l<15;l++) {
							points[j*15+k][i*15+l][2] = maxm;
					}
			}*/

	//mapa pewnoœci pomiaru wysokoœci danych punktów terenu przez robota
	FILE *mapa2=fopen(mapa2_adres,"r");
	if (mapa2!=NULL) {
		loadDatData(mapa2, reliability_map);
		fclose(mapa2);
	}

	//mapa pomiaru wysokoœci danych punktów terenu przez robota
	FILE *mapa3=fopen(mapa3_adres,"r");
	if (mapa3!=NULL) {
		loadDatData(mapa3, measured_map);
		fclose(mapa3);
	}
}