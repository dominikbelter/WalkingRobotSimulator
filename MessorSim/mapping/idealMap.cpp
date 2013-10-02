#include "idealMap.h"
#include "../functions.h" 

CIdealMap::CIdealMap(COdeWorld* dynamicWorld)
{
	this->dynamicWorld=dynamicWorld;
	map_ideal=new double*[this->getXnumPoints()];
	for (int i=0; i<this->getXnumPoints(); i++)
	{
		map_ideal[i]=new double[this->getYnumPoints()];
		for (int j=0; j<this->getYnumPoints(); j++)
		{
			map_ideal[i][j]=dynamicWorld->ground->points[i][j][2];
		}
	}
}

CIdealMap::~CIdealMap(void)
{
	for (int i=0; i<this->getXnumPoints(); i++)
	{
		delete [] map_ideal[i];
	}
	delete [] map_ideal;
}

double CIdealMap::getMaxHeight()
{
	return dynamicWorld->ground->getMaxHeight();
}

float CIdealMap::computeK1(int size, int x, int y)
{
	float neutral_height = getHeight(x,y);
	float k1=0;
	for (int n=-size;n<=size;n++) {
		for (int m=-size;m<=size;m++) {
			k1+=neutral_height-getHeight(x+n,y+m);
		}
	}
	return k1;
}

// compute k2 coefficient
float CIdealMap::computeK2(int size, int x, int y)
{
	float neutral_height = getHeight(x,y);
	float k2=0;
	for (int n=-size;n<=size;n++) {
		for (int m=-size;m<=size;m++) {
			k2+=abs(neutral_height-getHeight(x+n,y+m));
		}
	}
	return k2;
}

double CIdealMap::getHeight(int x, int y)
{
	if ((x<0)||(y<0)||(x>=this->getXnumPoints())||(y>=this->getYnumPoints())) 
		return -1;
	else
		return map_ideal[x][y]+0.003;
}

float CIdealMap::ComputeSphericalVariance(int size, int x, int y)
{
	size = 1;
	float ** normal;
	int max = (size*2)*(size*2)*2;
	normal = new float *[max];
	for (int i=0;i<max;i++)
		normal[i]=new float[3];
	float ** vert = new float*[3];
	for(int i = 0; i < 3; i++)
		vert[i] = new float[3];
	int iter=0;
	for (int i=-size;i<size;i++){
		for (int j=-size;j<size;j++) {
			for (int l=0;l<3;l++){
				if (l<2){
					vert[0][l]=getHeight(x+i*4,y+j*4);//points[x+i*4][y+j*4][l];
					vert[1][l]=getHeight(x+i*4,y+j*4+4);//points[x+i*4+4][y+j*4+4][l];
					vert[2][l]=getHeight(x+i*4+4,y+j*4+4);//points[x+i*4+4][y+j*4][l];
				}
				else {
					vert[0][l]=getMaxSquareHeight(x+i*4, y+j*4, 3);//points[x+i*4][y+j*4][l];
					vert[1][l]=getMaxSquareHeight(x+i*4+4, y+j*4+4, 3);//points[x+i*4+1][y+j*4+1][l];
					vert[2][l]=getMaxSquareHeight(x+i*4+4, y+j*4, 3);//points[x+i*4+1][y+j*4][l];
				}
			}
			if (iter==max)
				int rr=4;
			calcNormal(vert, normal[iter]);
			iter++;
			for (int l=0;l<3;l++){
				if (l<2){
					vert[0][l]=getHeight(x+i*4,y+j*4);//points[x+i*4][y+j*4][l];
					vert[1][l]=getHeight(x+i*4,y+j*4+4);//points[x+i*4][y+j*4+4][l];
					vert[2][l]=getHeight(x+i*4+4,y+j*4+4);//points[x+i*4+4][y+j*4+4][l];
				}
				else {
					vert[0][l]=getMaxSquareHeight(x+i*4, y+j*4, 3);//points[x+i][y+j][l];
					vert[1][l]=getMaxSquareHeight(x+i*4, y+j*4+4, 3);//points[x+i][y+j+1][l];
					vert[2][l]=getMaxSquareHeight(x+i*4+4, y+j*4+4, 3);//points[x+i+1][y+j+1][l];
				}
			}
			calcNormal(vert, normal[iter]);
			iter++;
		}
	}
	float sum[3]={0,0,0};
	for (int i=0;i<max;i++){
		sum[0]+=normal[i][0];
		sum[1]+=normal[i][1];
		sum[2]+=normal[i][2];
	}
	float R = sqrt(pow(sum[0],2)+pow(sum[1],2)+pow(sum[2],2));
	float omega=1-(R/max);
	for (int i=0;i<max;i++)
		delete [] normal[i];
	delete [] normal;
	for(int i = 0; i < 3; i++)
		delete [] vert[i];
	delete [] vert;
	return omega;
}

double CIdealMap::getMaxSquareHeight(int x, int y, int size)
{
	float max=-1e13;
	if ((x<0)||(y<0)||(x>=this->getXnumPoints())||(y>=this->getYnumPoints())) return -1;
	else
		max = map_ideal[x][y];
	for (int i=-size;i<=size;i++){
		if ((x+i<0)||(y+i<0)||(x+i>=this->getXnumPoints())||(y+i>=this->getYnumPoints())) return -1;
		else if (map_ideal[x+i][y+i]>max)
			max = map_ideal[x+i][y+i];
	}
	return max;
}

double CIdealMap::getReliability(int x, int y)
{
		return dynamicWorld->ground->getReliability(x,y);
}


void CIdealMap::CalculateGlobalCoordinates(int x, int y, float * result)
{
	double centerX=dynamicWorld->ground->getCenterX();
	double centerY=dynamicWorld->ground->getCenterY();
	double factorX=dynamicWorld->ground->getScaleFactorX();
	double factorY=dynamicWorld->ground->getScaleFactorY();
	//if (getXnumPoints()%2==0)
	//{
		result[0]=(x)*factorX-centerX+(factorX/2);
	//}
	//else result[0]=(x)*factorX-centerX;

	//if (getYnumPoints()%2==0)
	//{
		result[1]=-(y*factorY-centerY+(factorY/2));
	//}
	//else result[1]=-(y*factorY-centerY);
}


void CIdealMap::CalculateGroundCoordinates(double x, double y, int* result)
{
	double centerX=dynamicWorld->ground->getCenterX();
	double centerY=dynamicWorld->ground->getCenterY();
	double factorX=dynamicWorld->ground->getScaleFactorX();
	double factorY=dynamicWorld->ground->getScaleFactorY();
	//if (getXnumPoints()%2==0)
	//{
		result[0]=dynamicWorld->ground->my_round((x+centerX-(factorX/2))/factorX);
	//}
	//else result[0]=my_round((x+centerX)/factorX);

	//if (getYnumPoints()%2==0)
	//{
		result[1]=dynamicWorld->ground->my_round((-y+centerY-factorY/2)/factorY);
	//}
	//else result[1]=my_round((-y+centerY)/getScaleFactorY());
}

void  CIdealMap::getScaleFactor(float *factor)
{
	factor[0]=getXsize()/getXnumPoints();
	factor[1]=getYsize()/getYnumPoints();
}

///wyznacza wspó³czynnik skali dla Y
double CIdealMap::getScaleFactorY()
{
	return dynamicWorld->ground->getScaleFactorY();
}

///wyznacza wspó³czynnik skali dla Y
double CIdealMap::getScaleFactorX()
{
	return dynamicWorld->ground->getScaleFactorX();
}

//get mesh_x_size
int CIdealMap::getXnumPoints()
{
	return dynamicWorld->ground->getXnumPoints();
}

///get mesh_y_size
int CIdealMap::getYnumPoints()
{
	return dynamicWorld->ground->getYnumPoints();
}

///get x size in meters
double CIdealMap::getXsize()
{
	return dynamicWorld->ground->getXsize();
}

///get y size in meters
double CIdealMap::getYsize()
{
	return dynamicWorld->ground->getYsize();
}

///wyznacza wspó³rzêdn¹ X œrodka terenu w jego wspolrzednych
double CIdealMap::getCenterX()
{
	return dynamicWorld->ground->getCenterX();
}

///wyznacza wspó³rzêdn¹ Y œrodka terenu w jego wspolrzednych
double CIdealMap::getCenterY()
{
	return dynamicWorld->ground->getCenterY();
}

//zaokr¹glanie
int CIdealMap::my_round(double x) 
{
	int i = (int) x;
	if (x >= 0.0) 
	{
		return ((x-i) >= 0.5) ? (i + 1) : (i);
	} 
	else 
	{
		return (-x+i >= 0.5) ? (i - 1) : (i);
	}
} 