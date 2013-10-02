#pragma once
#include <stdlib.h>
#include <gl\glut.h> // glut.h includes gl.h and glu.h
#include <stdio.h>
#include <ode/ode.h>    // ode library header

#define ground_surface 100

class CGround
{
public:
	CGround(void);
	/// length_x - surface length in x axis [m], length_y - surface length in y axis [m]
	/// size_x - surface size in x axis (number of points), size_y - surface size in y axis (number of points)
	CGround(double length_x, double length_y, int size_x, int size_y, double max_height);
public:
	~CGround(void);
public:
	//wczytuje mape z pliku
	void readCustMap(int sizex, int sizey);
	/// draw lines - siatka
	void DrawGrid(void);
	///losuje liczbe rzeczywista z zakresu <min,max>
	double randDouble(double min, double max); 
	/// losuje punkty siatki (wysokosc)
	void randMesh(void); 
	///get mesh_x_size
	int getXnumPoints();
	///get mesh_y_size
	int getYnumPoints();
	///get length (x) in meteres
	double getXsize();
	///get length (y) in meters
	double getYsize();
	///wyznacza wspó³czynnik skali dla X
	void getScaleFactor(float *factor);
	///wyznacza wspó³czynnik skali dla Y
	double getScaleFactorY();
	///wyznacza wspó³czynnik skali dla X
	double getScaleFactorX();
	///initialize list
	void initializeList(int * indexes, dVector3 * triVert);
	/// draw mesh
	void DrawMesh(int a[6], int b[6], int foot_groundx_def[6], int foot_groundy_def[6] ,int targetx, int targety); 
	/// return table of indexes (triangle points)
	void returnIndexes(int * indexes); 
	/// return triangle coordinates
	void returnTriVert(dVector3 * triVert);
	///przelicza wspolrzedne robota na wspolrzedne terenu
	void CalculateGroundCoordinates(double x, double y, int * result);
	///wyznacza wspó³rzêdn¹ Y œrodka terenu w jego wspolrzednych
	double getCenterX();
	///wyznacza wspó³rzêdn¹ Y œrodka terenu w jego wspolrzednych
	double getCenterY();
	//funkcja do zaokraglania
	int my_round (double x);
	//zwraca maksymaln¹ wysokoœæ
	double getMaxHeight();
	///zwraca wysokoœæ gruntu w dany punkcie 
	double getReliability(int x, int y);
	//load *.dat map file
	void loadDatData(FILE * file, double *** data_structure);

// zmienne
private:
	///surface length in x axis
	double mesh_x_length; 
	///surface length in y axis
	double mesh_y_length; 
	///surface size in x axis (number of points)
	int mesh_x_size; 
	///surface size in y axis (number of points)
	int mesh_y_size; 
	///min height of the ground
	float min_height;
	///max height of the ground
	double max_height; 
	///reliability map
	double *** reliability_map;
	/// measured map
	double *** measured_map;
public :
	/// tablica z punktami (x,y,z) reprezentujaca uksztaltowanie terenu
	double *** points; 
};
