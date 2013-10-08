#ifndef _LocalMap_H
#define _LocalMap_H

/*
*
*
* Plik zawiera definicję klasy CLocalMap.
* 
* Dominik Belter, Adam Łopatowski, Dawid Rosiński
* date 2010.06
*
*/

#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pureScan.h"
#include "../functions.h"
#include "mapStruct.h"
#include "MapFilter/gridMap_filter.h"
#include "laser_rangefinder.h"
#include <iostream>
#include <fstream>
//#include "cxcore.h"
//#include "highgui.h"
//#include "cv.h"
#include <gl\glut.h>

#define W 3 //stała o którą zwiększana jest mapa wiarygosności

//using namespace qrk;
using namespace std;



/*  
* klasa CLocalMap - lokalna mapa otoczenia robota
*
*/
class CLocalMap : private CPureScan, public CGridMap_filter
{
	private:

	public:

		/*Konstruktor klasy CLocalMap
		 * device - nazwa urzadzenia do ktorego podlaczony jest skaner.
		 * size_x - ilosc komorek w x
		 * size_y - ilosc komorek w y
		 * raster_x - rozmiar komorki w x
		 * raster_y - rozmiar komorki w y
		 * source - tryb: 0 - dane ze skanera, 1 - dane z pliku
		*/
		CLocalMap(int size_x, int size_y, float raster_x, float raster_y, bool source, CIdealMap* map);

		// destruktor klasy CLocalMap
		~CLocalMap(void);

                // Tworzy mape
                void CreateMap();

                // Zeruje mape
                void ClearMap();
		
                // akutalizująca mapy na podstawie współrzędnych xyz w pomiaru
                void UpdateMap(float x, float y, float atrybut,float max_err);

                // przesuniecie mapy 
                void MoveMap( float dx, float dy, float dkat_Z );

                // obrot mapy 
                void RotateMap(float kat_X, float kat_Y, float kat_Z);

                //wyznacza nachylenia w kierunkach "x" i "y"
                void CalculateInclination();

                // zwraca wskaźnik do mapy
                SMapa** GetMapPointer( );

                // zwraca wysokość w (x,y)
                float GetElevation(int x, int y);

                // zwraca wiarygodność w (x,y)
                float GetCertainty(int x, int y);

                // zwraca nachylenie_X w (x,y)
                float GetInclination_X(int x, int y);

                // zwraca nachylenie_Y w (x,y)
                float GetInclination_Y(int x, int y);

		// funkcja główna: wykonuje skan i tworzy mape
		void makeScan(float angle_min, float angle_max, int scan_no, float x,float y,float z,float alpha,float beta,float gamma);

                //
		void Calculate3DMap(int no, float kat_X, float kat_Y, float kat_Z, float deltaY);

                // Wyznacza współrzędne globalne, tórjwymiarowe; zwaraca wartosc globalna pochylanie skanera "gamma"
		float CalculateGlobalCoordinates(float* punkty, float kat_Y, float kat_X, float Alfa);

                //Wyznacz wysokość robota
                float CalculateRobotHight(float *s_f, float kat_X, float kat_Y);

		// makes 4x4 transform matrix
        CPunctum makeTransformMatrix(const char * type, float value);
		
		//get max height from the square
		double getMaxSquareHeight(int x, int y, int size);

		/// oblicz wspolczynniki jakosci wariancja sferyczna
		float ComputeSphericalVariance(int size, int x, int y);

		///get mesh_x_size
		int getXnumPoints();

		///get mesh_y_size
		int getYnumPoints();
		
		///get x size in meters
		float getXsize();
		
		///get y size in meters
		float getYsize();

		///wyznacza wspolczynnik skali dla X
		float getScaleFactorX();

		///wyznacza wspolczynnik skali dla Y
		float getScaleFactorY();

		///wyznacza wspolrzedne X srodka terenu w jego wspolrzednych
		float getCenterX();

		///wyznacza wspolrzedne Y srodka terenu w jego wspolrzednych
		float getCenterY();
		
		///get x raster size in meters
		float getXRastersize();

		///get y raster size in meters
		float getYRastersize();
		
		/// save to file
		void saveMap2file(char * nazwa_pliku);

		///  przelicza wspolrzedne terenu na wspolrzedne lokalne
		void CalculateLocalCoordinates(int x, int y, float res[2]);

		///przelicza wspolrzedne robota na wspolrzedne terenu
		void CalculateGroundCoordinates(double x, double y, int * result);
		
		/// funkcja do zaokraglania
		int my_round (double x);
		
		// save map to .dat
		void saveMap2dat(char * nazwa_pliku);

		// save robot state
		void saveRobotState2file(float x,float y,float z,float alpha,float beta,float gamma);

		// plik z danymi o stanie robota
		FILE * robotState;
		
		// read robot state
		void readRobotState(float * x,float * y,float * z,float * alpha,float * beta,float * gamma);

		// plik o stanie robota
		FILE * readRobot;

		//void setAngles(float kat_X, float kat_Y);

		// wyznacza delta_z_max
		float CalculateMaxErr(float kat_X, float kat_Y, float kat_rad, float Alfa, float gamma, float deltaY, float l);

		// wyznacza delta_y na podstawie obrotu
		float CalculateDeltaY_afterRotate(float x_s, float y_s, float kat_Z);

		void DrawLocalMap(void);

		// mapa
		SMapa** grid_map;


	private:
		
		// ilosc komorek w x
		int size_x;
		
		// ilosc komorek w y
		int size_y;
		
		// rozmiar komorki w x
		float raster_x;
		
		// rozmiar komorki w y
		float raster_y;

                /// zmienna wykorzystywana przy przyrostach wspolrzednej x
                float  increment_x;

                /// zmienna wykorzystywana przy przyrostach wspolrzednej y
                float increment_y;

                // Wspołrzędne stopy
                SFoot*	foot;
		
		// kat pochylenia platformy
		float imu_rot_x;
		// kat pochylenia platformy
		float imu_rot_y;

		// wysokość robota
		float robot_dz;

		// obiekt klasy CConfig
//		CConfig config;

		//pozycja skanera w ukladzie robota
		float scanner_x;
		float scanner_y;
		float scanner_z;

		// orientacja skanera w układzie robota
		float scanner_alpha;
		float scanner_beta;
		float scanner_gamma;

		// zmienna trybu pracy: 0 - dane ze skanera, 1 -  dane z pliku
		bool source;

		// 0 - imu wył; 1 - imu wł
  		bool _imu;
		
		// 0 -> aplikacja nie uwzglednia wysokosci na jakiej stoi robot; 1 -> uwzglednia wysokosc
		bool h_refParam;

		// zmienna pomocnicza wykorzystywana gdy source == 1 && h_refParam == 0
		float robot_dzTemp;
		
		//dostep do kamery
//		RPCCallerCamera* rpccaller_cam;
		
		//numer obrazka
		int image_no;

		float draw_robot_position[3];
//		float draw_robot_orientation[3];
};

#endif
