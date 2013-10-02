#pragma once
#include "../ODE/OdeWorld.h"

class CIdealMap
{
public:
	CIdealMap(COdeWorld* dynamicWorld);
	~CIdealMap(void);
public:
	// zwraca maksymaln¹ wysokoœæ
	double getMaxHeight();
	// compute k1 coefficient
	float computeK1(int size, int x, int y);
	// compute k2 coefficient
	float computeK2(int size, int x, int y);
	///zwraca wysokoœæ gruntu w danym punkcie 
	double getHeight(int x, int y);
	/// oblicz wspolczynniki jakosci wariancja sferyczna
	float ComputeSphericalVariance(int size, int x, int y);
	//get max height from the square
	double getMaxSquareHeight(int x, int y, int size);
	///zwraca pewnoœæ wysokoœci gruntu w danym punkcie 
	double getReliability(int x, int y);
	///przelicza wspolrzedne "powierzchni" na wspolrzedne robota
	void CalculateGlobalCoordinates(int x, int y, float * result);
	///przelicza wspolrzedne robota na wspolrzedne terenu
	void CalculateGroundCoordinates(double x, double y, int * result);
	///wyznacza wspó³czynnik skali dla X
	void getScaleFactor(float *factor);
	///wyznacza wspó³czynnik skali dla Y
	double getScaleFactorY();
	///wyznacza wspó³czynnik skali dla X
	double getScaleFactorX();
	///get mesh_x_size
	int getXnumPoints();
	///get mesh_y_size
	int getYnumPoints();
	///get length (x) in meteres
	double getXsize();
	///get length (y) in meters
	double getYsize();
	///wyznacza wspó³rzêdn¹ Y œrodka terenu w jego wspolrzednych
	double getCenterX();
	///wyznacza wspó³rzêdn¹ Y œrodka terenu w jego wspolrzednych
	double getCenterY();
	//funkcja do zaokraglania
	int my_round (double x);
public:
	//tablica zawieraj¹ca wysokoœæ punktów okreœlonych wspó³rzêdnymi x i y
	double ** map_ideal;
	COdeWorld* dynamicWorld;
};