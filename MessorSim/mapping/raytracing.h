#include "idealMap.h"
#include "../math/punctum.h"
#include <vector>

using namespace std;

class CRayTracing
{
public:
	CRayTracing(void);
	~CRayTracing(void);
	//szukanie punktu przeciêcia promienia sensora z gruntem
	bool groundTrace(CIdealMap* map, float* position, float* sensor_position, float* orientation, float* sensor_orientation, int range, vector<long>* data, vector<float>* kat_radiany);
private:
	//macierze do obliczeñ przekszta³ceñ geometrycznych
	//pozycja robota
	CPunctum robot_pos;
	//macierz obliczeñ tymczasowych
	CPunctum temp;
	//aktualnie sprawdzany punkt promienia (macierz wêdruj¹ca wzd³u¿ promienia a¿ do przeciêcia z gruntem)
	CPunctum ray;
	//pozycja sensora
	CPunctum sensor_pos;
	int coordinates[2];
};