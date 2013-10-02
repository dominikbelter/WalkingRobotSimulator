#include "laser_rangefinder.h"

CRayTracing raytracer;

CRangefinder::CRangefinder(void)
{
}

CRangefinder::~CRangefinder(void)
{
}

CHokuyoRangefinder::CHokuyoRangefinder(CIdealMap* map)
{
	this->map=map;

	//ustawienie parametrów czujnika Hokuyo
	sensor_position[0] = 0; //pozycja sensora w uk³adzie robota w osi: x
	sensor_position[1] = 0.202;//y
	sensor_position[2] = 0.15;//z

	sensor_orientation[0] = -0.785398; //-45 stopni - pochylenie sensora w uk³adzie robota wokó³ osi: x (pitch)
	sensor_orientation[1] = 0; //y (roll)
	sensor_orientation[2] = 0; //z (yaw)

	range=1500; //1.5 m
	resolution = 0.006283185307; //0.36 stopnia
}

CHokuyoRangefinder::~CHokuyoRangefinder(void)
{
}

void CHokuyoRangefinder::rangefinding(float min, float max, float* robot_position, float* robot_orientation, vector<long>* data, vector<float>* kat_radiany)
{
	//ustawienie wstêpnej orientacji sensora - sk¹d zaczynamy pomiar wzglêdem k¹ta 0 le¿¹cego na osi y robota (orientacja ujemna - robot "patrzy" sensorem w prawo, dodatnia - w lewo)
	float sensor_scan_orientation[3];
	sensor_scan_orientation[0]=sensor_orientation[0];
	sensor_scan_orientation[1]=sensor_orientation[1];
	sensor_scan_orientation[2]=sensor_orientation[2]+min;
	//czyszczenie wektorów danych wynikowych
	while (data->size()>0)
		data->pop_back();
	while (kat_radiany->size()>0)
		kat_radiany->pop_back();
	//ustalenie liczby dokonanych pomiarów na podstawie okreœlonego maksymalnego i minimalnego k¹ta oraz rozdzielczoœci pomiaru dla danego sensora
	float scans = (max-min)/resolution;
	//pomiary
	for (int i=0; i<scans; i++)
	{
		//obracanie sensora w osi z o k¹t okreœlony jako jego rozdzielczoœæ
		sensor_scan_orientation[2]=sensor_scan_orientation[2]+resolution;
		if(raytracer.groundTrace(map, robot_position, sensor_position, robot_orientation, sensor_scan_orientation, range, data, kat_radiany))
		{//je¿eli pomiar wykonany zosta³ prawid³owo (nie przekroczono zasiêgu sensora, do wektora 'data' zapisany zosta³ pomiar odleg³oœci) dodajemy do wektora 'kat_radiany' pomiar k¹ta
			kat_radiany->push_back(((i+1)*resolution)+min);
		}
	}
}