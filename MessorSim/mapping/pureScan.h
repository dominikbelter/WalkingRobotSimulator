#ifndef _pureScan_H
#define _pureScan_H

/*
* Plik zawiera definicję klasy Scan.
* 
* date 2009.18.08
* author: Krzysztof Krzyśków (przerzucenie do klasy i rozbudowa funkcjonalnosci - Dominik Belter)
*
* date 2010.06.11
* wprowadzenie zmian i rozbudowa - Dawid Rosiński
* 
*/

#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include "idealMap.h"

//#include "../URG04lx/cpp/all/UrgCtrl.h"
#include "MapFilter/pureScan_filter.h"
#include "laser_rangefinder.h"


//using namespace qrk;
using namespace std;

/*
* Klasa odczytuje dane ze skanera laserowego URG04lx lub z pliku "skany.txt" w zależności od wybranego trybu. 
* Jednocześnie zapisuje dane do pliku "skany.txt"
*
* Klasa zawiera dane potrzebne do reprezentacji pomiarów w trójwymiarowej przestrzeni kartezjańskiej. 
* Dane opisują punkty w przestrzeni 2D.
*
*/
class CPureScan: public CPureScan_filter
{
private:
		// współrzedna x pomiaru o indeksie [i]
		float scan_x[726];

		// współrzedna y pomiaru o indeksie [i]
		float scan_y[726];
	
		// wartosc pomiaru o indeksie [i]
		float scan_l[726];
		
		// wartosc kata odchylenia wiazki pomiaru o indeksie [i]
		float scan_kat[726];
		
		// obiekt klasy skanera
//		UrgCtrl urg;
 		
		// ilosc skanow zapisanych w pliku "skany.txt"
		int scan_size;

		// numer przetwarzanego skanu z pliku "skany.txt"		
		int scan_nr;                

		// zmienna zawierajaca wartosci katow w radianach 		
		std::vector<float> kat_radiany;

		// zmienna do ktorej wczytywane sa wszystkie dane z pliku "skany.txt"               
		std::vector < std::vector < long > > allData; 

		// obiekt klasy CPureScan_filter
                CPureScan_filter filter;

		// Wczytuje skany z pliku "skany.txt"		
		int LoadScansFromFile();

		// Pobiera kolejny skan z pliku "skany.txt" 
		vector<long> SingleScanFromFile();
	
		// Wczytuje kat w radianach z pliku "kat_rad.txt"
		void LoadAngles(); 

		float robot_position[3];

		float robot_orientation[3];
			
		CHokuyoRangefinder* hrf;

	public:

		CIdealMap* map;

		// Konstruktor klasy CPureScan, source okresla tryb pracy: 0 - dane ze skanera, 1 - dane z pliku
		CPureScan(bool source, CIdealMap* map);

		// destruktor klasy CPureScan
		~CPureScan(void);
		
		// Wypisuje zawartość klasy CPureScan
		void Show();

		// Czysci zawartość klasy CPureScan
		void clear();
		
		// Zwraca położenie względem osi x
		float Get_X(int i);

		// Zwraca położenie względem osi y
		float Get_Y(int i);

                // Zwraca zmiarzoną odleglosc
		float Get_l(int i);

                // Zwraca wartosc kata
		float Get_kat_rad(int i);
		 
		/* 
		 *  Funkcja pobierająca dane ze skanera laserowego lub z pliku
		 *  min - kat minimalny  
		 *  max - kat maksymalny
		 *  scan_no - liczba skanow
		 *  source - tryb pracy: 0 - dane ze skanera, 1 - dane z pliku
		*/
		 int scan(float min, float max, int scan_no, bool source);

		//zapis skanu do pliku tekstowego
		void SaveScan2File(vector<long> pomiar);
		 
		// plik do którego zapisywane są skany		
		FILE * skanPlik;
};

#endif
