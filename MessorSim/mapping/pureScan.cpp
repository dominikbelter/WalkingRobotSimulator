#include "pureScan.h"
#include <iostream>

using namespace std;

// Konstruktor klasy CPureScan, source okresla tryb pracy: 0 - dane ze skanera, 1 - dane z pliku
CPureScan::CPureScan(bool source, CIdealMap* map) 
{
//	LoadAngles();

	if(source == 1) //jesli wybrany jest tryb offline to wczytujemy pomiary z txt
		LoadScansFromFile();
 //   else
 //   {
	//	char device1[20], dev[20];
	//	strcpy(device1,"/dev/ttyACM");
		//for (int i=0;i<32;i++)
		//{
		//	sprintf(dev, "%s%d",device1,i);
		//	printf("%s\n",dev);
  //	if (urg.connect(dev)) {
  //  		//printf("UrgCtrl::connect: %s\n", urg.what());
  //    		printf("Nawiązano połączenie ze skanerem ...");
  //    		//printf ("\033[%d;%d;%dm", 0, 37,37);
  //     		printf("\t %s \n", urg.what());
  //     		//printf ("\033[%d;%d;%dm", 0, 30,30);
		//printf(" \n");
//			i=32;
//		}
//		else
//		{
//       		printf ("\033[%d;%d;%dm", 0, 31,31);
//       		printf ("BŁĄD: ");
//       		//printf ("\033[%d;%d;%dm", 0, 30,30);
//       		printf("Nie udało się nawiązać połączenia ze skanerem! \n");
//       		//printf ("\033[%d;%d;%dm", 0, 37,37);
//       		printf("Sprawdź połączenie lub nazwę urządzenia w pliku konfiguracyjnym.\n\n");
//       		//printf ("\033[%d;%d;%dm", 0, 30,30);
////        		exit(0);
//		}
//	}
//}
  //std::cout << "running....\n";
  	
    this->clear();
  
   //resetowanie pliku
   skanPlik = fopen ("pliki/wynik/skany.txt","w");
   fclose(skanPlik);

   this->map=map;
   hrf=new CHokuyoRangefinder(map);
  
}

// destruktor klasy CPureScan
CPureScan::~CPureScan()
{
}

// Wypisuje zawartość klasy CPureScan
void CPureScan::Show()
{
	for(int i=0;i<726;i++)
	{
		cout<<i<<": "<<scan_x[i]<<" "<<scan_y[i]<<" "<<"\n";
	}
}

// Czysci zawartość klasy CPureScan
void CPureScan::clear()
{
	for(int i=0;i<726;i++)
	{
		scan_x[i]=scan_y[i]=scan_l[i]=scan_kat[i]=0;
	}
}

// Zwraca położenie względem osi x
float CPureScan::Get_X(int i)
{
	return scan_x[i];
}

// Zwraca położenie względem osi y
float CPureScan::Get_Y(int i)
{
	return scan_y[i];
}

// Zwraca zmiarzoną odleglosc
float CPureScan::Get_l(int i)
{
	return scan_l[i];
}

// Zwraca wartosc kata
float CPureScan::Get_kat_rad(int i)
{
	return scan_kat[i];
}

/* 
 *  Funkcja pobierająca dane ze skanera laserowego lub z pliku
 *  min - kat minimalny  
 *  max - kat maksymalny
 *  scan_no - liczba skanow
 *  source - tryb pracy: 0 - dane ze skanera, 1 - dane z pliku
*/
int CPureScan::scan(float min, float max, int scan_no, bool source)
{
	int k=0;
	this->clear();//clear x and y arrays
//      urg.setCaptureRange(urg.rad2index(min), urg.rad2index(max));
	for (int i = 0; i < scan_no; i++) 
	{
		long timestamp = 0;
		vector<long> data;
		
		int n;
        if(source == 1)
		{
			data = SingleScanFromFile();
			n = data.size();
		}

		else 
		{
			map->dynamicWorld->robotODE.imu.getIMUposition(robot_position);
			map->dynamicWorld->robotODE.imu.getIMUorientation(robot_orientation);
			hrf->rangefinding(min,max,robot_position,robot_orientation,&data,&kat_radiany);
			n=data.size();
		}


	

        //zapis skanu do pliku
//		SaveScan2File(data);
	
		FilterScan(data, "max_angle");
		FilterScan(data, "dist_correct");
       
	// inne rodzaje grupowania
        //FilterScan(data, "odleg");
        //FilterScan(data, "stosun");

		float pos_x,pos_y;
		for (int j = 0; j < n; j++) 
		{
			pos_x = (int)((data[j])* sin(-kat_radiany[j]));        
			pos_y = (int)((data[j])* cos( kat_radiany[j]));

			if(data[j]>275)
			{
				scan_l[k] = data[j];
				scan_kat[k] = kat_radiany[j];
				scan_x[k] += (0.1*pos_x/(float)scan_no);  
				scan_y[k] += (0.1*pos_y/(float)scan_no);  
				k++;
			}
		}
	}
	return k;
}

// Wczytuje skany z pliku "skany.txt"
int CPureScan::LoadScansFromFile()
{
	//printf("\nWczytuje skany.. \n");
	ifstream file ("pliki/zrodlo/skany.txt");

	string line;
	allData.clear();

        while ( getline(file, line) ){
            vector < long > data;
            long value;
            istringstream iss(line);

            while (iss >> value){
                data.push_back(value);
            }
            allData.push_back(data);
        }

        scan_nr = 0;
        scan_size = allData.size();

	if(scan_size == 0) {
		printf("Błąd odczytu pliku 'skany.txt'. \n");
		printf("Aplikacja zastanie zamknięta .. \n");
		exit(0);
	}
	
	printf("Plik 'skany.txt' wczytany poprawnie ..\n");
	printf("Ilość wykonanych skanów, którą należy podać do aplikacji 'klien_mapping' = %d \n",scan_size);

   return allData.size();
}

// Pobiera kolejny skan z pliku "skany.txt"
std::vector<long> CPureScan::SingleScanFromFile(){

	std::vector <long> singleScan;

	if(scan_nr < scan_size)
	{
            for ( std::vector < long > :: size_type j = 0, length = allData[scan_nr].size(); j < length; ++j)
			{
				singleScan.push_back(allData[scan_nr][j]);
			}
        }
	else
	{
	    printf("Wszystkie skany zostały przetworzone.\n"); 
	    printf("Sprawdź czy pliki 'skany.txt' oraz 'StanRobota.txt' dotycza tego samego pomiaru. \n");
	    exit(0);
	}

	scan_nr++;

	return singleScan;
}

// Wczytuje kat w radianach z pliku "kat_rad.txt"
void CPureScan::LoadAngles(){

	std::ifstream file("kat_rad.txt");
	std::string line;

        while ( getline(file, line) )
        {
            float value;
            std::istringstream iss(line);
            while (iss >> value)
            {
                kat_radiany.push_back(value);
            }
        }
	
	if(kat_radiany.size()==726){
		printf("\nPlik 'kat_rad.txt' wczytany poprawnie ..\n");
	}
	else{
		printf("\nBłąd w wczytaniu pliku 'kat_rad.txt'.\n");
		printf("Skopiuj do folderu 'mapping' orginalny plik 'kat_rad.txt'\n");
		printf("Aplikacja zosanie zamknięta ..\n");
		exit(0);
	}
}

//zapis skanu do pliku tekstowego
void CPureScan::SaveScan2File(vector<long> pomiar)
{
			skanPlik = fopen ("pliki/wynik/skany.txt","a");

			for(int i=0; i<pomiar.size(); i++)
            		{
		                fprintf(skanPlik,"%u ", pomiar[i]);
            		}
            		fprintf(skanPlik,"\n");

			fclose (skanPlik);
}