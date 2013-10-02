#ifndef _gridMap_filter_H
#define _gridMap_filter_H


#include <string.h>
#include <math.h>
//#include <iostream>

#include "../mapStruct.h"


class CGridMap_filter
{
	private:
            void ModifiedCAS();

            void CAS();

            // funkcja wyznaczajaca indeks ciaglosci przestrzennej mapy wysokosci
            bool Wyznacz_gs(int x, int y);

            // funkcja wyznaczajaca indeks ciaglosci przestrzennej mapy wiarygodnosci
            bool Wyznacz_gc(int x, int y);

            //funkcja wyznaczajaca wyjscie filtru Weighted Median Filter w komorce mapy o wspolrzednych "x" i "y"
            float WyznaczWMF(int x, int y); // maska filtru 5x5

            // funkcja sortujaca tablice "tab" o dlugosci "length"
            void Sortuj(float* tab, int length);

            SMapa** filteringMap;
            int size_x;
            int size_y;

        
        public:
            void FilterMap(SMapa** gridMap,int size_x, int size_y,  const char * type);
};
#endif
