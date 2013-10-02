#include "gridMap_filter.h"
#include <iostream>

#define a 3		// o ile zwiększa się wartoc w mapie wiarygonosci
#define k 7		// szerokosc okna filtracji
#define W 3		// indeks przemieszczenia robota
#define F 7		// prog warunku na gs
#define E 0.04          // prog warunku na gc

using namespace std;

void CGridMap_filter::FilterMap(SMapa** gridMap, int size_x, int size_y, const char* type)
{
    this->filteringMap = gridMap;
    this->size_x = size_x;
    this->size_y = size_y;

    if (!strcmp(type,"modifiedCAS")){
        ModifiedCAS();
    }
    else if (!strcmp(type,"CAS")){
        CAS();
    }

    gridMap = this->filteringMap;
}

void CGridMap_filter::ModifiedCAS( ){
	for(int i = (int)(k/2); i < size_x - (int)((float)k/2); i++)
		for(int j = (int)(k/2); j < size_y - (int)((float)k/2); j++){
			if( filteringMap[ i ][ j ].certainty == 0)
				filteringMap[ i ][ j ].elevation = WyznaczWMF(i, j);
		}
}

//funkcja realizujaca filtr CAS
void CGridMap_filter::CAS( ){
	for(int i = (int)(k/2); i < size_x - (int)((float)k/2); i++)
		for(int j = (int)(k/2); j < size_y - (int)((float)k/2); j++){

                    if( filteringMap[ i ][ j ].certainty <= W && !Wyznacz_gs(i, j) && !Wyznacz_gc(i, j))
                        filteringMap[ i ][ j ].elevation = 0;
                    if( Wyznacz_gs(i, j) && filteringMap[ i ][ j ].certainty == 0)
			filteringMap[ i ][ j ].elevation = WyznaczWMF(i, j);
                }
}

// funkcja wyznaczajaca indeks ciaglosci przestrzennej mapy wysokosci
bool CGridMap_filter::Wyznacz_gs(int x, int y){

    bool gs = true;
    float suma_h = 0;
    float temp   = 0;
    float max_h  = 0; 

    for(int i = x - (int)(k/2); i < x + (int)(k/2); i++)
	for(int j = y - (int)(k/2); j < y + (int)(k/2); j++){
		temp = filteringMap[ i ][ j ].elevation;

                if( temp < 0 ) temp *= (-1);

		suma_h += temp;
		if( max_h < temp ) max_h = temp;
        }

    suma_h /= max_h;

    if( suma_h < F ) gs= false; 

return gs;
}


// funkcja wyznaczajaca indeks ciaglosci przestrzennej mapy wiarygodnosci
bool CGridMap_filter::Wyznacz_gc(int x, int y){

    bool gc = true;
    int suma_c = 0;
    int max_c  = 0;
    float suma_c_norm = 0;
    float srednia   = 0;
    float wariancja = 0;

    // wyznaczenie sumy wiarygodnosci w oknie i wartosci maksymalnej
    for(int i = x - (int)(k/2); i < x + (int)(k/2); i++)
        for(int j = y - (int)(k/2); j < y + (int)(k/2); j++){
            suma_c += filteringMap[ i ][ j ].certainty;
            if( max_c < filteringMap[ i ][ j ].certainty ) max_c = filteringMap[ i ][ j ].certainty;
        }

    // badanie pierwszego warunku
    if( suma_c < 10 * W) gc = false;

    // wyznaczenie estymaty wartosci oczekiwanej w oknie znormalizowanej mapy wiarygodnosci
    srednia = suma_c / (float)max_c / (float)(k * k); // k * k - liczba probek

    for(int i = x - (int)(k/2); i < x + (int)(k/2); i++)
        for(int j = y - (int)(k/2); j < y + (int)(k/2); j++)
            suma_c_norm += pow( ((float)filteringMap[ i ][ j ].certainty / (float)max_c) - srednia, 2);

    // estymator nieobciazony wariancji
    wariancja = suma_c_norm / (float)(k * k - 1);

    if( wariancja > E ) gc = false;

    return gc;
}

//funkcja wyznaczajaca wyjscie filtru Weighted Median Filter w komorce mapy o wspolrzednych "x" i "y"
float CGridMap_filter::WyznaczWMF(int x, int y){ // maska filtru 5x5

	float* wartosci = new float[ 98 ];
	float wyjscie   = 0;
	int id = 0;
	for(int i = x - 3; i <= x + 3; i++)
		for(int j = y - 3; j <= y + 3; j++){
			wartosci[ id ] = filteringMap[ i ][ j ].elevation;
			id++;
			if( !((i == x && j != y - 3 && j != y + 3) ||
				  (j == y && i != x - 3 && i != x + 3)))
			{
				wartosci[ id ] = filteringMap[ i ][ j ].elevation;
				id++;
			}
		}

	Sortuj(wartosci, id);
	wyjscie = wartosci[ (int)((float)id/2) ];

    delete wartosci;
    return wyjscie;

}

// funkcja sortujaca tablice "tab" o dlugosci "length"
void CGridMap_filter::Sortuj(float* tab, int length){

	bool flaga;
	do{
		flaga = false;
		for(int i = 0; i < length - 1; i++){
			if( tab[ i ] > tab[ i + 1 ]){
				float temp = tab[ i ];
				tab[ i ] = tab[ i + 1 ];
				tab[ i + 1 ] = temp;
				flaga = true;
			}
		}
	}while( flaga );

}

