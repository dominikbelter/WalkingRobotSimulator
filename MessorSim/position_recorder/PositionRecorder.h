#pragma once
#include "../math/punctum.h"
#include <iostream>
#include <list>

class CPositionRecorder
{
public:
	CPositionRecorder(void);
	~CPositionRecorder(void);
	/// dopisanie macierzy na koniec listy
	void savePunctum (CPunctum punkt);
	/// dopisanie macierzy do listy
	void insertPunctum (CPunctum punkt);
	/// dopisanie punktu do listy
	void savePosition (double x, double y, double z);
	/// dopisanie punktu do listy - tylko orientacja
	void saveOrientation (double alpha, double beta, double gamma);
	/// dopisanie punktu do konca listy na podstawie wspolrzednych i katow eulera
	void savePositionAndOrientation (double x, double y, double z,double alpha, double beta, double gamma);
	/// dopisanie punktu do listy na podstawie wspolrzednych i katow eulera
	void insertPositionAndOrientation (double x, double y, double z,double alpha, double beta, double gamma);
	/// rysuje wykres
	void plot(double red, double green, double blue, double thickness, const char marker=0);
	// marker='o' - znacznik w postaci sfery
	/// ustawia opoznienie
	void setDelay(int value);
	/// save to file
	void savePosition2file(const char * filename);
	/// pobierz ostatnio zapisany punkt
	void getLastPoint(float *x, float *y, float *z);
	/// pobierz ostatnio zapisany punkt
	CPunctum getLast(void);
	/// pobierz pierwszy element
	bool getFirst(CPunctum *point);
	/// pobierz pierwszy element
	bool getNext(CPunctum *point);
	/// pobierz kolejny element element - z przesunieciem
	bool getNextShift(CPunctum * point, int shift);
	/// wstaw element
	bool insertAfter(CPunctum * point);
	/// cofa iterator o 'shift' pozycji
	bool reverse(int shift);
	//usuwa ostatnie 'shift' elementow z listy
	void eraseLast(int shift);
	/// pobierz aktualny element element
	bool getElement(CPunctum * point);
	/// aktualizuj wartosc aktualnego elementu
	void updateElement(CPunctum * point);
	/// aktualizuj wartosc aktualnego elementu
	void updatePreviousElement(CPunctum * point);
	/// ustawia iterator na poczatek listy
	void setBegin();
	/// czysci zawartosc
	void clear();
	/// odwraca kolejnosc zawartosci
	void reverseOrder();
	/// pobierz liczbe elementow
	int getSize();
	//semafor dla zlikwidowania bledu odczytu listy przy jednoczesnym rysowaniu i zapisywaniu z niej do pliku
	bool plot_stop;

	// zmienne
private:
	/// lista do przechowywania obiektow - macierzy jednorodnych okreslajacych pozycje i orientacje
	std::list<CPunctum> container;
	/// delay how many step are not saved
	int delay;
	/// delay iterator
	int delay_iterator;
	/// recorder iterator 
	std::list<CPunctum>::iterator rec_it;
};
