#pragma once
#include "../math/punctum.h"

class CFoot_RC
{
public:
	CFoot_RC(void);
	~CFoot_RC(void);
	/// ustawia stycznik false wylaczony, true wlaczony
	void setContact(bool value);
	/// pobiera stan stycznika
	bool getContact(void);

	//zmienne
private:
	/// wspolrzedne stopy (x,y,z) - macierz jednorodna
	CPunctum foot_position;
	/// stan stycznika (true - zwarty, flase - rozwarty)
	bool contact;

};
