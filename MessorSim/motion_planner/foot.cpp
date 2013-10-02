#include "foot.h"

CFoot::CFoot(void)
: contact(false)
{
}

CFoot::~CFoot(void)
{
}

/// ustawia stycznik false wylaczony, true wlaczony
void CFoot::setContact(bool value)
{
	contact = value;
}

/// pobiera stan stycznika
bool CFoot::getContact(void)
{
	return contact;
}
