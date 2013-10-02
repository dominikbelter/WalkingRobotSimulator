#include "foot_rc.h"

CFoot_RC::CFoot_RC(void)
: contact(false)
{
}

CFoot_RC::~CFoot_RC(void)
{
}

/// ustawia stycznik false wylaczony, true wlaczony
void CFoot_RC::setContact(bool value)
{
	contact = value;
}

/// pobiera stan stycznika
bool CFoot_RC::getContact(void)
{
	return contact;
}
