#ifndef MAP_STRUCT
#define MAP_STRUCT

// Struktura zawierająca dane o mapie rastrowej 
struct SMapa{

	float elevation;
	int   certainty;
	float inclination_x;
	float inclination_y;

};

struct SFoot{
	int x;
	int y;
};

#endif
