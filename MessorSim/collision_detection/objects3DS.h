#ifndef H_OBJECTS3DS
#define H_OBJECTS3DS

//#include <gl\gl.h>
//#include <gl\glu.h>
#include <stdlib.h>
#include <gl\glut.h>

//#include <gl\glaux.h>
#include "types.h"
#include "3dsloader.h"
#include "coldet.h"
#include "terrain.h"

class CObjects3DS {
public:
	CObjects3DS();
public:
	~CObjects3DS();
public:
	
	void Object3DS(int obj_qty);
	char ObjLoad(char *p_object_name);
	void CollisionModels(void);
	void TerrainCollisionModels(void);


	obj_type object[MAX_OBJECTS];
	int obj_qty;

	CollisionModel3D* stopka_1_model;
	CollisionModel3D* stopka_2_model;
	CollisionModel3D* stopka_3_model;
	CollisionModel3D* stopka_4_model;
	CollisionModel3D* stopka_5_model;
	CollisionModel3D* stopka_6_model;

	CollisionModel3D* segment_I_model_1;

	CollisionModel3D* segment_I_model_2;

	CollisionModel3D* segment_I_model_3;

	CollisionModel3D* segment_I_model_4;

	CollisionModel3D* segment_I_model_5;

	CollisionModel3D* segment_I_model_6;


	CollisionModel3D* segment_II_model_1;

	CollisionModel3D* segment_II_model_2;

	CollisionModel3D* segment_II_model_3;

	CollisionModel3D* segment_II_model_4;
	CollisionModel3D* segment_II_model_5;
	CollisionModel3D* segment_II_model_6;

	CollisionModel3D* korpus_dol_model;
	CollisionModel3D* korpus_gora_model;

	CollisionModel3D* przegub_typu_C_1;
	CollisionModel3D* przegub_typu_C_2;
	CollisionModel3D* przegub_typu_C_3;
	CollisionModel3D* przegub_typu_C_4;
	CollisionModel3D* przegub_typu_C_5;
	CollisionModel3D* przegub_typu_C_6;

	CollisionModel3D* teren_1_model;
	CollisionModel3D* teren_2_model;

	CTerrain ground;
};
#endif // 