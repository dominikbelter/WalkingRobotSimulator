#include "objects3DS.h"
#include "../functions.h"

CObjects3DS::CObjects3DS(){
	obj_qty =0;
	stopka_1_model	= newCollisionModel3D();
	stopka_2_model	= newCollisionModel3D();
	stopka_3_model	= newCollisionModel3D();
	stopka_4_model	= newCollisionModel3D();
	stopka_5_model	= newCollisionModel3D();
	stopka_6_model	= newCollisionModel3D();

	segment_I_model_1	= newCollisionModel3D();

	segment_I_model_2	= newCollisionModel3D();

	segment_I_model_3	= newCollisionModel3D();

	segment_I_model_4	= newCollisionModel3D();

	segment_I_model_5	= newCollisionModel3D();

	segment_I_model_6	= newCollisionModel3D();


	segment_II_model_1	= newCollisionModel3D();

	segment_II_model_2	= newCollisionModel3D();

	segment_II_model_3	= newCollisionModel3D();

	segment_II_model_4	= newCollisionModel3D();
	segment_II_model_5	= newCollisionModel3D();
	segment_II_model_6	= newCollisionModel3D();

	korpus_dol_model		= newCollisionModel3D();
	korpus_gora_model		= newCollisionModel3D();

	przegub_typu_C_1		= newCollisionModel3D();
	przegub_typu_C_2		= newCollisionModel3D();
	przegub_typu_C_3		= newCollisionModel3D();
	przegub_typu_C_4		= newCollisionModel3D();
	przegub_typu_C_5		= newCollisionModel3D();
	przegub_typu_C_6		= newCollisionModel3D();

	teren_1_model			= newCollisionModel3D();
	teren_2_model			= newCollisionModel3D();

	ground.InitializeTerrain();
}

CObjects3DS::~CObjects3DS(){
}

void CObjects3DS::Object3DS(int obj_qty)
{
    int i,j;
	float normal[3];
	float ** vert = new float*[3];
	for(int i = 0; i < 3; i++)
		vert[i] = new float[3];
    i=obj_qty;
	glColor3f(0.5,0.5,0.5);	
	glBegin(GL_TRIANGLES); // glBegin and glEnd delimit the vertices that define a primitive (in our case triangles)
		for (j=0;j<object[i].polygons_qty;j++)
		{
			//----------------- FIRST VERTEX -----------------
			// Coordinates of the first vertex
			vert[0][0]=object[i].vertex[ object[i].polygon[j].a ].x*0.254;
			vert[0][1]=object[i].vertex[ object[i].polygon[j].a ].y*0.254;
			vert[0][2]=object[i].vertex[ object[i].polygon[j].a ].z*0.254;
			//----------------- SECOND VERTEX -----------------
			// Coordinates of the second vertex
			vert[1][0]=object[i].vertex[ object[i].polygon[j].b ].x*0.254;
			vert[1][1]=object[i].vertex[ object[i].polygon[j].b ].y*0.254;
			vert[1][2]=object[i].vertex[ object[i].polygon[j].b ].z*0.254;
			//----------------- THIRD VERTEX -----------------
			// Coordinates of the Third vertex
			vert[2][0]=object[i].vertex[ object[i].polygon[j].c ].x*0.254;
			vert[2][1]=object[i].vertex[ object[i].polygon[j].c ].y*0.254;
			vert[2][2]=object[i].vertex[ object[i].polygon[j].c ].z*0.254;

			calcNormal(vert, normal);
			glNormal3d (normal[0],normal[1],normal[2]);

			glVertex3f(vert[0][0], vert[0][1], vert[0][2]);

			glVertex3f(vert[1][0], vert[1][1], vert[1][2]);
        
			glVertex3f(vert[2][0], vert[2][1], vert[2][2]);
		}
	glEnd();
	for(int i = 0; i < 3; i++)
		delete [] vert[i];
	delete [] vert;	
}

char CObjects3DS::ObjLoad(char *p_object_name)
{
    if (Load3DS (&object[obj_qty],p_object_name)==0) return(0); //Object loading
    obj_qty++; // Let's increase the object number and get ready to load another object!
	return (1); // If all is ok then return 1
}

void CObjects3DS::CollisionModels(void)
{
	
	int i=0;
	int j;

for (j=0;j<object[i].polygons_qty;j++)
		{
			
			korpus_dol_model->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);
		}
korpus_dol_model->finalize();


i=1;
for (j=0;j<object[i].polygons_qty;j++)
		{
			
			korpus_gora_model->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);
		}
korpus_gora_model->finalize();

i=7;
	for (j=0;j<object[i].polygons_qty;j++)
		{
			
			   stopka_1_model->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);

			   stopka_2_model->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);

			   stopka_3_model->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);

			   stopka_4_model->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);
			
			   stopka_5_model->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);

			   stopka_6_model->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);

		}
	
	
	stopka_1_model->finalize ();
	stopka_2_model->finalize ();
	stopka_3_model->finalize ();
	stopka_4_model->finalize ();
	stopka_5_model->finalize ();
	stopka_6_model->finalize ();

i=5;
	for (j=0;j<object[i].polygons_qty;j++)
		{

			segment_I_model_1->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);
			
			segment_I_model_2->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);
			
			segment_I_model_3->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);
			
			segment_I_model_4->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);
			
			segment_I_model_5->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);
			
			segment_I_model_6->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);
			
	}
segment_I_model_1->finalize();
segment_I_model_2->finalize();
segment_I_model_3->finalize();
segment_I_model_4->finalize();
segment_I_model_5->finalize();
segment_I_model_6->finalize();

i=4;
	for (j=0;j<object[i].polygons_qty;j++)
		{

			segment_II_model_1->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);
			
			
			segment_II_model_2->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);
			
			segment_II_model_3->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);
			
			segment_II_model_4->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);
			
			segment_II_model_5->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);
			
			segment_II_model_6->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);
	}

segment_II_model_1->finalize();
segment_II_model_2->finalize();
segment_II_model_3->finalize();
segment_II_model_4->finalize();
segment_II_model_5->finalize();
segment_II_model_6->finalize();

i=2;

for (j=0;j<object[i].polygons_qty;j++)
		{

			przegub_typu_C_1->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);

			przegub_typu_C_2->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);
			
			przegub_typu_C_3->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);

			przegub_typu_C_4->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);

			przegub_typu_C_5->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);

			przegub_typu_C_6->addTriangle(	object[i].vertex[ object[i].polygon[j].a ].x*0.254, object[i].vertex[ object[i].polygon[j].a ].y*0.254, object[i].vertex[ object[i].polygon[j].a ].z*0.254, 
											object[i].vertex[ object[i].polygon[j].b ].x*0.254, object[i].vertex[ object[i].polygon[j].b ].y*0.254,	object[i].vertex[ object[i].polygon[j].b ].z*0.254,
											object[i].vertex[ object[i].polygon[j].c ].x*0.254, object[i].vertex[ object[i].polygon[j].c ].y*0.254,	object[i].vertex[ object[i].polygon[j].c ].z*0.254);

		}
przegub_typu_C_1->finalize();
przegub_typu_C_2->finalize();
przegub_typu_C_3->finalize();
przegub_typu_C_4->finalize();
przegub_typu_C_5->finalize();
przegub_typu_C_6->finalize();


}

void CObjects3DS::TerrainCollisionModels(void)
{
	delete teren_1_model;
	teren_1_model = newCollisionModel3D();
	delete teren_2_model;
	teren_2_model = newCollisionModel3D();

	for (int z = 1; z < ground.MAP_Z; z++)
	{
		for (int x = 1; x < ground.MAP_X; x++)
		{
			teren_1_model->addTriangle(	ground.terrain[x-1][z-1][0],	ground.terrain[x-1][z-1][1],	ground.terrain[x-1][z-1][2],
										ground.terrain[x][z-1][0],		ground.terrain[x][z-1][1],		ground.terrain[x][z-1][2],
										ground.terrain[x][z][0],		ground.terrain[x][z][1],		ground.terrain[x][z][2]);
		}
		
	}
	teren_1_model->finalize();

	for (int z = 1; z < ground.MAP_Z; z++)
	{
		for (int x = 1; x < ground.MAP_X; x++)
		{
			teren_2_model->addTriangle(	ground.terrain[x-1][z-1][0],	ground.terrain[x-1][z-1][1],	ground.terrain[x-1][z-1][2],
										ground.terrain[x][z][0],		ground.terrain[x][z][1],		ground.terrain[x][z][2],
										ground.terrain[x-1][z][0],		ground.terrain[x-1][z][1],		ground.terrain[x-1][z][2]);
		
					
		}
		
	}
	teren_2_model->finalize();

}