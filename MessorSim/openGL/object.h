// Object Class    by Alan Baylis 2001

#ifndef ObjectH
#define ObjectH

#include "../math/vector.h"
#include "../math/CQuaternion.h"

class OBJECT
{
    public:
        OBJECT();
          ~OBJECT();
                
                void Reset();
                void Rotate();
                void Draw();
                void UpdatePosition();
                void UpdatePosition(float x, float y, float z); 
                void MoveX(); 
                void MoveY(); 
                void MoveZ(); 
                void MoveX(float x); 
                void MoveY(float y); 
                void MoveZ(float z); 
                CVector GetXUnit();                
                CVector GetYUnit();                
                CVector GetZUnit();                

				CQuaternion Orientation;
                CVector Position;
                float Delta_x;   //Rotation deltas  
                float Delta_y;
                float Delta_z;
         float Movement_x;    //Movement displacements
        float Movement_y;
        float Movement_z;
                float Multiplier;
};

#endif
