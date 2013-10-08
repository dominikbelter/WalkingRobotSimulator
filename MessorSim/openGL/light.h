// Light Class    by Alan Baylis 2001

#ifndef LightH
#define LightH

//#include "../shared.h"
#include <stdio.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include "../openGL/object.h"

extern int GL_LIGHT[8];

class LIGHT : public OBJECT
{
public:
    LIGHT();
      ~LIGHT();

        void Reset();
    void Update();
    void Apply();
 
        int LightNumber;
        float Ambient[4];
        float Diffuse[4];
        float Specular[4];
        bool SpotLight;
        CVector SpotDirection;
        float SpotExponent;
        float SpotCutoff;
        float Constant;
        float Linear;
        float Quadratic;
        float GlobalAmbient[4];
        bool Positional;
        bool LocalViewer;
        bool TwoSided; 
};

#endif