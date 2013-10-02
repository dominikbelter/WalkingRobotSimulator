#include <windows.h>
#include "light.h"


LIGHT::LIGHT()
{
}

LIGHT::~LIGHT()
{
}

void LIGHT::Reset()
{
    Orientation.Reset();
    Position.Set(2.0, 2.0, 3.0);
    Delta_x = 0.0;
    Delta_y = 0.0;
    Delta_z = 0.0;
    Movement_x = 0.0;
    Movement_y = 0.0;
    Movement_z = 0.0;
    Ambient[0] = 0.0;
    Ambient[1] = 0.0;
    Ambient[2] = 0.0;
    Ambient[3] = 1.0;
    Diffuse[0] = 1.0;
    Diffuse[1] = 1.0;
    Diffuse[2] = 1.0;
    Diffuse[3] = 1.0;
    Specular[0] = 1.0;
    Specular[1] = 1.0;
    Specular[2] = 1.0;
    Specular[3] = 1.0;
    SpotLight = GL_FALSE;
    SpotDirection = GetZUnit();
    SpotExponent = 0.0;
    SpotCutoff = 180.0;
    Constant = 1.0;
    Linear = 0.0;
    Quadratic = 0.0;
    GlobalAmbient[0] = 0.2;
    GlobalAmbient[1] = 0.2;
    GlobalAmbient[2] = 0.2;
    GlobalAmbient[3] = 1.0;
    Positional = GL_FALSE;
    LocalViewer = GL_FALSE;
    TwoSided = GL_FALSE;
    float LightPosition[] = {Position.x, Position.y, Position.z, (float)Positional};
    glLightfv(GL_LIGHT[LightNumber], GL_POSITION, LightPosition);
    glLightfv(GL_LIGHT[LightNumber], GL_AMBIENT, Ambient);
    glLightfv(GL_LIGHT[LightNumber], GL_DIFFUSE, Diffuse);
    glLightfv(GL_LIGHT[LightNumber], GL_SPECULAR, Specular);
    glEnable(GL_LIGHT[LightNumber]);
}

void LIGHT::Update()
{
    glLightfv(GL_LIGHT[LightNumber], GL_AMBIENT, Ambient);
    glLightfv(GL_LIGHT[LightNumber], GL_DIFFUSE, Diffuse);
    glLightfv(GL_LIGHT[LightNumber], GL_SPECULAR, Specular);
}

void LIGHT::Apply()
{
    UpdatePosition();
    float LightPosition[] = {Position.x, Position.y, Position.z, (float)Positional};
    glLightfv(GL_LIGHT[LightNumber], GL_POSITION, LightPosition);
}
