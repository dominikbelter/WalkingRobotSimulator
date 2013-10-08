#include <windows.h>
#include "object.h"

extern bool freeCamera;
float pi = 3.141592654;
float radian = pi/180;

OBJECT::OBJECT()
{
    Delta_x = 0.0;
    Delta_y = 0.0;
    Delta_z = 0.0;
    Movement_x = 0.0;
    Movement_y = 0.0;
    Movement_z = 0.0;
}

OBJECT::~OBJECT()
{
}

void OBJECT::Reset()
{
    Orientation.Reset();
    Position.reset();
        Delta_x = 0.0;
        Delta_y = 0.0;
        Delta_z = 0.0;
        Movement_x = 0.0;
        Movement_y = 0.0;
        Movement_z = 0.0;
}

void OBJECT::Rotate()
{
    CQuaternion temp_quat;
    temp_quat.EulerToQuat(Delta_x * radian * Multiplier, Delta_y * radian * Multiplier, Delta_z * radian * Multiplier);
    Orientation.multQuat(temp_quat);
}

void OBJECT::Draw()
{
  // Should probably be a pure virtual 
}

void OBJECT::UpdatePosition()
{
    if (Movement_x != 0)
        MoveX();
    if (Movement_y != 0)
        MoveY();
    if (Movement_z != 0)
        MoveZ();

    Movement_x = 0.0;
    Movement_y = 0.0;
    Movement_z = 0.0;
}

void OBJECT::UpdatePosition(float x, float y, float z)
{
    if (x != 0)
        MoveX(x);
    if (y != 0)
        MoveY(y);
    if (z != 0)
        MoveZ(z);
}

void OBJECT::MoveX()
{
    float DirX;
    float DirY;
    float DirZ;
    float W;
    float X;
    float Y;
    float Z;
    CQuaternion TempQuat;
    CQuaternion TempQuat2;
    TempQuat = Orientation;
    TempQuat2.EulerToQuat(0.0, -90.0*(pi/180), 0.0);
    TempQuat.multQuat(TempQuat2);
    W = TempQuat.w;
    X = TempQuat.x;
    Y = TempQuat.y;
    Z = TempQuat.z;
    DirX = 2.0 * ( X * Z - W * Y );
    DirY = 2.0 * ( Y * Z + W * X );
    DirZ = 1.0 - 2.0 * ( X * X + Y * Y );
    Position.x += DirX * Movement_x * Multiplier;
    Position.y += DirY * Movement_x * Multiplier;
    Position.z += DirZ * Movement_x * Multiplier;
}

void OBJECT::MoveY()
{
    float DirX;
    float DirY;
    float DirZ;
    float W;
    float X;
    float Y;
    float Z;
	CQuaternion TempQuat;
	CQuaternion TempQuat2;
    TempQuat = Orientation;
    TempQuat2.EulerToQuat(90.0*(pi/180), 0.0, 0.0);
    TempQuat.multQuat(TempQuat2);
    W = TempQuat.w;
    X = TempQuat.x;
    Y = TempQuat.y;
    Z = TempQuat.z;
    DirX = 2.0 * ( X * Z - W * Y );
    DirY = 2.0 * ( Y * Z + W * X );
    DirZ = 1.0 - 2.0 * ( X * X + Y * Y );
    Position.x += DirX * Movement_y * Multiplier;
    Position.y += DirY * Movement_y * Multiplier;
    Position.z += DirZ * Movement_y * Multiplier;
}

void OBJECT::MoveZ()
{
    float DirX;
    float DirY;
    float DirZ;
    float W = Orientation.w;
    float X = Orientation.x;
    float Y = Orientation.y;
    float Z = Orientation.z;
    DirX = 2.0 * ( X * Z - W * Y );
    DirY = 2.0 * ( Y * Z + W * X );
    DirZ = 1.0 - 2.0 * ( X * X + Y * Y );
    Position.x += DirX * Movement_z * Multiplier;
    Position.y += DirY * Movement_z * Multiplier;
    Position.z += DirZ * Movement_z * Multiplier;
}

void OBJECT::MoveX(float x)
{
    float DirX;
    float DirY;
    float DirZ;
    float W;
    float X;
    float Y;
    float Z;
    CQuaternion TempQuat;
	CQuaternion TempQuat2;
    TempQuat = Orientation;
    TempQuat2.EulerToQuat(0.0, -90.0*(pi/180), 0.0);
    TempQuat.multQuat(TempQuat2);
    W = TempQuat.w;
    X = TempQuat.x;
    Y = TempQuat.y;
    Z = TempQuat.z;
    DirX = 2.0 * ( X * Z - W * Y );
    DirY = 2.0 * ( Y * Z + W * X );
    DirZ = 1.0 - 2.0 * ( X * X + Y * Y );
    Position.x += DirX * x;
    Position.y += DirY * x;
    Position.z += DirZ * x;
}

void OBJECT::MoveY(float y)
{
    float DirX;
    float DirY;
    float DirZ;
    float W;
    float X;
    float Y;
    float Z;
	CQuaternion TempQuat;
	CQuaternion TempQuat2;
    TempQuat = Orientation;
    TempQuat2.EulerToQuat(90.0*(pi/180), 0.0, 0.0);
    TempQuat.multQuat(TempQuat2);
    W = TempQuat.w;
    X = TempQuat.x;
    Y = TempQuat.y;
    Z = TempQuat.z;
    DirX = 2.0 * ( X * Z - W * Y );
    DirY = 2.0 * ( Y * Z + W * X );
    DirZ = 1.0 - 2.0 * ( X * X + Y * Y );
    Position.x += DirX * y;
    Position.y += DirY * y;
    Position.z += DirZ * y;
}

void OBJECT::MoveZ(float z)
{
    float DirX;
    float DirY;
    float DirZ;
    float W = Orientation.w;
    float X = Orientation.x;
    float Y = Orientation.y;
    float Z = Orientation.z;
    DirX = 2.0 * ( X * Z - W * Y );
    DirY = 2.0 * ( Y * Z + W * X );
    DirZ = 1.0 - 2.0 * ( X * X + Y * Y );
    Position.x += DirX * z;
    Position.y += DirY * z;
    Position.z += DirZ * z;
}

CVector OBJECT::GetXUnit()
{
    float DirX;
    float DirY;
    float DirZ;
    float W;
    float X;
    float Y;
    float Z;
    CQuaternion TempQuat;
    CQuaternion TempQuat2;
    TempQuat = Orientation;
    TempQuat2.EulerToQuat(0.0, -90.0*(pi/180), 0.0);
    TempQuat.multQuat(TempQuat2);
    W = TempQuat.w;
    X = TempQuat.x;
    Y = TempQuat.y;
    Z = TempQuat.z;
    DirX = 2.0 * ( X * Z - W * Y );
    DirY = 2.0 * ( Y * Z + W * X );
    DirZ = 1.0 - 2.0 * ( X * X + Y * Y );
    CVector Unit;
    Unit.x += DirX * 1;
    Unit.y += DirY * 1;
    Unit.z += DirZ * 1;
    return Unit;
}

CVector OBJECT::GetYUnit()
{
    float DirX;
    float DirY;
    float DirZ;
    float W;
    float X;
    float Y;
    float Z;
    CQuaternion TempQuat;
    CQuaternion TempQuat2;
    TempQuat = Orientation;
    TempQuat2.EulerToQuat(90.0*(pi/180), 0.0, 0.0);
    TempQuat.multQuat(TempQuat2);
    W = TempQuat.w;
    X = TempQuat.x;
    Y = TempQuat.y;
    Z = TempQuat.z;
    DirX = 2.0 * ( X * Z - W * Y );
    DirY = 2.0 * ( Y * Z + W * X );
    DirZ = 1.0 - 2.0 * ( X * X + Y * Y );
    CVector Unit;
    Unit.x += DirX * 1;
    Unit.y += DirY * 1;
    Unit.z += DirZ * 1;
    return Unit;
}

CVector OBJECT::GetZUnit()
{
    float DirX;
    float DirY;
    float DirZ;
    float W = Orientation.w;
    float X = Orientation.x;
    float Y = Orientation.y;
    float Z = Orientation.z;
    DirX = 2.0 * ( X * Z - W * Y );
    DirY = 2.0 * ( Y * Z + W * X );
    DirZ = 1.0 - 2.0 * ( X * X + Y * Y );
    CVector Unit;
    Unit.x += DirX * 1;
    Unit.y += DirY * 1;
    Unit.z += DirZ * 1;
    return Unit;
}

