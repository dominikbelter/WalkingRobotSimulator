#include <cmath>
#include "vector.h"

VECTOR::VECTOR(float sx, float sy, float sz)
:
    x(sx),
    y(sy),
    z(sz)
{
}

VECTOR::~VECTOR()
{
}

void VECTOR::Reset()
{
    x = 0;
    y = 0;
    z = 0;
}

float VECTOR::DotProduct(VECTOR vect)
{
      float dot;
      dot = vect.x * x + vect.y * y + vect.z * z;
      return dot;
}

void VECTOR::CrossVector(VECTOR vect)
{
      VECTOR temp = *this;
      x = vect.y * temp.z - vect.z * temp.y;
      y = vect.z * temp.x - vect.x * temp.z;
      z = vect.x * temp.y - vect.y * temp.x;
}

float VECTOR::GetMagnitude()
{
    float magnitude = (float)sqrt(x * x + y * y + z * z);
    if (magnitude != 0.0f)
        return magnitude;
    else
        return 1.0;
}

void VECTOR::Normalize()
{
    float magnitude = this->GetMagnitude();
    x /= magnitude;
    y /= magnitude;
    z /= magnitude;
}
