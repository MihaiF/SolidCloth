#include <Engine/Base.h>
#include "Quaternion.h"

// TODO: inline everything

Quaternion& Quaternion::operator=(const Quaternion& q)
{
	s=q.s;
	v=q.v;
	return *this;
}

float& Quaternion::operator[](int i)
{
	switch(i){
		case 1:
			return v[0];			
		case 2:
			return v[1];
		case 3:
			return v[2];
		case 0:
		default:
			return s;
	}
}

float Quaternion::Length() const
{
	return sqrt(s * s +v.X() * v.X() +v.Y() * v.Y() +v.Z() * v.Z());
}

Matrix3 Quaternion::ToMatrix() const
{
	Matrix3 M;
	M.m[0][0] = 1.f - 2.f * (v.Y() * v.Y() + v.Z() * v.Z());
	M.m[0][1] = 2.f * (v.X() * v.Y() - s * v.Z());
	M.m[0][2] = 2.f * (v.X() * v.Z() + s * v.Y());
	M.m[1][0] = 2.f * (v.X() * v.Y() + s * v.Z());
	M.m[1][1] = 1.f - 2.f * (v.X() * v.X() + v.Z() * v.Z());
	M.m[1][2] = 2.f * (v.Y() * v.Z() - s * v.X());
	M.m[2][0] = 2.f * (v.X() * v.Z() - s * v.Y());
	M.m[2][1] = 2.f * (v.Y() * v.Z() + s * v.X());
	M.m[2][2] = 1.f - 2.f * (v.X() * v.X() + v.Y() * v.Y());
	return M;
}

// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
void Quaternion::SetFromMatrix(Matrix3& A)
{
    float trace = A.m[0][0] + A.m[1][1] + A.m[2][2];
    if (trace > 0) 
    {
        float f = 0.5f / sqrtf(trace + 1.0f);
        s = 0.25f / f;
        v.x = (A.m[2][1] - A.m[1][2]) * f;
        v.y = (A.m[0][2] - A.m[2][0]) * f;
        v.z = (A.m[1][0] - A.m[0][1]) * f;
    }
    else {
        if (A.m[0][0] > A.m[1][1] && A.m[0][0] > A.m[2][2]) {
            float f = 2.0f * sqrtf(1.0f + A.m[0][0] - A.m[1][1] - A.m[2][2]);
            s = (A.m[2][1] - A.m[1][2]) / f;
            v.x = 0.25f * f;
            v.y = (A.m[0][1] + A.m[1][0]) / f;
            v.z = (A.m[0][2] + A.m[2][0]) / f;
        }
        else if (A.m[1][1] > A.m[2][2]) {
            float f = 2.0f * sqrtf(1.0f + A.m[1][1] - A.m[0][0] - A.m[2][2]);
            s = (A.m[0][2] - A.m[2][0]) / f;
            v.x = (A.m[0][1] + A.m[1][0]) / f;
            v.y = 0.25f * f;
            v.z = (A.m[1][2] + A.m[2][1]) / f;
        }
        else {
            float f = 2.0f * sqrtf(1.0f + A.m[2][2] - A.m[0][0] - A.m[1][1]);
            s = (A.m[1][0] - A.m[0][1]) / f;
            v.x = (A.m[0][2] + A.m[2][0]) / f;
            v.y = (A.m[1][2] + A.m[2][1]) / f;
            v.z = 0.25f * f;
        }
    }
}

void Quaternion::operator +=(Quaternion q)
{
	s += q.s;
	v += q.v;
}

Quaternion qScale(Quaternion q, float s){
	Quaternion qs;
	for(int i=0;i<4;i++)
		qs[i]=q[i]*s;
	return qs;
}

Quaternion operator*(Quaternion q, float s){
	return qScale(q,s);
}

Quaternion operator*(float s, Quaternion q){
	return qScale(q,s);
}

Quaternion qNormalize(Quaternion q){
	Quaternion qn;
	float l=q.Length();
	if(l) qn=qScale(q,1/l);
	return qn;
}

Quaternion qMultiply(Quaternion q1, Quaternion q2)
{
	Quaternion r;
	r.s = q1.s * q2.s - q1.v.Dot(q2.v);
	r.v = q1.s * q2.v + q2.s * q1.v + q1.v.Cross(q2.v);
	return r;
}

Quaternion operator*(Quaternion q1, Quaternion q2){
	return qMultiply(q1,q2);
}

Quaternion operator*(Vector3 v, Quaternion q){
	Quaternion p(0, v);
	return p*q;
}

Quaternion operator*(Quaternion q, Vector3 v){
	Quaternion p(0, v);
	return p*q;
}

Matrix3 qToMatrix(Quaternion q){
	return q.ToMatrix();
}

Vector3 qRotate(Quaternion q, Vector3 v)
{
	Quaternion qi = q.GetConjugate();
	Quaternion r = q * (v * qi);
	return r.v;
}

Quaternion operator+(Quaternion q1, Quaternion q2)
{
	Quaternion r;
	r.s = q1.s + q2.s;
	r.v = q1.v + q2.v;
	return r;
}

float qDot(Quaternion q1, Quaternion q2)
{
	return q1.s * q2.s + q1.v * q2.v;
}