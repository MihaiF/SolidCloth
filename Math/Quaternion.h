#ifndef QUATERNION_H
#define QUATERNION_H

#include "Matrix3.h"
#include "Vector4.h"

namespace Math
{

	class Quaternion
	{
	public:
		float s; // a.k.a. w
		Vector3 v;

	public:
		Quaternion() : s(1) { }
		Quaternion(float qs, Vector3 qv) : s(qs), v(qv) { }
		Quaternion(Vector4 a) : s(a[3]), v(a[0], a[1], a[2]) { }
		Quaternion(const Quaternion& q) : s(q.s), v(q.v) { } // do I really need this copy constructor?
		Quaternion& operator =(const Quaternion& q); // or this operator?
		float& operator[](int i);
		float Length() const;
		Matrix3 ToMatrix() const;
		void operator +=(Quaternion q);
		Quaternion GetConjugate() const { return Quaternion(s, -v); }

		static Quaternion Rotation(float angle, const Vector3& axis);
		void GetAxisAngle(float& angle, Vector3& axis) const;
		void SetAxisAngle(float angle, const Vector3& axis);
		void SetFromMatrix(Matrix3& A);

		void Scale(float scale)
		{
			s *= scale;
			v.Scale(scale);
		}

		void Normalize()
		{
			float l = Length();
			if (l != 0)
				Scale(1.f / l);
		}

		void SetFromTo(Vector3 vFrom, Vector3 vTo)
		{
			// [TODO] this page seems to have optimized version:
			//    http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors

			// [RMS] not ideal to explicitly normalize here, but if we don't,
			//   output quaternion is not normalized and this causes problems,
			//   eg like drift if we do repeated SetFromTo()
			Vector3 from = vFrom;
			from.Normalize();
			Vector3 to = vTo;
			to.Normalize();
			Vector3 bisector = from + to;
			bisector.Normalize();
			s = from.Dot(bisector);
			if (s != 0)
			{
				Vector3 cross = from.Cross(bisector);
				v = cross;
			}
			else
			{
				float invLength;
				if (fabs(from.x) >= fabs(from.y))
				{
					// V1.x or V1.z is the largest magnitude component.
					invLength = (float)(1.0 / sqrt(from.x * from.x + from.z * from.z));
					v.x = -from.z * invLength;
					v.y = 0;
					v.z = +from.x * invLength;
				}
				else
				{
					// V1.y or V1.z is the largest magnitude component.
					invLength = (float)(1.0 / sqrt(from.y * from.y + from.z * from.z));
					v.x = 0;
					v.y = +from.z * invLength;
					v.z = -from.y * invLength;
				}
			}
			Normalize();   // aaahhh just to be safe...
		}

	};

	Quaternion qNormalize(Quaternion q);
	Quaternion qScale(Quaternion q, float s);
	Quaternion operator*(Quaternion q, float s);
	Quaternion operator*(float s, Quaternion q);
	Quaternion operator+(Quaternion q1, Quaternion q2);
	Quaternion qMultiply(Quaternion q1, Quaternion q2);
	Quaternion operator*(Quaternion q1, Quaternion q2);
	Quaternion operator*(Vector3 v, Quaternion q);
	Quaternion operator*(Quaternion q, Vector3 v);
	Matrix3 qToMatrix(Quaternion q);
	Vector3 qRotate(Quaternion q, Vector3 v);
	float qDot(Quaternion q1, Quaternion q2);

	inline Quaternion Quaternion::Rotation(float angle, const Vector3& axis)
	{
		Quaternion q;
		float u = 0.5f * angle;
		q.s = cosf(u);
		q.v = sinf(u) * axis;
		// TODO: normalize axis or quaternion
		return q;
	}

	inline void Quaternion::GetAxisAngle(float& angle, Vector3& axis) const
	{
		//float len = v.Length();
		//axis = v;
		//axis.Normalize();
		//angle = atan2f(len, s);

		//if (q1.w > 1) q1.normalise(); // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is normalised
		angle = 2 * acosf(s);
		float len = sqrtf(1 - s * s); // assuming quaternion normalized then w is less than 1, so term always positive.
		axis = v;
		if (len > 0.001f) { // test to avoid divide by zero, s is always positive due to sqrt
			axis.Scale(1.f / len);
		}
	}

	inline void Quaternion::SetAxisAngle(float angle, const Vector3& axis)
	{
		float len = axis.Length();
		if (len == 0)
			return;
		s = cosf(0.5f * angle);
		v = (sinf(0.5f * angle) / len) * axis;
	}

} // namespace Math

#endif