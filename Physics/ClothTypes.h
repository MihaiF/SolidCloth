#ifndef CLOTH_TYPES_H
#define CLOTH_TYPES_H

#include <Engine/Types.h>
#include <Math/Vector3.h>
#include <Math/Vector2.h>

namespace Geometry
{
	class Mesh;
}

namespace Physics
{
	struct Particle
	{
		Math::Vector3 pos, vel;
		float invMass;
		Math::Vector3 force;
		Math::Vector3 prev; // rename to prev
		Math::Vector2 uv;
		int collided;

		Particle() : invMass(1.f) 
		{
			pos.SetZero();
			vel.SetZero();
			force.SetZero();
		}
	};

	struct Quad
	{
		uint32 i1, i2, i3, i4;
	};

	struct Triangle
	{
		// TODO: cleanup!
		uint32 i1, i2, i3;

		// needed by solver
		float area;
		float du1, du2, dv1, dv2, det, invDet, dv12, du12;
		float lu0, lv0, su, sv;
		float lambdaU, lambdaV, lambdaUV;
		Math::Vector2 lambda2;
		float dot;
		float sigma, lu2, lv2;
		float euu, evv, euv;

		bool collided;

		size_t GetOppositeVertex(size_t a, size_t b)
		{
			if (i1 != a && i1 != b)
				return i1;
			if (i2 != a && i2 != b)
				return i2;
			return i3;
		}
	};

	struct PrimitivePair
	{
		int idx1, idx2;
	};

	struct Constraint
	{
		float stiffness, relaxation;
		Math::Vector3 disp; // constraint force
		float lambda; // force magnitude
		float dLambda;
		Math::Vector3 normal; // force direction
		float depth;
		//float omega; // relaxation factor
		Constraint() : stiffness(1.f), lambda(0.f), depth(0)
		{
			disp.SetZero();
		}
	};

	struct MouseSpring : Constraint
	{
		bool active;
		Math::Vector3 point;
		int i1, i2, i3;
		float a1, a2, a3;
		float len, stiffness;
	};

	struct Link : Constraint
	{
		unsigned i1, i2;
		float len; // reference length
		float f; // temp - used by implicit CG
	};

	struct Contact : Constraint
	{
		Math::Vector3 point, vel;
		float lambdaF = 0; // friction
		unsigned idx; // particle index
		int feature = -1;
		const Geometry::Mesh* mesh = nullptr;
	};

	struct EdgeContact : Contact
	{
		uint32 i1, i2;
		float w1, w2;
	};

	struct TriContact : Contact
	{
		uint32 i1, i2, i3;
		float w1, w2, w3;
	};

	struct SelfContact : Constraint
	{
		uint32 i1, i2, i3, i4;
		float w1, w2, w3;
		float lambdaF;
	};

	struct BendConstraint : Constraint
	{
		uint32 i1, i2, i3, i4;
		float theta0; // the initial angle
		float l1, l2;
	};

}

#endif // CLOTH_TYPES_H