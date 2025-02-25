#pragma once

#include "Geometry/Collision3D.h"

namespace Physics
{

struct SphereSDF
{
	const Sphere& mSphere;

	SphereSDF(const Sphere& sph) : mSphere(sph) {}

	float QueryPoint(Math::Vector3 point, Math::Vector3& closestPt, Math::Vector3& normal, int& feature) const
	{
		Math::Vector3 delta = point - mSphere.center;
		float len = delta.Length();
		ASSERT(len != 0);
		delta /= len;
		normal = delta;
		closestPt = mSphere.center + mSphere.radius * delta;
		feature = -1;
		return len - mSphere.radius;
	}

	float QuerySegment(Math::Vector3 p, Math::Vector3 q, Math::Vector3& closestPt, Math::Vector3& normal, Math::Vector2& coords, int& feature) const
	{
		float t;
		Math::Vector3 ptSegm = Geometry::ClosestPtSegm(mSphere.center, p, q, t);
		Math::Vector3 delta = ptSegm - mSphere.center;
		float len = delta.Length();
		ASSERT(len != 0);
		delta /= len;
		normal = delta;
		closestPt = mSphere.center + mSphere.radius * delta;
		coords.x = 1.0f - t;
		coords.y = t;
		feature = -1;
		return len - mSphere.radius;
	}

	float QueryTriangle(Math::Vector3 a, Math::Vector3 b, Math::Vector3 c, Math::Vector3& closestPt, Math::Vector3& normal, Math::Vector3& coords, int& feature) const
	{
		Math::Vector3 ptOnTri;
		int region = Geometry::ClosestPtPointTriangle(mSphere.center, a, b, c, ptOnTri, coords);
		Math::Vector3 delta = ptOnTri - mSphere.center;
		float len = delta.Length();
		ASSERT(len != 0);
		delta /= len;
		normal = delta;
		closestPt = mSphere.center + mSphere.radius * delta;
		feature = -1;
		return len - mSphere.radius;
	}
};

struct CapsuleSDF
{
	const Capsule& mCapsule;
	Math::Vector3 p, q;

	CapsuleSDF(const Capsule& cap) : mCapsule(cap)
	{
		Math::Vector3 halfDir(0, mCapsule.hh, 0);
		halfDir = qRotate(mCapsule.rot, halfDir);
		p = mCapsule.center + halfDir;
		q = mCapsule.center - halfDir;
	}

	float QueryPoint(Math::Vector3 point, Math::Vector3& closestPt, Math::Vector3& normal, int& feature) const
	{
		Math::Vector3 cp = GetClosestPointOnSegment(point);
		Math::Vector3 delta = point - cp;
		float len = delta.Length();
		delta /= len;
		normal = delta;
		closestPt = cp + mCapsule.r * delta;
		feature = -2;
		return len - mCapsule.r;
	}

	float QuerySegment(Math::Vector3 a, Math::Vector3 b, Math::Vector3& closestPt, Math::Vector3& normal, Math::Vector2& coords, int& feature) const
	{
		Math::Vector3 closestPtSegm, closestPtGen;
		float paramSegm, paramGen;
		Geometry::ClosestPtSegmSegm(a, b, p, q, paramSegm, paramGen, closestPtSegm, closestPtGen);

		Math::Vector3 delta = closestPtSegm - closestPtGen;
		float len = delta.Length();
		delta /= len;

		normal = delta;
		closestPt = closestPtGen + mCapsule.r * delta;
		coords.x = 1.0f - paramSegm;
		coords.y = paramSegm;
		feature = -2;

		return len - mCapsule.r;
	}

	float QueryTriangle(Math::Vector3 a, Math::Vector3 b, Math::Vector3 c, Math::Vector3& closestPt, Math::Vector3& normal, Math::Vector3& coords, int& feature) const
	{
		float t;
		Math::Vector3 closestPtEdge, closestPtTri;
		float len = Geometry::ClosestPtSegmTriangle(p, q, a, b, c, t, coords, closestPtEdge, closestPtTri);

		Math::Vector3 delta = closestPtTri - closestPtEdge;
		delta /= len;

		normal = delta;
		closestPt = closestPtEdge + mCapsule.r * delta;
		feature = -2;

		return len - mCapsule.r;
	}

	Math::Vector3 GetClosestPointOnSegment(const Math::Vector3& point) const
	{
		float t;
		return Geometry::ClosestPtSegm(point, p, q, t);
	}
};

struct GridSDF
{
	const Geometry::SDF& mSDF;

	GridSDF(const Geometry::SDF& sdf) : mSDF(sdf) {}

	float QueryPoint(Math::Vector3 point, Math::Vector3& closestPt, Math::Vector3& normal, int& feature) const
	{
		float val = mSDF.GetValue(point);
		Math::Vector3 grad = mSDF.GetGrad(point);
		grad.Normalize();
		normal = grad;
		closestPt = point - val * grad;
		feature = -3;
		return val;
	}

	float QuerySegment(Math::Vector3 a, Math::Vector3 b, Math::Vector3& closestPt, Math::Vector3& normal, Math::Vector2& coords, int&) const
	{
		// TODO: implement using Nvidia paper
		//ASSERT(false);
		return 0;
	}

	float QueryTriangle(Math::Vector3 a, Math::Vector3 b, Math::Vector3 c, Math::Vector3& closestPt, Math::Vector3& normal, Math::Vector3& coords, int&) const
	{
		// find the initial guess for s_i (smallest distance)
		float distA = mSDF.GetValue(a);
		float distB = mSDF.GetValue(b);
		float distC = mSDF.GetValue(c);
		if (distA == FLT_MAX || distB == FLT_MAX || distC == FLT_MAX)
			return FLT_MAX;

		Math::Vector3 cp = a; // closest point on triangle
		float dist = distA;
		if (distB < dist)
		{
			cp = b;
			dist = distB;
		}
		if (distC < dist)
		{
			cp = c;
			//dist = distC;
		}

		// run iterative Frank-Wolfe algorithm
		for (int i = 1; i < 10; i++)
		{
			// 1. find s by direct enumeration of a, b, c
			dist = mSDF.GetValue(cp);
			Math::Vector3 grad = mSDF.GetGrad(cp);
			float dotA = a.Dot(grad);
			float dotB = b.Dot(grad);
			float dotC = c.Dot(grad);
			float dot = dotA;
			Math::Vector3 s = a;
			if (dotB < dot)
			{
				dot = dotB;
				s = b;
			}
			if (dotC < dot)
			{
				dot = dotC;
				s = c;
			}
			Printf("%d: dist: %.3f, dot: %.3f\n", i, dist, dot);

			// 2. update closest point
			float alpha = 2.f / (i + 2.9f);
			Math::Vector3 delta = s - cp;
			cp += alpha * delta;
		}

		// return all the information
		float val = mSDF.GetValue(cp);
		Math::Vector3 grad = mSDF.GetGrad(cp);
		grad.Normalize();
		normal = grad;
		closestPt = cp - val * grad;
		coords = Geometry::Barycentric(a, b, c, cp);

		return val;
	}
};

struct MeshSDF
{
	const Geometry::Mesh mMesh;
	const Geometry::AabbTree* mTree;

	MeshSDF(const Geometry::Mesh& mesh, const Geometry::AabbTree* tree) : mMesh(mesh), mTree(tree) {}

	float QueryPoint(Math::Vector3 point, Math::Vector3& closestPt, Math::Vector3& normal, int& feature) const
	{
		Geometry::ClosestTriangleToPoint info = ClosestPointOnMeshToPointAcc(point, mMesh, mTree, feature);
		float dist = info.distance;
		closestPt = info.closestPtMesh;
		normal = info.normal;
		feature = info.tri;
		Math::Vector3 delta = point - closestPt;
		if (delta.Dot(normal) < 0)
			dist = -dist;
		return dist;
	}

	float QuerySegment(Math::Vector3 a, Math::Vector3 b, Math::Vector3& closestPt, Math::Vector3& normal, Math::Vector2& coords, int& feature) const
	{
		Geometry::ClosestEdgeToSegment info;
		float dist = ClosestPointOnMeshToSegmentAcc(a, b, mMesh, mTree, false, feature, info);
		closestPt = info.closestPtMesh;
		normal = info.normal;
		coords = info.coordsSegm;
		feature = info.edge;
		Math::Vector3 pt = coords.x * a + coords.y * b;
		Math::Vector3 delta = pt - closestPt;
		if (delta.Dot(normal) < 0)
			dist = -dist;
		return dist;
	}

	float QueryTriangle(Math::Vector3 a, Math::Vector3 b, Math::Vector3 c, Math::Vector3& closestPt, Math::Vector3& normal, Math::Vector3& coords, int& feature) const
	{
		Geometry::ClosestVertexToTriangle info = ClosestPointOnMeshToTriangleAcc(a, b, c, mMesh, mTree, feature);
		closestPt = info.closestPtMesh;
		normal = info.normal;
		coords = info.baryCoords;
		feature = info.vertex;
		Math::Vector3 delta = info.closestPtTri - info.closestPtMesh;
		float dist = info.distance;
		if (delta.Dot(normal) < 0)
			dist = -dist;
		return dist;
	}
};

}