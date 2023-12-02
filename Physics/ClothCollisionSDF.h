#pragma once

#include "Geometry/Collision3D.h"

namespace Physics
{

struct SphereSDF
{
	const Sphere& mSphere;

	SphereSDF(const Sphere& sph) : mSphere(sph) {}

	float QueryPoint(Vector3 point, Vector3& closestPt, Vector3& normal, int& feature) const
	{
		Vector3 delta = point - mSphere.center;
		float len = delta.Length();
		ASSERT(len != 0);
		delta /= len;
		normal = delta;
		closestPt = mSphere.center + mSphere.radius * delta;
		feature = -1;
		return len - mSphere.radius;
	}

	float QuerySegment(Vector3 p, Vector3 q, Vector3& closestPt, Vector3& normal, Vector2& coords, int& feature) const
	{
		float t;
		Vector3 ptSegm = Geometry::ClosestPtSegm(mSphere.center, p, q, t);
		Vector3 delta = ptSegm - mSphere.center;
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

	float QueryTriangle(Vector3 a, Vector3 b, Vector3 c, Vector3& closestPt, Vector3& normal, Vector3& coords, int& feature) const
	{
		Vector3 ptOnTri;
		int region = Geometry::ClosestPtPointTriangle(mSphere.center, a, b, c, ptOnTri, coords);
		Vector3 delta = ptOnTri - mSphere.center;
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
	Vector3 p, q;

	CapsuleSDF(const Capsule& cap) : mCapsule(cap)
	{
		Vector3 halfDir(0, mCapsule.hh, 0);
		halfDir = qRotate(mCapsule.rot, halfDir);
		p = mCapsule.center + halfDir;
		q = mCapsule.center - halfDir;
	}

	float QueryPoint(Vector3 point, Vector3& closestPt, Vector3& normal, int& feature) const
	{
		Vector3 cp = GetClosestPointOnSegment(point);
		Vector3 delta = point - cp;
		float len = delta.Length();
		delta /= len;
		normal = delta;
		closestPt = cp + mCapsule.r * delta;
		feature = -2;
		return len - mCapsule.r;
	}

	float QuerySegment(Vector3 a, Vector3 b, Vector3& closestPt, Vector3& normal, Vector2& coords, int& feature) const
	{
		Vector3 closestPtSegm, closestPtGen;
		float paramSegm, paramGen;
		Geometry::ClosestPtSegmSegm(a, b, p, q, paramSegm, paramGen, closestPtSegm, closestPtGen);

		Vector3 delta = closestPtSegm - closestPtGen;
		float len = delta.Length();
		delta /= len;

		normal = delta;
		closestPt = closestPtGen + mCapsule.r * delta;
		coords.x = 1.0f - paramSegm;
		coords.y = paramSegm;
		feature = -2;

		return len - mCapsule.r;
	}

	float QueryTriangle(Vector3 a, Vector3 b, Vector3 c, Vector3& closestPt, Vector3& normal, Vector3& coords, int& feature) const
	{
		float t;
		Vector3 closestPtEdge, closestPtTri;
		float len = Geometry::ClosestPtSegmTriangle(p, q, a, b, c, t, coords, closestPtEdge, closestPtTri);

		Vector3 delta = closestPtTri - closestPtEdge;
		delta /= len;

		normal = delta;
		closestPt = closestPtEdge + mCapsule.r * delta;
		feature = -2;

		return len - mCapsule.r;
	}

	Vector3 GetClosestPointOnSegment(const Vector3& point) const
	{
		float t;
		return Geometry::ClosestPtSegm(point, p, q, t);
	}
};

struct GridSDF
{
	const Geometry::SDF& mSDF;

	GridSDF(const Geometry::SDF& sdf) : mSDF(sdf) {}

	float QueryPoint(Vector3 point, Vector3& closestPt, Vector3& normal, int& feature) const
	{
		float val = mSDF.GetValue(point);
		Vector3 grad = mSDF.GetGrad(point);
		grad.Normalize();
		normal = grad;
		closestPt = point - val * grad;
		feature = -3;
		return val;
	}

	float QuerySegment(Vector3 a, Vector3 b, Vector3& closestPt, Vector3& normal, Vector2& coords, int&) const
	{
		// TODO: implement using Nvidia paper
		ASSERT(false);
		return 0;
	}

	float QueryTriangle(Vector3 a, Vector3 b, Vector3 c, Vector3& closestPt, Vector3& normal, Vector3& coords, int&) const
	{
		// TODO: implement using Nvidia paper
		ASSERT(false);
		return 0;
	}
};

struct MeshSDF
{
	const Geometry::Mesh mMesh;
	const Geometry::AabbTree* mTree;

	MeshSDF(const Geometry::Mesh& mesh, const Geometry::AabbTree* tree) : mMesh(mesh), mTree(tree) {}

	float QueryPoint(Vector3 point, Vector3& closestPt, Vector3& normal, int& feature) const
	{
		Geometry::ClosestTriangleToPoint info = ClosestPointOnMeshToPointAcc(point, mMesh, mTree, feature);
		float dist = info.distance;
		closestPt = info.closestPtMesh;
		normal = info.normal;
		feature = info.tri;
		Vector3 delta = point - closestPt;
		if (delta.Dot(normal) < 0)
			dist = -dist;
		return dist;
	}

	float QuerySegment(Vector3 a, Vector3 b, Vector3& closestPt, Vector3& normal, Vector2& coords, int& feature) const
	{
		Geometry::ClosestEdgeToSegment info;
		float dist = ClosestPointOnMeshToSegmentAcc(a, b, mMesh, mTree, feature, info);
		closestPt = info.closestPtMesh;
		normal = info.normal;
		coords = info.coordsSegm;
		feature = info.edge;
		Vector3 pt = coords.x * a + coords.y * b;
		Vector3 delta = pt - closestPt;
		if (delta.Dot(normal) < 0)
			dist = -dist;
		return dist;
	}

	float QueryTriangle(Vector3 a, Vector3 b, Vector3 c, Vector3& closestPt, Vector3& normal, Vector3& coords, int& feature) const
	{
		Geometry::ClosestVertexToTriangle info = ClosestPointOnMeshToTriangleAcc(a, b, c, mMesh, mTree, feature);
		closestPt = info.closestPtMesh;
		normal = info.normal;
		coords = info.baryCoords;
		feature = info.vertex;
		Vector3 delta = info.closestPtTri - info.closestPtMesh;
		float dist = info.distance;
		if (delta.Dot(normal) < 0)
			dist = -dist;
		return dist;
	}
};

}