#pragma once

#include "Geometry/Collision3D.h"

namespace Physics
{

struct SphereSDF
{
	const Sphere& mSphere;

	SphereSDF(const Sphere& sph) : mSphere(sph) {}

	float QueryPoint(Vector3 point, Vector3& closestPt, Vector3& normal) const
	{
		Vector3 delta = point - mSphere.center;
		float len = delta.Length();
		ASSERT(len != 0);
		delta /= len;
		normal = delta;
		closestPt = mSphere.center + mSphere.radius * delta;
		return len - mSphere.radius;
	}

	float QuerySegment(Vector3 p, Vector3 q, Vector3& closestPt, Vector3& normal) const
	{
		float t;
		Vector3 ptSegm = Geometry::ClosestPtSegm(mSphere.center, p, q, t);
		Vector3 delta = ptSegm - mSphere.center;
		float len = delta.Length();
		ASSERT(len != 0);
		delta /= len;
		normal = delta;
		closestPt = mSphere.center + mSphere.radius * delta;
		return len - mSphere.radius;
	}

	float QueryTriangle(Vector3 a, Vector3 b, Vector3 c, Vector3& closestPt, Vector3& normal, Vector3& coords) const
	{
		Vector3 ptOnTri = Geometry::ClosestPtPointTriangle(mSphere.center, a, b, c, coords);
		Vector3 delta = ptOnTri - mSphere.center;
		float len = delta.Length();
		ASSERT(len != 0);
		delta /= len;
		normal = delta;
		closestPt = mSphere.center + mSphere.radius * delta;
		return len - mSphere.radius;
	}
};

struct CapsuleSDF
{
	const Capsule& mCapsule;

	CapsuleSDF(const Capsule& cap) : mCapsule(cap) {}

	float QueryPoint(Vector3 point, Vector3& closestPt, Vector3& normal) const
	{
		Vector3 cp = GetClosestPointOnSegment(point);
		Vector3 delta = point - cp;
		float len = delta.Length();
		delta /= len;
		normal = delta;
		closestPt = cp + mCapsule.r * delta;
		return len - mCapsule.r;
	}

	float QueryTriangle(Vector3 a, Vector3 b, Vector3 c, Vector3& closestPt, Vector3& normal, Vector3& coords) const
	{
		Vector3 halfDir(0, mCapsule.hh, 0);
		halfDir = qRotate(mCapsule.rot, halfDir);
		Vector3 p = mCapsule.center + halfDir;
		Vector3 q = mCapsule.center - halfDir;

		float t;
		Vector3 closestPtEdge, closestPtTri;
		Geometry::ClosestPtSegmTriangle(p, q, a, b, c, t, coords, closestPtEdge, closestPtTri);

		Vector3 delta = closestPtTri - closestPtEdge;
		float len = delta.Length();
		delta /= len;

		normal = delta;
		closestPt = closestPtEdge + mCapsule.r * delta;

		return len - mCapsule.r;
	}

	Vector3 GetClosestPointOnSegment(const Vector3& point) const
	{
		Vector3 halfDir(0, mCapsule.hh, 0);
		halfDir = qRotate(mCapsule.rot, halfDir);
		Vector3 p = mCapsule.center + halfDir;
		Vector3 q = mCapsule.center - halfDir;
		float t;
		return Geometry::ClosestPtSegm(point, p, q, t);
	}
};

struct GridSDF
{
	const Geometry::SDF& mSDF;

	GridSDF(const Geometry::SDF& sdf) : mSDF(sdf) {}

	float QueryPoint(Vector3 point, Vector3& closestPt, Vector3& normal) const
	{
		float val = mSDF.GetValue(point);
		Vector3 grad = mSDF.GetGrad(point);
		grad.Normalize();
		normal = grad;
		closestPt = point - val * grad;
		return val;
	}

	float QueryTriangle(Vector3 a, Vector3 b, Vector3 c, Vector3& closestPt, Vector3& normal, Vector3& coords) const
	{
		// TODO: implement usin Nvidia paper
		ASSERT(false);
		return 0;
	}
};

struct MeshSDF
{
	const Geometry::Mesh mMesh;
	const Geometry::AabbTree* mTree;

	MeshSDF(const Geometry::Mesh& mesh, const Geometry::AabbTree* tree) : mMesh(mesh), mTree(tree) {}

	float QueryPoint(Vector3 point, Vector3& closestPt, Vector3& normal) const
	{
		float dist = ClosestPointOnMeshToPointAcc(point, mMesh, mTree, closestPt, normal);
		Vector3 delta = point - closestPt;
		if (delta.Dot(normal) < 0)
			dist = -dist;
		return dist;
	}

	float QueryTriangle(Vector3 a, Vector3 b, Vector3 c, Vector3& closestPt, Vector3& normal, Vector3& coords) const
	{
		Geometry::ClosestMeshTriangle info;
		float dist = ClosestPointOnMeshToTriangle(a, b, c, mMesh, info);
		closestPt = info.closestPtMesh;
		normal = info.normal;
		coords = info.baryCoords;
		Vector3 delta = info.closestPtTri - info.closestPtMesh;
		if (delta.Dot(normal) < 0)
			dist = -dist;
		return dist;
	}

	Vector3 GetClosestPoint(const Vector3& point) const
	{
		Vector3 closestPt, normal;
		float dist = ClosestPointOnMeshToPointAcc(point, mMesh, mTree, closestPt, normal);
		return closestPt;
	}
};

}