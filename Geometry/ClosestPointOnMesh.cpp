#include "ClosestPointOnMesh.h"

#include <Math/Vector3.h>
#include <Geometry/Mesh.h>
#include <Geometry/AabbTree.h>
#include <Geometry/VoronoiRegions.h>
#include <Engine/Profiler.h>
#include <Engine/Utils.h>

#include <omp.h>

using namespace Math;

namespace Geometry
{

int ClosestVertexOnMeshToPoint(const Vector3& p, const Mesh& mesh)
{
	PROFILE_SCOPE("Closest pt-mesh vertex");

	// brute force closest point
	float distMin = FLT_MAX;
	int closestPt = -1;
	for (int i = 0; i < mesh.vertices.size(); i++)
	{
		Vector3 pt = mesh.vertices[i];
		float dist = (pt - p).Length();
		if (dist < distMin)
		{
			distMin = dist;
			closestPt = i;
		}
	}
	return closestPt;
}

Vector3 ComputeNormal(const Mesh& mesh, int tri, Vector3 coords, bool usePseudoNormal)
{
	Vector3 normal;

	int i0 = mesh.indices[tri * 3 + 0];
	int i1 = mesh.indices[tri * 3 + 1];
	int i2 = mesh.indices[tri * 3 + 2];

	if (mesh.edges.size() == 0)
		usePseudoNormal = false;

	if (!usePseudoNormal)
	{
		normal = coords.x * mesh.normals[i0] + coords.y * mesh.normals[i1] + coords.z * mesh.normals[i2];
		normal.Normalize();
	}
	else
	{
		const float eps = 1e-12f;
		bool test1 = coords.x > eps;
		bool test2 = coords.y > eps;
		bool test3 = coords.z > eps;
		int sum = test1 + test2 + test3;
		ASSERT(sum != 0);
		if (sum == 3)
		{
			normal = mesh.triangles[tri].n;
			//normal = (b - a).Cross(c - a); // if the point is inside the triangle
		}
		else if (sum == 2)
		{
			int e = -1;
			if (!test3)
				e = mesh.triangles[tri].e[0];
			else if (!test1)
				e = mesh.triangles[tri].e[1];
			else //if (!test2)
				e = mesh.triangles[tri].e[2];
			normal = mesh.edges[e].n;
		}
		else //if (sum == 1)
		{
			// TODO: make sure it's the angle weighed normal
			if (test1)
				normal = mesh.normals[i0];
			else if (test2)
				normal = mesh.normals[i1];
			else //if (test3)
				normal = mesh.normals[i2];
		}
	}

	return normal;
}

float ClosestPointOnMeshToPoint(const Vector3& p, const Mesh& mesh, ClosestTriangleToPoint& result)
{
	PROFILE_SCOPE("Closest pt-mesh");

	// brute force closest point
	float distMin = 1e10f;
	for (int i = 0; i < mesh.indices.size(); i += 3)
	{
		int i0 = mesh.indices[i];
		int i1 = mesh.indices[i + 1];
		int i2 = mesh.indices[i + 2];
		const Vector3& a = mesh.vertices[i0];
		const Vector3& b = mesh.vertices[i1];
		const Vector3& c = mesh.vertices[i2];
		Vector3 coords, pt;
		int region = ClosestPtPointTriangle(p, a, b, c, pt, coords);
		float dist = (pt - p).Length();
		if (dist < distMin) // TODO: parallel reduction
		{
			distMin = dist;
			result.distance = dist;
			result.tri = i / 3;
			result.closestPtMesh = pt;
			result.baryOnMesh = coords;
			result.normal = ComputeNormal(mesh, i / 3, coords); // TODO: delta vector normalized as an alternative
			result.region = region;
		}
	}
	
	return distMin;
}

ClosestTriangleToPoint ClosestPointOnMeshToPoint(const Vector3& p, const Mesh& mesh, const std::vector<int>& triangles)
{
	ClosestTriangleToPoint result;

	// brute force closest point
	float distMin = 1e10f;
	for (int j = 0; j < triangles.size(); j++)
	{
		int i = triangles[j] * 3;
		int i0 = mesh.indices[i];
		int i1 = mesh.indices[i + 1];
		int i2 = mesh.indices[i + 2];
		const Vector3& a = mesh.vertices[i0];
		const Vector3& b = mesh.vertices[i1];
		const Vector3& c = mesh.vertices[i2];
		Vector3 coords, pt;
		int region = ClosestPtPointTriangle(p, a, b, c, pt, coords);
		float dist = (pt - p).Length();
		if (dist < distMin) // TODO: parallel reduction
		{
			distMin = dist;
			result.distance = dist;
			result.tri = i / 3;
			result.closestPtMesh = pt;
			result.baryOnMesh = coords;
			result.normal = ComputeNormal(mesh, i / 3, coords); // TODO: delta vector normalized as an alternative
			result.region = region;
		}
	}

	return result;
}

void ClosestPointOnMeshToPointRec(const Vector3& p, const Mesh& mesh, const AabbTree* node, float& minDist, ClosestTriangleToPoint& result)
{
	ASSERT(node != nullptr);

	// first go through the triangles list, if not empty
	for (size_t i = 0; i < node->triangles.size(); i++)
	{
		const BoxInfo& boxInfo = node->triangles[i];
		int tri = boxInfo.idx;
		int i0 = mesh.indices[tri * 3 + 0];
		int i1 = mesh.indices[tri * 3 + 1];
		int i2 = mesh.indices[tri * 3 + 2];
		const Vector3& v0 = mesh.vertices[i0];
		const Vector3& v1 = mesh.vertices[i1];
		const Vector3& v2 = mesh.vertices[i2];
		Vector3 pt, coords;
		int region = ClosestPtPointTriangle(p, v0, v1, v2, pt, coords);
		float dist = (p - pt).Length();
		if (dist < minDist)
		{
			minDist = dist;
			result.distance = dist;
			result.tri = tri;
			result.closestPtMesh = pt;
			result.normal = ComputeNormal(mesh, tri, coords);
			result.baryOnMesh = coords;
			result.region = region;
		}
	}

	// compute distances to left and right boxes
	if (node->left == nullptr && node->right == nullptr)
		return;

	float distLeft = node->left != nullptr ? node->left->box.Distance(p) : FLT_MAX;
	float distRight = node->right != nullptr ? node->right->box.Distance(p) : FLT_MAX;

	// TODO: in IGL they also check if is inside each of the 2 boxes

	if (distLeft < distRight)
	{
		// first look left
		if (distLeft < minDist)
			ClosestPointOnMeshToPointRec(p, mesh, node->left, minDist, result);
		// if still needed, look right
		if (distRight < minDist)
			ClosestPointOnMeshToPointRec(p, mesh, node->right, minDist, result);
	}
	else
	{
		// first look right
		if (distRight < minDist)
			ClosestPointOnMeshToPointRec(p, mesh, node->right, minDist, result);
		// if still needed, look left
		if (distLeft < minDist)
			ClosestPointOnMeshToPointRec(p, mesh, node->left, minDist, result);
	}
}

ClosestTriangleToPoint ClosestPointOnMeshToPointAcc(const Vector3& p, const Mesh& mesh, const AabbTree* tree, int seed)
{
	ClosestTriangleToPoint result;
	if (seed >= 0)
	{
		int i0 = mesh.indices[seed * 3 + 0];
		int i1 = mesh.indices[seed * 3 + 1];
		int i2 = mesh.indices[seed * 3 + 2];
		const Vector3& v0 = mesh.vertices[i0];
		const Vector3& v1 = mesh.vertices[i1];
		const Vector3& v2 = mesh.vertices[i2];
		Vector3 pt, coords;
		int region = ClosestPtPointTriangle(p, v0, v1, v2, pt, coords);
		float dist = (p - pt).Length();

		// if the point is still inside the Voronoi region of the triangle
		// TODO: use the triangle "cell" instead, as it also includes borders
		if (region == TR_FACE_INTERIOR)
		{
			result.distance = dist;
			result.tri = seed;
			result.closestPtMesh = pt;
			result.normal = ComputeNormal(mesh, seed, coords);
			result.baryOnMesh = coords;
			result.region = region;
			return result;
		}
	}

	float minDist = FLT_MAX;
	ClosestPointOnMeshToPointRec(p, mesh, tree, minDist, result);
	return result;
}

float ClosestPointOnMeshToTriangle(Vector3 a, Vector3 b, Vector3 c, const Mesh& mesh, ClosestVertexToTriangle& result)
{
	// It's possible that the triangle is actually intersecting the mesh and then the closest point is ill-defined.
	// The true closest points would be those along the intersection poly-line.
	// But what we are really looking for is the minimum unsigned distance (which we are doing here).
	// The hope is that the found vertex is actually the endpoint of the intersecting edge (one of them).
	// This closest point on the mesh then has a closest distance counterpart on the triangle, and this
	// should correspond to the maximum penetration depth of the triangle inside the mesh.
	// This maximum penetration should correspond to a minimum distance to the medial axis of the mesh.
	// It should be proven though that this pair can only be made between a triangle a triangle and a vertex.
	// It could be for example that this pair is achieved between an edge of the triangle and one of the mesh.

	// TODO: closest edge to one of the 3 triangle edges; and then pick the minimum distance

	float distMin = FLT_MAX;
	// brute force: go through all the vertices of the mesh
	for (int i = 0; i < mesh.vertices.size(); i++)
	{
		Vector3 bary, cp;
		int region = ClosestPtPointTriangle(mesh.vertices[i], a, b, c, cp, bary);
		Vector3 delta = cp - mesh.vertices[i];
		float dist = delta.Length();
		if (dist < distMin)
		{
			distMin = dist;
			result.vertex = i;
			result.closestPtMesh = mesh.vertices[i];
			result.closestPtTri = cp;
			result.baryCoords = bary;
			result.normal = mesh.normals[i]; // use the vertex normal for now; TODO: other options
			result.distance = dist;
			result.region = region;
		}
	}

	return distMin;
}

float ClosestPointOnMeshToTriangle(Vector3 a, Vector3 b, Vector3 c, const Mesh& mesh, const std::vector<int>& vertexSet, ClosestVertexToTriangle& result)
{
	if (vertexSet.size() == 0)
	{
		result.distance = FLT_MAX;
		return FLT_MAX;
	}

	float distMin = FLT_MAX;
	for (int j = 0; j < vertexSet.size(); j++)
	{
		int i = vertexSet[j];
		Vector3 bary, cp;
		int region = ClosestPtPointTriangle(mesh.vertices[i], a, b, c, cp, bary);
		Vector3 delta = cp - mesh.vertices[i];
		float dist = delta.Length();
		if (dist < distMin)
		{
			distMin = dist;
			result.vertex = i;
			result.closestPtMesh = mesh.vertices[i];
			result.closestPtTri = cp;
			result.baryCoords = bary;
			result.normal = mesh.normals[i]; // use the vertex normal for now; TODO: other options
			result.distance = dist;
			result.region = region;
		}
	}

	return distMin;
}

void ClosestPointOnMeshToTriangleRec(Vector3 a, Vector3 b, Vector3 c, const Mesh& mesh, const AabbTree* node, float& minDist, ClosestVertexToTriangle& result)
{
	ASSERT(node != nullptr);

	// first go through the vertex list, if not empty
	for (size_t i = 0; i < node->vertices.size(); i++)
	{
		const BoxInfo& boxInfo = node->vertices[i];
		int v = boxInfo.idx;
		Vector3 bary, cp;
		int region = ClosestPtPointTriangle(mesh.vertices[v], a, b, c, cp, bary);
		Vector3 delta = cp - mesh.vertices[v];
		float dist = delta.Length();
		if (dist < minDist)
		{
			minDist = dist;
			result.vertex = v;
			result.closestPtMesh = mesh.vertices[v];
			result.closestPtTri = cp;
			result.baryCoords = bary;
			result.normal = mesh.normals[v]; // use the vertex normal for now; TODO: other options
			result.distance = dist;
			result.region = region;
		}
	}

	// compute distances to left and right boxes
	if (node->left == nullptr && node->right == nullptr)
		return;

	Aabb3 triBox;
	triBox.Add(a);
	triBox.Add(b);
	triBox.Add(c);

	float distLeft = node->left != nullptr ? node->left->box.Distance(triBox) : FLT_MAX;
	float distRight = node->right != nullptr ? node->right->box.Distance(triBox) : FLT_MAX;

	if (distLeft < distRight)
	{
		// first look left
		if (distLeft < minDist)
			ClosestPointOnMeshToTriangleRec(a, b, c, mesh, node->left, minDist, result);
		// if still needed, look right
		if (distRight < minDist)
			ClosestPointOnMeshToTriangleRec(a, b, c, mesh, node->right, minDist, result);
	}
	else
	{
		// first look right
		if (distRight < minDist)
			ClosestPointOnMeshToTriangleRec(a, b, c, mesh, node->right, minDist, result);
		// if still needed, look left
		if (distLeft < minDist)
			ClosestPointOnMeshToTriangleRec(a, b, c, mesh, node->left, minDist, result);
	}
}

ClosestVertexToTriangle ClosestPointOnMeshToTriangleAcc(Vector3 a, Vector3 b, Vector3 c, const Mesh& mesh, const AabbTree* node, int seed)
{
	ClosestVertexToTriangle result;
	if (seed >= 0)
	{
		Vector3 bary, cp;
		int region = ClosestPtPointTriangle(mesh.vertices[seed], a, b, c, cp, bary);
		Vector3 delta = cp - mesh.vertices[seed];
		float dist = delta.Length();

		// Test with the Voronoi region of the seed vertex, not that of the triangle
		if (TestPointInVertexVoronoiRegion(cp, mesh, seed))
		{
			result.vertex = seed;
			result.closestPtMesh = mesh.vertices[seed];
			result.closestPtTri = cp;
			result.baryCoords = bary;
			result.normal = mesh.normals[seed]; // use the vertex normal for now; TODO: other options
			result.distance = dist;
			result.region = region;
			return result;
		}
	}
	float minDist = FLT_MAX;
	ClosestPointOnMeshToTriangleRec(a, b, c, mesh, node, minDist, result);
	return result;
}

float ClosestPointOnMeshToSegment(Vector3 p, Vector3 q, const Mesh& mesh, ClosestEdgeToSegment& result)
{
	float distMin = FLT_MAX;
	for (int i = 0; i < mesh.edges.size(); i++)
	{
		int i1 = mesh.edges[i].i1;
		int i2 = mesh.edges[i].i2;
		Vector3 a = mesh.vertices[i1];
		Vector3 b = mesh.vertices[i2];
		float paramSegm, paramMesh;
		Vector3 cpSegm, cpMesh;
		Geometry::ClosestPtSegmSegm(p, q, a, b, paramSegm, paramMesh, cpSegm, cpMesh);
		float dist = (cpSegm - cpMesh).Length();
		if (dist < distMin)
		{
			distMin = dist;
			result.edge = i;
			result.distance = dist;
			result.closestPtMesh = cpMesh;
			result.closestPtSegment = cpSegm;
			result.coordsSegm.x = 1.0f - paramSegm;
			result.coordsSegm.y = paramSegm;
			result.coordsMesh.x = 1.0f - paramMesh;
			result.coordsMesh.y = paramMesh;
			result.normal = mesh.edges[i].n; // use the edge pseudo-normal

			// this is a special case when the closest point is a vertex => use vertex normal
			// for now, we use 0 and 1 as results of clamping from ClosestPtSegmSegm; anything else is considered on the edge
			if (paramMesh == 0.f)
				result.normal = mesh.normals[i1];
			if (paramMesh == 1.f)
				result.normal = mesh.normals[i2];
		}
	}

	return distMin;
}

void ClosestPointOnMeshToSegmentRec(Vector3 p, Vector3 q, const Mesh& mesh, const AabbTree* node, float& minDist, ClosestEdgeToSegment& result)
{
	ASSERT(node != nullptr);

	// first go through the vertex list, if not empty
	for (size_t e = 0; e < node->edges.size(); e++)
	{
		const BoxInfo& boxInfo = node->edges[e];
		int i = boxInfo.idx;

		int i1 = mesh.edges[i].i1;
		int i2 = mesh.edges[i].i2;
		Vector3 a = mesh.vertices[i1];
		Vector3 b = mesh.vertices[i2];
		float paramSegm, paramMesh;
		Vector3 cpSegm, cpMesh;
		Geometry::ClosestPtSegmSegm(p, q, a, b, paramSegm, paramMesh, cpSegm, cpMesh);
		float dist = (cpSegm - cpMesh).Length();
		if (dist < minDist)
		{
			minDist = dist;
			result.distance = dist;
			result.edge = i;
			result.closestPtMesh = cpMesh;
			result.closestPtSegment = cpSegm;
			result.coordsSegm.x = 1.0f - paramSegm;
			result.coordsSegm.y = paramSegm;
			result.coordsMesh.x = 1.0f - paramMesh;
			result.coordsMesh.y = paramMesh;
			result.normal = mesh.edges[i].n; // use the edge pseudo-normal

			// this is a special case when the closest point is a vertex => use vertex normal
			// for now, we use 0 and 1 as results of clamping from ClosestPtSegmSegm; anything else is considered on the edge
			if (paramMesh == 0.f)
				result.normal = mesh.normals[i1];
			if (paramMesh == 1.f)
				result.normal = mesh.normals[i2];
		}
	}

	// compute distances to left and right boxes
	if (node->left == nullptr && node->right == nullptr)
		return;

	Aabb3 edgeBox;
	edgeBox.Add(p);
	edgeBox.Add(q);

	float distLeft = node->left != nullptr ? node->left->box.Distance(edgeBox) : FLT_MAX;
	float distRight = node->right != nullptr ? node->right->box.Distance(edgeBox) : FLT_MAX;

	if (distLeft < distRight)
	{
		// first look left
		if (distLeft < minDist)
			ClosestPointOnMeshToSegmentRec(p, q, mesh, node->left, minDist, result);
		// if still needed, look right
		if (distRight < minDist)
			ClosestPointOnMeshToSegmentRec(p, q, mesh, node->right, minDist, result);
	}
	else
	{
		// first look right
		if (distRight < minDist)
			ClosestPointOnMeshToSegmentRec(p, q, mesh, node->right, minDist, result);
		// if still needed, look left
		if (distLeft < minDist)
			ClosestPointOnMeshToSegmentRec(p, q, mesh, node->left, minDist, result);
	}
}

float ClosestPointOnMeshToSegmentAcc(Vector3 p, Vector3 q, const Mesh& mesh, const AabbTree* node, int seed, ClosestEdgeToSegment& result)
{
	const float eps = 0.09f;

	if (seed >= 0)
	{
		int i1 = mesh.edges[seed].i1;
		int i2 = mesh.edges[seed].i2;
		Vector3 a = mesh.vertices[i1];
		Vector3 b = mesh.vertices[i2];
		float paramSegm, paramMesh;
		Vector3 cpSegm, cpMesh;
		int region = Geometry::ClosestPtSegmSegm(p, q, a, b, paramSegm, paramMesh, cpSegm, cpMesh);
		
		// first check if we're not too close to one of the vertices on the mesh
		bool inVR = true;
		if (region != ER_EDGE2_INTERIOR || paramMesh < eps || paramMesh > 1.f - eps) // TODO: replace the eps check with the dots below
		{
			inVR = false;
		}
		else
		{
			// now check if cpSegm is inside the Voronoi region of the mesh edge - only test with the endpoint planes
			// curios, if we ever hit this after the previous test... probably not (but not expensive anyway)

			// compute the edge direction (aligned with the first triangle)
			Vector3 e = b - a;
			e.Normalize();

			float dot3 = e.Dot(cpSegm - a);
			float dot4 = e.Dot(cpSegm - b);

			if (dot3 < 0 || dot4 > 0)
				inVR = false;
		}

		if (inVR)
		{
			float dist = (cpSegm - cpMesh).Length();
			result.distance = dist;
			result.edge = seed;
			result.closestPtMesh = cpMesh;
			result.coordsSegm.x = 1.0f - paramSegm;
			result.coordsSegm.y = paramSegm;
			result.coordsMesh.x = 1.0f - paramMesh;
			result.coordsMesh.y = paramMesh;
			result.normal = mesh.edges[seed].n; // use the edge pseudo-normal
			return dist;
		}
	}

	float minDist = FLT_MAX;
	ClosestPointOnMeshToSegmentRec(p, q, mesh, node, minDist, result);

	return minDist;
}



} // namespace Geometry