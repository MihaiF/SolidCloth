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

Vector3 ComputeTriangleNormal(const Mesh& mesh, int tri, Vector3 coords, bool usePseudoNormal, bool useMeshNormal)
{
	Vector3 normal;

	int i0 = mesh.indices[tri * 3 + 0];
	int i1 = mesh.indices[tri * 3 + 1];
	int i2 = mesh.indices[tri * 3 + 2];

	if (mesh.edges.size() == 0)
		usePseudoNormal = false;

	if (!usePseudoNormal)
	{
		if (!useMeshNormal)
		{
			Vector3 a = mesh.vertices[i0];
			Vector3 b = mesh.vertices[i1];
			Vector3 c = mesh.vertices[i2];
			normal = (b - a).Cross(c - a);
		}
		else
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

void ClosestTriKernel(Vector3 p, const Mesh& mesh, int tri, float& distMin, ClosestTriangleToPoint& result, bool useDelta = true)
{
	int i0 = mesh.indices[tri * 3];
	int i1 = mesh.indices[tri * 3 + 1];
	int i2 = mesh.indices[tri * 3 + 2];
	const Vector3& a = mesh.vertices[i0];
	const Vector3& b = mesh.vertices[i1];
	const Vector3& c = mesh.vertices[i2];
	Vector3 coords, pt;
	int region = ClosestPtPointTriangle(p, a, b, c, pt, coords);
	Vector3 delta = p - pt;
	float dist = delta.Length();
	if (dist == 0)
		return; // this is the point itself!
	if (dist < distMin) // TODO: parallel reduction
	{
		distMin = dist;
		result.distance = dist;
		result.tri = tri;
		result.closestPtMesh = pt;
		result.baryOnMesh = coords;
		if (!useDelta)
			result.normal = ComputeTriangleNormal(mesh, tri, coords);
		else
			result.normal = (1.f / dist) * delta;
		result.region = region;
	}
}

float ClosestPointOnMeshToPoint(Vector3 p, const Mesh& mesh, ClosestTriangleToPoint& result)
{
	PROFILE_SCOPE("Closest pt-mesh");

	// brute force closest point
	float distMin = 1e10f;
	for (int i = 0; i < mesh.GetNumTriangles(); i++)
	{
		ClosestTriKernel(p, mesh, i, distMin, result);
	}
	
	return distMin;
}

void ClosestPointsOnMeshToPoint(Vector3 p, const Mesh& mesh, std::vector<ClosestTriangleToPoint>& results, float maxDist, int idx)
{
	//PROFILE_SCOPE("ClosestPointsOnMeshToPoint");
	bool isBorder = mesh.IsBorderVertex(idx);

	// brute force closest point
	for (int i = 0; i < mesh.GetNumTriangles(); i++)
	{
		int i0 = mesh.indices[i * 3];
		int i1 = mesh.indices[i * 3 + 1];
		int i2 = mesh.indices[i * 3 + 2];
		const Vector3& a = mesh.vertices[i0];
		const Vector3& b = mesh.vertices[i1];
		const Vector3& c = mesh.vertices[i2];
		Vector3 coords, pt;
		int region = ClosestPtPointTriangle(p, a, b, c, pt, coords);
		bool inVR = idx >= 0 ? TestPointInVertexVoronoiRegion(pt, mesh, idx) : false;
		Vector3 delta = p - pt;
		float dist = delta.Length();
		if (dist > maxDist)
			continue;
		if (dist == 0)
			continue; // this is the point itself!
		ClosestTriangleToPoint result;
		if (region == TR_FACE_INTERIOR)
		{
			if (!(inVR || TestPointInTriangleVoronoiRegion(p, mesh, i)))
				continue;
			result.regionType = 0;
		}
		if (region == TR_EDGE_AB)
		{
			int e = mesh.triangles[i].e[0];
			if (!mesh.IsBorderEdge(e) && !(inVR || TestPointInEdgeVoronoiRegion(p, mesh, e)))
				continue;
			result.regionType = 1;
			result.feature = mesh.triangles[i].e[0];
		}
		if (region == TR_EDGE_BC)
		{
			int e = mesh.triangles[i].e[1];
			if (!mesh.IsBorderEdge(e) && !(inVR || TestPointInEdgeVoronoiRegion(p, mesh, e)))
				continue;
			result.regionType = 1;
			result.feature = mesh.triangles[i].e[1];
		}
		if (region == TR_EDGE_AC)
		{
			int e = mesh.triangles[i].e[2];
			if (!mesh.IsBorderEdge(e) && !(inVR || TestPointInEdgeVoronoiRegion(p, mesh, e)))
				continue;
			result.regionType = 1;
			result.feature = mesh.triangles[i].e[2];
		}
		if (region == TR_VERTEX_A)
		{
			if (!isBorder && !(inVR || TestPointInVertexVoronoiRegion(p, mesh, i0)))
				continue;
			result.regionType = 2;
			result.feature = i0;
		}
		if (region == TR_VERTEX_B)
		{
			if (!isBorder && !(inVR || TestPointInVertexVoronoiRegion(p, mesh, i1)))
				continue;
			result.regionType = 2;
			result.feature = i1;
		}
		if (region == TR_VERTEX_C)
		{
			if (!isBorder && !(inVR || TestPointInVertexVoronoiRegion(p, mesh, i2)))
				continue;
			result.regionType = 2;
			result.feature = i2;
		}

		// search if the same feature appeared before
		if (result.feature >= 0)
		{
			bool found = false;
			for (int j = 0; j < results.size(); j++)
			{
				if (result.regionType == results[j].regionType && result.feature == results[j].feature)
				{
					found = true;
					break;
				}
			}
			if (found)
				continue;
		}

		result.distance = dist;
		result.tri = i;
		result.closestPtMesh = pt;
		result.baryOnMesh = coords;
		//if (!useDelta)
		//	result.normal = ComputeTriangleNormal(mesh, i / 3, coords);
		//else
		result.normal = (1.f / dist) * delta;
		result.region = region;
		result.side = SignedVolume(p, a, b, c);
		results.push_back(result);
	}
}

ClosestTriangleToPoint ClosestPointOnMeshToPoint(Vector3 p, const Mesh& mesh, const std::vector<int>& triangles)
{
	ClosestTriangleToPoint result;

	// brute force closest point
	float distMin = 1e10f;
	for (int j = 0; j < triangles.size(); j++)
	{
		ClosestTriKernel(p, mesh, triangles[j], distMin, result);
	}

	return result;
}

void ClosestPointOnMeshToPointRec(Vector3 p, const Mesh& mesh, const AabbTree* node, float& minDist, 
	ClosestTriangleToPoint& result, bool useDeltaNormal)
{
	ASSERT(node != nullptr);

	// first go through the triangles list, if not empty
	for (size_t i = 0; i < node->triangles.size(); i++)
	{
		const BoxInfo& boxInfo = node->triangles[i];
		int tri = boxInfo.idx;
		ClosestTriKernel(p, mesh, tri, minDist, result, useDeltaNormal);
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
			ClosestPointOnMeshToPointRec(p, mesh, node->left, minDist, result, useDeltaNormal);
		// if still needed, look right
		if (distRight < minDist)
			ClosestPointOnMeshToPointRec(p, mesh, node->right, minDist, result, useDeltaNormal);
	}
	else
	{
		// first look right
		if (distRight < minDist)
			ClosestPointOnMeshToPointRec(p, mesh, node->right, minDist, result, useDeltaNormal);
		// if still needed, look left
		if (distLeft < minDist)
			ClosestPointOnMeshToPointRec(p, mesh, node->left, minDist, result, useDeltaNormal);
	}
}

ClosestTriangleToPoint ClosestPointOnMeshToPointAcc(Vector3 p, const Mesh& mesh, const AabbTree* tree, int seed, bool useMeshNormal)
{
	ClosestTriangleToPoint result;
	float minDist = FLT_MAX;

	if (seed >= 0)
	{
		ClosestTriKernel(p, mesh, seed, minDist, result);

		// if the point is still inside the Voronoi region of the triangle
		// TODO: use the triangle "cell" instead, as it also includes borders
		if (result.region == TR_FACE_INTERIOR)
			return result;
	}

	ClosestPointOnMeshToPointRec(p, mesh, tree, minDist, result, useMeshNormal);
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
		if (dist == 0) // the point itself
			continue;
		if (dist < minDist)
		{
			minDist = dist;
			result.vertex = v;
			result.closestPtMesh = mesh.vertices[v];
			result.closestPtTri = cp;
			result.baryCoords = bary;
			//result.normal = mesh.normals[v]; // use the vertex normal for now; TODO: other options
			result.normal = (1.f / dist) * delta;
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
	//if (seed >= 0)
	//{
	//	Vector3 bary, cp;
	//	int region = ClosestPtPointTriangle(mesh.vertices[seed], a, b, c, cp, bary);
	//	Vector3 delta = cp - mesh.vertices[seed];
	//	float dist = delta.Length();

	//	// Test with the Voronoi region of the seed vertex, not that of the triangle
	//	if (TestPointInVertexVoronoiRegion(cp, mesh, seed))
	//	{
	//		result.vertex = seed;
	//		result.closestPtMesh = mesh.vertices[seed];
	//		result.closestPtTri = cp;
	//		result.baryCoords = bary;
	//		//result.normal = mesh.normals[seed]; // use the vertex normal for now; TODO: other options
	//		result.normal = (1.f / dist) * delta;
	//		result.distance = dist;
	//		result.region = region;
	//		return result;
	//	}
	//}
	float minDist = FLT_MAX;
	ClosestPointOnMeshToTriangleRec(a, b, c, mesh, node, minDist, result); // TODO: representative triangle alternative
	return result;
}

Vector3 ComputeEdgeNormal(const Mesh& mesh, int e, float param)
{
	int i1 = mesh.edges[e].i1;
	int i2 = mesh.edges[e].i2;
	Vector3 normal = mesh.edges[e].n; // use the edge pseudo-normal

	// this is a special case when the closest point is a vertex => use vertex normal
	// for now, we use 0 and 1 as results of clamping from ClosestPtSegmSegm; anything else is considered on the edge
	if (param == 0.f)
		normal = mesh.normals[i1];
	if (param == 1.f)
		normal = mesh.normals[i2];

	return normal;
}

float ClosestPointOnMeshToSegment(Vector3 p, Vector3 q, const Mesh& mesh, ClosestEdgeToSegment& result, int e2)
{
	float distMin = FLT_MAX;
	for (int i = 0; i < mesh.edges.size(); i++)
	{
		int i1 = mesh.edges[i].i1;
		int i2 = mesh.edges[i].i2;
		if (e2 >= 0)
		{
			const Mesh::Edge& edge2 = mesh.edges[e2];
			if (i1 == edge2.i1 || i1 == edge2.i2 || i2 == edge2.i1 || i2 == edge2.i2)
			{
				continue;
			}
		}
		Vector3 a = mesh.vertices[i1];
		Vector3 b = mesh.vertices[i2];
		float paramSegm, paramMesh;
		Vector3 cpSegm, cpMesh;
		int region = Geometry::ClosestPtSegmSegm(p, q, a, b, paramSegm, paramMesh, cpSegm, cpMesh);
		float dist = (cpSegm - cpMesh).Length();
		if (dist != 0 && dist < distMin)
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
			if (e2 >= 0) // self
				result.normal = cpMesh - cpSegm;
			else
				result.normal = ComputeEdgeNormal(mesh, i, paramMesh);
			result.normal.Normalize();
			result.side = SignedVolume(p, q, a, b);
		}
	}

	return distMin;
}

float ClosestPointOnMeshToSegment(Vector3 p, Vector3 q, const Mesh& mesh, const std::vector<int>& edgeSet, ClosestEdgeToSegment& result)
{
	if (edgeSet.size() == 0)
	{
		result.distance = FLT_MAX;
		return FLT_MAX;
	}

	float distMin = FLT_MAX;
	for (int j = 0; j < edgeSet.size(); j++)
	{
		int i = edgeSet[j];
		int i1 = mesh.edges[i].i1;
		int i2 = mesh.edges[i].i2;
		Vector3 a = mesh.vertices[i1];
		Vector3 b = mesh.vertices[i2];
		float paramSegm, paramMesh;
		Vector3 cpSegm, cpMesh;
		int region = Geometry::ClosestPtSegmSegm(p, q, a, b, paramSegm, paramMesh, cpSegm, cpMesh);
		float dist = (cpSegm - cpMesh).Length();
		const float eps = 0.01f;
		bool notInterior = region != ER_EDGE_INTERIOR || !(paramSegm > eps && paramSegm < 1.f - eps && paramMesh > eps && paramMesh < 1.f - eps);
		if (notInterior)
		{
			//minDist = dist;
			continue;
		}
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
			result.normal = ComputeEdgeNormal(mesh, i, paramMesh);
		}
	}

	return distMin;
}

void ClosestPointOnMeshToSegmentEdgeRec(Vector3 p, Vector3 q, const Mesh& mesh, const AabbTree* edgeTree, bool useDelta, float& minDist, ClosestEdgeToSegment& result)
{
	ASSERT(edgeTree != nullptr);

	// first go through the edges list, if not empty
	for (size_t e = 0; e < edgeTree->edges.size(); e++)
	{
		const BoxInfo& boxInfo = edgeTree->edges[e];
		int i = boxInfo.idx;

		int i1 = mesh.edges[i].i1;
		int i2 = mesh.edges[i].i2;
		Vector3 a = mesh.vertices[i1];
		Vector3 b = mesh.vertices[i2];
		float paramSegm, paramMesh;
		Vector3 cpSegm, cpMesh;
		int region = ClosestPtSegmSegm(p, q, a, b, paramSegm, paramMesh, cpSegm, cpMesh);
		const float eps = 0.001f;
		Vector3 delta = cpMesh - cpSegm;
		float dist = delta.Length();
		if (dist == 0)
			continue; // the point itself
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
			if (useDelta)
				result.normal = (1.f / dist) * delta;
			else
				result.normal = ComputeEdgeNormal(mesh, i, paramMesh);
			result.region = region;
		}
	}

	// compute distances to left and right boxes
	if (edgeTree->left == nullptr && edgeTree->right == nullptr)
		return;

	Aabb3 edgeBox;
	edgeBox.Add(p);
	edgeBox.Add(q);

	float distLeft = edgeTree->left != nullptr ? edgeTree->left->box.Distance(edgeBox) : FLT_MAX;
	float distRight = edgeTree->right != nullptr ? edgeTree->right->box.Distance(edgeBox) : FLT_MAX;

	if (distLeft < distRight)
	{
		// first look left
		if (distLeft < minDist)
			ClosestPointOnMeshToSegmentEdgeRec(p, q, mesh, edgeTree->left, useDelta, minDist, result);
		// if still needed, look right
		if (distRight < minDist)
			ClosestPointOnMeshToSegmentEdgeRec(p, q, mesh, edgeTree->right, useDelta, minDist, result);
	}
	else
	{
		// first look right
		if (distRight < minDist)
			ClosestPointOnMeshToSegmentEdgeRec(p, q, mesh, edgeTree->right, useDelta, minDist, result);
		// if still needed, look left
		if (distLeft < minDist)
			ClosestPointOnMeshToSegmentEdgeRec(p, q, mesh, edgeTree->left, useDelta, minDist, result);
	}
}

float ClosestPointOnMeshToSegmentAcc(Vector3 p, Vector3 q, const Mesh& mesh, const AabbTree* tree, bool useDelta, int seed, ClosestEdgeToSegment& result)
{
	float minDist = FLT_MAX;
	ClosestPointOnMeshToSegmentEdgeRec(p, q, mesh, tree, useDelta, minDist, result);

	return minDist;
}



} // namespace Geometry