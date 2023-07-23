#include "ClosestPointOnMesh.h"

#include <Math/Vector3.h>
#include <Geometry/Mesh.h>
#include <Geometry/AabbTree.h>
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

Vector3 ComputeNormal(const Mesh& mesh, int tri, BarycentricCoords coords, bool usePseudoNormal = true)
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

float ClosestPointOnMeshToPoint(const Vector3& p, const Mesh& mesh, Vector3& closestPt, Vector3& normal, BarycentricCoords& coords)
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
		Vector3 pt = ClosestPtPointTriangle(p, a, b, c, coords);
		float dist = (pt - p).Length();
		if (dist < distMin) // TODO: parallel reduction
		{
			distMin = dist;
			closestPt = pt;
			normal = ComputeNormal(mesh, i / 3, coords);
			// TODO: delta vector normalized as an alternative
		}
	}
	
	return distMin;
}

void ClosestPointOnMeshToPointRec(const Vector3& p, const Mesh& mesh, const AabbTree* node, float& minDist, Vector3& closestPt, Vector3& normal)
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
		BarycentricCoords coords;
		Vector3 pt = ClosestPtPointTriangle(p, v0, v1, v2, coords);
		float dist = (p - pt).Length();
		if (dist < minDist)
		{
			minDist = dist;
			closestPt = pt;
			normal = ComputeNormal(mesh, tri, coords);
		}
	}

	// compute distances to left and right boxes
	if (node->left == nullptr && node->right == nullptr)
		return;

	float distLeft = node->left != nullptr ? AabbDistance(p, node->left->box.min, node->left->box.max) : FLT_MAX;
	float distRight = node->right != nullptr ? AabbDistance(p, node->right->box.min, node->right->box.max) : FLT_MAX;

	// TODO: in IGL they also check if is inside each of the 2 boxes

	if (distLeft < distRight)
	{
		// first look left
		if (distLeft < minDist)
			ClosestPointOnMeshToPointRec(p, mesh, node->left, minDist, closestPt, normal);
		// if still needed, look right
		if (distRight < minDist)
			ClosestPointOnMeshToPointRec(p, mesh, node->right, minDist, closestPt, normal);
	}
	else
	{
		// first look right
		if (distRight < minDist)
			ClosestPointOnMeshToPointRec(p, mesh, node->right, minDist, closestPt, normal);
		// if still needed, look left
		if (distLeft < minDist)
			ClosestPointOnMeshToPointRec(p, mesh, node->left, minDist, closestPt, normal);
	}
}

float ClosestPointOnMeshToPointAcc(const Vector3& p, const Mesh& mesh, const AabbTree* tree, Vector3& closestPt, Vector3& normal)
{
	float minDist = FLT_MAX;
	ClosestPointOnMeshToPointRec(p, mesh, tree, minDist, closestPt, normal);
	return minDist;
}

float ClosestPointOnMeshToTriangle(Vector3 a, Vector3 b, Vector3 c, const Mesh& mesh, ClosestMeshTriangle& result)
{
	float distMin = FLT_MAX;
	// brute force: go through all the vertices of the mesh
	for (int i = 0; i < mesh.vertices.size(); i++)
	{
		Vector3 bary;
		Vector3 cp = ClosestPtPointTriangle(mesh.vertices[i], a, b, c, bary);
		Vector3 delta = cp - mesh.vertices[i];
		float dist = delta.Length();
		if (dist < distMin)
		{
			distMin = dist;
			result.vertex = i;
			result.closestPtMesh = mesh.vertices[i];
			result.closestPtTri = bary.x * a + bary.y * b + bary.z * c;
			result.baryCoords = bary;
			result.normal = mesh.normals[i]; // use the vertex normal for now; TODO: other options
			result.distance = dist;
		}
	}

	return distMin;
}

// not really working
float ClosestPointOnMeshToTriangleOMP(Vector3 a, Vector3 b, Vector3 c, const Mesh& mesh, ClosestMeshTriangle& info)
{
	ClosestMeshTriangle results[20];
	int numThreads;

	#pragma omp parallel num_threads(4)
	{
		numThreads = omp_get_num_threads();

		int n = mesh.vertices.size() / numThreads;
		int id = omp_get_thread_num();
		int start = id * n;
		int stop;
		if (id != (numThreads - 1))
			stop = start + n;
		else
			stop = mesh.vertices.size();

		ClosestMeshTriangle& result = results[id];
		float distMin = FLT_MAX;

		for (int i = start; i < stop; i++)
		{
			Vector3 bary;
			Vector3 cp = ClosestPtPointTriangle(mesh.vertices[i], a, b, c, bary);
			Vector3 delta = cp - mesh.vertices[i];
			float dist = delta.Length();
			if (dist < distMin)
			{
				distMin = dist;
				result.vertex = i;
				result.closestPtMesh = mesh.vertices[i];
				result.closestPtTri = bary.x * a + bary.y * b + bary.z * c;
				result.baryCoords = bary;
				result.normal = mesh.normals[i]; // use the vertex normal for now; TODO: other options
				result.distance = dist;
			}
		}
	}

	float distMin = FLT_MAX;
	for (int i = 0; i < numThreads; i++)
	{
		if (results[i].distance < distMin)
		{
			distMin = results[i].distance;
			info = results[i];
		}
	}

	return distMin;
}

} // namespace Geometry