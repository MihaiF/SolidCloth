#pragma once

#include <Math/Vector3.h>
#include <Geometry/Mesh.h>

namespace Geometry
{
	Math::Vector3 ComputeNormal(const Mesh& mesh, int tri, Math::Vector3 coords, bool usePseudoNormal = true);

	int ClosestVertexOnMeshToPoint(const Math::Vector3& p, const Mesh& mesh);

	struct ClosestTriangleToPoint
	{
		Math::Vector3 closestPtMesh;
		Math::Vector3 normal;
		Math::Vector3 baryOnMesh;
		float distance;
		int tri;
		int region = -1;
	};

	float ClosestPointOnMeshToPoint(const Math::Vector3& p, const Mesh& mesh, ClosestTriangleToPoint& result);

	ClosestTriangleToPoint ClosestPointOnMeshToPoint(const Vector3& p, const Mesh& mesh, const std::vector<int>& triangles);
	
	ClosestTriangleToPoint ClosestPointOnMeshToPointAcc(const Math::Vector3& p, const Mesh& mesh, const struct AabbTree* tree, int seed = -1);

	struct ClosestEdgeToSegment
	{
		Math::Vector3 closestPtMesh;
		Math::Vector3 closestPtSegment;
		Math::Vector3 normal;
		Math::Vector2 coordsMesh;
		Math::Vector2 coordsSegm;
		int edge;
		float distance;
	};
	
	float ClosestPointOnMeshToSegment(Math::Vector3 p, Math::Vector3 q, const Geometry::Mesh& mesh, ClosestEdgeToSegment& result);

	float ClosestPointOnMeshToSegmentAcc(Math::Vector3 p, Math::Vector3 q, const Mesh& mesh, const AabbTree* node, int seed, ClosestEdgeToSegment& result);

	struct ClosestVertexToTriangle
	{
		Math::Vector3 closestPtMesh;
		Math::Vector3 closestPtTri;
		Math::Vector3 normal;
		Math::Vector3 baryCoords;
		float distance;
		int vertex = -1;
		int region;
	};

	// Returns the distance and closest point to a mesh from a triangle
	// The closest point is always located on a vertex
	float ClosestPointOnMeshToTriangle(Math::Vector3 a, Math::Vector3 b, Math::Vector3 c, const Mesh& mesh, ClosestVertexToTriangle& result);

	float ClosestPointOnMeshToTriangle(Vector3 a, Vector3 b, Vector3 c, const Mesh& mesh, const std::vector<int>& vertexSet, ClosestVertexToTriangle& result);

	ClosestVertexToTriangle ClosestPointOnMeshToTriangleAcc(Math::Vector3 a, Math::Vector3 b, Math::Vector3 c, const Mesh& mesh, const AabbTree* node, int seed = -1);
}