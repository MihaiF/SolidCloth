#pragma once

#include <Math/Vector3.h>
#include <Geometry/Mesh.h>

namespace Geometry
{
	Math::Vector3 ComputeTriangleNormal(const Mesh& mesh, int tri, Math::Vector3 coords, bool usePseudoNormal = true, bool useMeshNormal = true);

	int ClosestVertexOnMeshToPoint(const Math::Vector3& p, const Mesh& mesh);

	struct ClosestEdgeToPoint
	{
		Math::Vector3 closestPtMesh;
		Math::Vector3 normal;
		Math::Vector2 coordsMesh;
		float distance = FLT_MAX;
		int edge = -1;
	};

	struct ClosestTriangleToPoint
	{
		Math::Vector3 closestPtMesh;
		Math::Vector3 normal;
		Math::Vector3 baryOnMesh;
		float distance = FLT_MAX;
		int tri = -1;
		int vtx = -1;
		int feature = -1; // edge or vertex index
		int region = -1;
		int regionType = -1; // 0 - face, 1 - edge, 2 - vertex
		float side = FLT_MAX;
	};

	float ClosestPointOnMeshToPoint(Math::Vector3 p, const Mesh& mesh, ClosestTriangleToPoint& result);

	void ClosestPointsOnMeshToPoint(Math::Vector3 p, const Mesh& mesh, std::vector<ClosestTriangleToPoint>& results, float maxDist = FLT_MAX, int idx = -1);

	ClosestTriangleToPoint ClosestPointOnMeshToPoint(Math::Vector3 p, const Mesh& mesh, const std::vector<int>& triangles);
	
	ClosestTriangleToPoint ClosestPointOnMeshToPointAcc(Math::Vector3 p, const Mesh& mesh, const struct AabbTree* tree, 
		int seed = -1, bool useDeltaNormal = true);

	struct ClosestEdgeToSegment
	{
		Math::Vector3 closestPtMesh;
		Math::Vector3 closestPtSegment;
		Math::Vector3 normal;
		Math::Vector2 coordsMesh;
		Math::Vector2 coordsSegm;
		int edge1 = -1;
		int edge = -1;
		float distance = -FLT_MAX;
		int region = -1;
		float side = FLT_MAX;
	};

	Math::Vector3 ComputeEdgeNormal(const Mesh& mesh, int e, float param);
	
	float ClosestPointOnMeshToSegment(Math::Vector3 p, Math::Vector3 q, const Geometry::Mesh& mesh, ClosestEdgeToSegment& result, int e2 = -1);

	float ClosestPointOnMeshToSegment(Math::Vector3 p, Math::Vector3 q, const Geometry::Mesh& mesh, const std::vector<int>& edgeSet, ClosestEdgeToSegment& result);

	float ClosestPointOnMeshToSegmentAcc(Math::Vector3 p, Math::Vector3 q, const Mesh& mesh, const AabbTree* tree, bool useDelta, int seed, ClosestEdgeToSegment& result);

	struct ClosestVertexToTriangle
	{
		Math::Vector3 closestPtMesh;
		Math::Vector3 closestPtTri;
		Math::Vector3 normal;
		Math::Vector3 baryCoords;
		float distance = -FLT_MAX;
		int vertex = -1;
		int region;
	};

	// Returns the distance and closest point to a mesh from a triangle
	// The closest point is always located on a vertex
	float ClosestPointOnMeshToTriangle(Math::Vector3 a, Math::Vector3 b, Math::Vector3 c, const Mesh& mesh, ClosestVertexToTriangle& result);

	float ClosestPointOnMeshToTriangle(Math::Vector3 a, Math::Vector3 b, Math::Vector3 c, const Mesh& mesh, const std::vector<int>& vertexSet, ClosestVertexToTriangle& result);

	ClosestVertexToTriangle ClosestPointOnMeshToTriangleAcc(Math::Vector3 a, Math::Vector3 b, Math::Vector3 c, const Mesh& mesh, const AabbTree* node, int seed = -1);
}