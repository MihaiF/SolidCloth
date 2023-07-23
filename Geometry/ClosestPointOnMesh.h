#include <Math/Vector3.h>
#include <Geometry/Mesh.h>

namespace Geometry
{
	int ClosestVertexOnMeshToPoint(const Math::Vector3& p, const Mesh& mesh);

	float ClosestPointOnMeshToPoint(const Math::Vector3& p, const Mesh& mesh, Math::Vector3& closestPt, Math::Vector3& normal, BarycentricCoords& coords);
	
	float ClosestPointOnMeshToPointAcc(const Math::Vector3& p, const Mesh& mesh, const struct AabbTree* tree, Math::Vector3& closestPt, Math::Vector3& normal);

	// TODO: ClosestPointOnMeshToSegment(const Math::Vector3& p, const Math::Vector3& q, const Geometry::Mesh& mesh)

	struct ClosestMeshTriangle
	{
		Math::Vector3 closestPtMesh;
		Math::Vector3 closestPtTri;
		Math::Vector3 normal;
		Math::Vector3 baryCoords;
		int vertex;
		float distance;
	};

	// Returns the distance and closest point to a mesh from a triangle
	// The closest point is always located on a vertex
	float ClosestPointOnMeshToTriangle(Math::Vector3 a, Math::Vector3 b, Math::Vector3 c, const Mesh& mesh, ClosestMeshTriangle& result);

	float ClosestPointOnMeshToTriangleOMP(Math::Vector3 a, Math::Vector3 b, Math::Vector3 c, const Mesh& mesh, ClosestMeshTriangle& result);
}