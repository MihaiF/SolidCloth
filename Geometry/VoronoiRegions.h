#pragma once

#include "Math/Vector3.h"

namespace Geometry
{
	class Mesh;

	bool TestPointInTriangleVoronoiRegion(Math::Vector3 p, const Mesh& mesh, int tri);

	bool TestPointInEdgeVoronoiRegion(Math::Vector3 p, const Mesh& mesh, int eID, float eps = 0.002f);

	bool TestPointInVertexVoronoiRegion(Math::Vector3 p, const Mesh& mesh, int vID);
}
