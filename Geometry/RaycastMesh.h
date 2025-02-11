#pragma once

#include "Mesh.h"

namespace Geometry
{
	struct RaycastInfo
	{
		Math::Vector3 bary;
		Math::Vector3 pt;
		Math::Vector3 normal;
		float t;
		int tri = -1;
	};

	bool RaycastMesh(Math::Vector3 p, Math::Vector3 q, const Mesh& mesh, RaycastInfo& info, bool takeFirst, 
		Math::Matrix4* transform = nullptr, bool isRay = true);
}
