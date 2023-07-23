#ifndef AABB_TREE_H
#define AABB_TREE_H

#include "Geometry/Aabb3.h"
#include "Engine/Types.h"

#include <vector>

namespace Geometry
{
	struct Mesh;

	struct BoxInfo
	{
		Geometry::Aabb3 box;
		uint32 idx; // associated primitive index
	};

	struct AabbTree
	{
		Geometry::Aabb3 box;
		std::vector<BoxInfo> triangles;
		std::vector<BoxInfo> vertices;
		std::vector<BoxInfo> edges;
		AabbTree *left, *right;
		AabbTree() : left(nullptr), right(nullptr) { }
		~AabbTree()
		{
			if (left) {
				delete left; 
				left = nullptr;
			}
			if (right) {
				delete right; 
				right = nullptr;
			}
		}
	};

	enum AabbTreeFlags
	{
		ATF_VERTICES = 1,
		ATF_TRIANGLES = 2,
		ATF_EDGES = 4,
	};
	
	AabbTree* ComputeMeshTree(const Geometry::Mesh& mesh, int flags, int maxLevel, float tol);
}

#endif
