#include "AabbTree.h"
#include "Geometry/Mesh.h"
#include <Engine/Profiler.h>

using namespace Math;

namespace Geometry
{
	void SplitNode(AabbTree* root, const Geometry::Mesh& mesh, int level, int maxLevel, int flags, float tol)
	{
		const int maxPrims = 100;
		if (root->triangles.size() < maxPrims 
			&& root->vertices.size() < maxPrims
			&& root->edges.size() < maxPrims)
			return;

		// choose longest extent and splitting plane
		Vector3 ext = root->box.GetExtent();
		int dir = ext.GetMaxComponent();
		float center = 0.5f * (root->box.min[dir] + root->box.max[dir]);

		root->left = new AabbTree;
		root->right = new AabbTree;

		if (flags & ATF_TRIANGLES)
		{
			for (size_t i = 0; i < root->triangles.size(); i++)
			{
				const BoxInfo& info = root->triangles[i];
				int tri = info.idx;
				// TODO: pre-compute box
				int idx = tri * 3;
				int i1 = mesh.indices[idx];
				int i2 = mesh.indices[idx + 1];
				int i3 = mesh.indices[idx + 2];
				Vector3 v1 = mesh.vertices[i1];
				Vector3 v2 = mesh.vertices[i2];
				Vector3 v3 = mesh.vertices[i3];

				Geometry::Aabb3 box;
				box.Add(v1);
				box.Add(v2);
				box.Add(v3);
				box.Extrude(tol);

				// TODO: use center
				if (box.min[dir] <= center)
				{
					root->left->triangles.push_back(info);
					root->left->box.Add(box);
				}
				else
				{
					root->right->triangles.push_back(info);
					root->right->box.Add(box);
				}
			}
			root->triangles.clear();
		}

		if ((flags & ATF_EDGES) != 0)
		{
			for (size_t i = 0; i < root->edges.size(); i++)
			{
				const BoxInfo& info = root->edges[i];
				int idx = info.idx;
				int i1 = mesh.edges[idx].i1;
				int i2 = mesh.edges[idx].i2;
				Vector3 v1 = mesh.vertices[i1];
				Vector3 v2 = mesh.vertices[i2];

				Aabb3 box;
				box.Add(v1);
				box.Add(v2);
				box.Extrude(tol);

				// TODO: use center
				if (box.min[dir] <= center)
				{
					root->left->edges.push_back(info);
					root->left->box.Add(box);
				}
				else
				{
					root->right->edges.push_back(info);
					root->right->box.Add(box);
				}
			}
			root->edges.clear();
		}

		if (flags & ATF_VERTICES)
		{
			for (size_t i = 0; i < root->vertices.size(); i++)
			{
				const BoxInfo& info = root->vertices[i];
				int idx = info.idx;
				// TODO: pre-compute box
				Geometry::Aabb3 ptBox;
				ptBox.Add(mesh.vertices[idx]);
				ptBox.Extrude(tol);
				if (mesh.vertices[idx][dir] <= center)
				{
					root->left->vertices.push_back(info);
					root->left->box.Add(ptBox);
				}
				else
				{
					root->right->vertices.push_back(info);
					root->right->box.Add(ptBox);
				}
			}
			root->vertices.clear();
		}

		if (root->left->triangles.empty() && root->left->vertices.empty() && root->left->edges.empty())
		{
			delete root->left;
			root->left = NULL;
		}
		if (root->right->triangles.empty() && root->right->vertices.empty() && root->right->edges.empty())
		{
			delete root->right;
			root->right = NULL;
		}

		if (level < maxLevel)
		{
			if (root->left)
				SplitNode(root->left, mesh, level + 1, maxLevel, flags, tol);
			if (root->right)
				SplitNode(root->right, mesh, level + 1, maxLevel, flags, tol);
		}
	}

	AabbTree* ComputeMeshTree(const Geometry::Mesh& mesh, int flags, int maxLevel, float tol)
	{
		//PROFILE_SCOPE("ComputeMeshTree");
		Geometry::Aabb3 bounds;
		for (size_t i = 0; i < mesh.vertices.size(); i++)
		{
			bounds.min = vmin(bounds.min, mesh.vertices[i]);
			bounds.max = vmax(bounds.max, mesh.vertices[i]);
		}
		AabbTree* root = new AabbTree;
		root->box = bounds;
		if (flags & ATF_TRIANGLES)
		{
			root->triangles.resize(mesh.indices.size() / 3);
			for (size_t i = 0; i < root->triangles.size(); i++)
			{
				root->triangles[i].idx = (uint32)i;
			}
		}
		if (flags & ATF_VERTICES)
		{
			root->vertices.resize(mesh.vertices.size());
			for (size_t i = 0; i < mesh.vertices.size(); i++)
			{
				root->vertices[i].idx = (uint32)i;
			}
		}
		if (flags & ATF_EDGES)
		{
			root->edges.resize(mesh.edges.size());
			for (size_t i = 0; i < mesh.edges.size(); i++)
			{
				root->edges[i].idx = (uint32)i;
			}
		}

		if (maxLevel <= 0)
			return root;

		SplitNode(root, mesh, 1, maxLevel, flags, tol);

		return root;
	}

}