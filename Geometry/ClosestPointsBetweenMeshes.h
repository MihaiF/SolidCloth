#pragma once

#include "ClosestPointOnMesh.h"

namespace Geometry
{
	class MeshClosestPoints
	{
	public:
		// also duplicated
		struct PrimitivePair
		{
			int idx1, idx2;
		};

	private:
		int mCollFlags;

		// TODO: rename
		std::vector<PrimitivePair> mPotentialContacts; // VT pairs
		std::vector<PrimitivePair> mPotentialTriContacts; // TV pairs
		std::vector<PrimitivePair> mPotentialEdgeContacts; // EE pairs
		std::vector<PrimitivePair> mTriTriCandidates; // TT pairs

		std::vector<std::vector<int>> mSetVT;
		std::vector<std::vector<int>> mSetTV;
		std::vector<std::vector<int>> mSetEE;

	public:
		std::vector<ClosestTriangleToPoint> mVertexInfos;
		std::vector<ClosestVertexToTriangle> mTriangleInfos;
		std::vector<ClosestEdgeToSegment> mEdgeInfos;

	private:
		void TestTrees(const Mesh& mesh1, const Mesh& mesh2, AabbTree* node1, AabbTree* node2, float radTol);
		void VertexVsTriangle(const Mesh* mesh1, const Mesh* mesh2, int vertexIndex, int triangleIndex);
		void TriangleVsVertex(const Mesh* mesh1, const Mesh* mesh2, int idx, int tri);
		void EdgeVsEdge(const Mesh* mesh1, const Mesh* mesh2, int e1, int e2);

		void HandleCandidatePairs(const Mesh& mesh1, const Mesh& mesh2);
		void HandleCandidatePairsSDF(const Mesh& mesh1, const Mesh& mesh2);

		void HandleTriTriPairs(const Mesh& mesh1, const Mesh& mesh2);

	public:
		MeshClosestPoints() : mCollFlags(0) { }
		
		void SetParams(int collFlags)
		{
			mCollFlags = collFlags;
		}

		void ClosestPoints(const Mesh& mesh1, const Mesh& mesh2, AabbTree* root1, AabbTree* root2, float radTol, bool useSDF);
		
		void ClosestPoints(const Mesh& mesh1, const Mesh& mesh2, const AabbTree* tree, float radTol, bool sign);

		void ClosestPoints(const Mesh& mesh, const AabbTree* tree, float radTol);

		void Clear()
		{
			mPotentialContacts.clear();
			mPotentialTriContacts.clear();
			mPotentialEdgeContacts.clear();
			mTriTriCandidates.clear();

			mVertexInfos.clear();
			mTriangleInfos.clear();
			mEdgeInfos.clear();
		}

	}; // class MeshClosestPoints
}
