#pragma once

#include <Math/Vector3.h>
#include <vector>

namespace Geometry
{
	struct AabbTree;
}

namespace Physics
{
	struct Triangle;
	struct Particle;
	struct PrimitivePair;
	struct SelfContact;
	class ClothModel;

	class SelfCollisionsDetector
	{
	public:
		SelfCollisionsDetector() : mModel(nullptr) { }
		void Detect();
		void SetClothModel(ClothModel* model) { mModel = model; }

	private:
		void Midphase();
		void Narrowphase();
		void EdgeEdgeDetection();

		typedef float (SelfCollisionsDetector::*CCDFunction)(float t, const Math::Vector3& x1, const Math::Vector3& x2, const Math::Vector3& x3, const Math::Vector3& x4,
			const Math::Vector3& v1, const Math::Vector3& v2, const Math::Vector3& v3, const Math::Vector3& v4);

		void VertexTriangleTest(const Math::Vector3& x1, const Math::Vector3& x2, const Math::Vector3& x3, const Math::Vector3& x4,
			const Math::Vector3& v1, const Math::Vector3& v2, const Math::Vector3& v3, const Math::Vector3& v4,
			int i1, int i2, int i3, int i4);
		bool VertexTriangleCCD(const Math::Vector3& x1, const Math::Vector3& x2, const Math::Vector3& x3, const Math::Vector3& x4,
			const Math::Vector3& v1, const Math::Vector3& v2, const Math::Vector3& v3, const Math::Vector3& v4,
			int i1, int i2, int i3, int i4, Math::Vector3& normal, Math::BarycentricCoords& barycentric);

		float EvaluateFun_Coplanarity(float t, const Math::Vector3& x1, const Math::Vector3& x2, const Math::Vector3& x3, const Math::Vector3& x4,
			const Math::Vector3& v1, const Math::Vector3& v2, const Math::Vector3& v3, const Math::Vector3& v4);

		bool SolveCubic(const Math::Vector3& x1, const Math::Vector3& x2, const Math::Vector3& x3, const Math::Vector3& x4,
			const Math::Vector3& v1, const Math::Vector3& v2, const Math::Vector3& v3, const Math::Vector3& v4, float& toi);
		bool CheckDegenerateCase(float b, float c, float d, float& toi);
		int FindInflexionPoints(float a, float b, float c, float& tInfl1, float& tInfl2);
		bool CheckInterval(CCDFunction fun, float ta, float tb, const Math::Vector3& x1, const Math::Vector3& x2, const Math::Vector3& x3, const Math::Vector3& x4,
			const Math::Vector3& v1, const Math::Vector3& v2, const Math::Vector3& v3, const Math::Vector3& v4, float& tx);

		void EdgeEdgeTest(const Math::Vector3& x1, const Math::Vector3& x2, const Math::Vector3& x3, const Math::Vector3& x4,
			const Math::Vector3& v1, const Math::Vector3& v2, const Math::Vector3& v3, const Math::Vector3& v4,
			int i1, int i2, int i3, int i4);
		bool EdgeEdgeCCD(const Math::Vector3& x1, const Math::Vector3& x2, const Math::Vector3& x3, const Math::Vector3& x4,
			const Math::Vector3& v1, const Math::Vector3& v2, const Math::Vector3& v3, const Math::Vector3& v4,
			int i1, int i2, int i3, int i4, Math::Vector3& normal, float& s, float& t);

	private:
		ClothModel* mModel;

		// keep these here so it doesn't keep reallocating
		std::vector<PrimitivePair> mVertexTriangleCandidates;
		std::vector<PrimitivePair> mEdgeEdgeCandidates;
	};
}

