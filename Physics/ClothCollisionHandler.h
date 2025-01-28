#pragma once

#include <vector>
#include <set>

namespace Physics
{
	class ClothModel;
	class CollisionWorld;

	struct ContactStats
	{
		int count = 0;
		float error = 0;
	};

	class ClothCollisionHandler
	{
	public:
		ClothCollisionHandler(const CollisionWorld& world);
		void SetClothModel(ClothModel* model); // TODO: should be part of constructor
		void DetectAndHandle(float h);
		void Update();

		void SolveContactsVelocity(float h);
		float SolveContactsPosition(float h);
		float SolveEdgeContacts();
		float SolveTriContacts();
		float SolveSelfTrisPosition(float h);
		ContactStats SolveSelfEdgesPosition(float h);

		void CheckContactsPosition();

	private:
		bool HandleContactsVelocity(float h);
		void HandleContactsPosition(float h);

	private:
		ClothModel* mModel;
		const CollisionWorld& mCollWorld;

		std::vector<int> mSetAssociations;
		std::vector<std::set<int>> mSetList;

		// constants
		const float baumgarte = 0.2f;
		const float threshold = 0.0001f;
		const int numVelocityIterations = 20;
		const int numPositionIterations = 0;

		// TODO: supplant by contact list
		std::vector<float> mVertexLambda;
		std::vector<float> mVertexLambdaF;
		std::vector<float> mEdgeLambda;
		std::vector<float> mEdgeLambdaF;
		std::vector<float> mTriLambda;
		std::vector<float> mTriLambdaF;
	};
}
