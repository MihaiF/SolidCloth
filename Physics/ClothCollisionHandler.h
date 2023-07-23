#pragma once

#include <vector>
#include <set>

namespace Physics
{
	class ClothModel;

	class ClothCollisionHandler
	{
	public:
		ClothCollisionHandler() : mModel(nullptr) {}
		void SetClothModel(ClothModel* model) { mModel = model; }
		void DetectAndHandle(float h);

		void SolveContactsVelocity(float h);
		void SolveContactsPosition(float h);

		void CheckContactsPosition();

	private:
		bool HandleContactsVelocity(float h);
		void HandleContactsPosition(float h);

	private:
		ClothModel* mModel;

		std::vector<int> mSetAssociations;
		std::vector<std::set<int>> mSetList;

		// constants
		const float baumgarte = 0.2f;
		const float threshold = 0.0001f;
		const int numSelfCollIterations = 2;
		const int numVelocityIterations = 20;
		const int numPositionIterations = 0;
	};
}
