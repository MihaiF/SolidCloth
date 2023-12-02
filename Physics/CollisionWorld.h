#pragma once

#include <vector>
#include <memory>

#include "Physics/Common.h"

namespace Physics
{
	class CollisionWorld
	{
	public:
		void ClearCollidables()
		{
			mCollidables.clear();
		}

		void AddCollidable(std::shared_ptr<Physics::Collidable>& coll)
		{
			mCollidables.push_back(coll);
		}

		size_t GetNumCollidables() const { return mCollidables.size(); }
		const Collidable* GetCollidable(size_t i) const { return mCollidables[i].get(); }
		Collidable* GetCollidable(size_t i) { return mCollidables[i].get(); }

	private:
		std::vector<std::shared_ptr<Collidable> > mCollidables;
	};

}
