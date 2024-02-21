#ifndef CLOTH_H
#define CLOTH_H

#include "Math/Vector3.h"
#include "ClothCollisionHandler.h"

#include <memory>

namespace Geometry
{
	struct Mesh;
}

namespace Physics
{
	class ClothModel;
	class CollisionWorld;

	enum MethodType
	{
		METHOD_PBD,
	};

	class ClothPatch
	{
	public:
		ClothPatch(const CollisionWorld& world);
		ClothPatch& operator =(const ClothPatch& other);
		~ClothPatch();
		// init quad sim-mesh based on input args
		void Init(int divisionsX, int divisionsY, float inc, const Math::Vector3& offset, bool horizontal, bool attached);
		// init tri-sim-mesh
		void Init(const Geometry::Mesh& mesh, const Math::Vector3& offset, bool attached);
		bool Step(float dt);
		bool UpdateMesh();
		void ComputeMass();

		const Geometry::Mesh& GetMesh() const { return *mMesh; }
		Geometry::Mesh& GetMesh() { return *mMesh; }
		Geometry::Mesh& GetPrevMesh() { return *mPrevMesh; }
		const ClothModel& GetModel() const { return *mModel; }
		ClothModel& GetModel() { return *mModel; }
		float GetMaxMass() const { return mMaxMass; }
		void SetPosition(const Math::Vector3& v) { mPosition = v; }
		void SetMethod(MethodType val);
		MethodType GetMethod() const { return mMethod; }
		void SetNumSteps(int val) { mNumSteps = val; }
		bool IsQuadMesh() const { return mIsQuadMesh; }

		ClothCollisionHandler& GetCollisionHandler() { return mCollisionHandler; }

		const CollisionWorld& GetCollWorld() const { return  mCollWorld; }

	private:
		bool mIsQuadMesh;
		ClothModel* mModel;
		std::shared_ptr<Geometry::Mesh> mMesh;
		std::unique_ptr<Geometry::Mesh> mPrevMesh;
		float mMaxMass;
		Math::Vector3 mPosition;
		MethodType mMethod;
		int mNumSteps;

		ClothCollisionHandler mCollisionHandler;

		const CollisionWorld& mCollWorld;
	};
}

#endif // CLOTH_H