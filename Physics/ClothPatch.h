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

	enum MethodType
	{
		METHOD_PBD,
	};

	class ClothPatch
	{
	public:
		ClothPatch();
		ClothPatch& operator =(const ClothPatch& other);
		~ClothPatch();
		// init quad sim-mesh based on input args
		void Init(int divisionsX, int divisionsY, float inc, const Math::Vector3& offset, bool horizontal, bool attached);
		// init tri-sim-mesh
		void Init(const Geometry::Mesh& mesh, const Math::Vector3& offset, bool attached);
		void Step(float dt);
		void UpdateMesh();
		void ComputeMass();

		const Geometry::Mesh& GetMesh() const { return *mMesh; }
		const ClothModel& GetModel() const { return *mModel; }
		ClothModel& GetModel() { return *mModel; }
		float GetMaxMass() const { return mMaxMass; }
		void SetPosition(const Math::Vector3& v) { mPosition = v; }
		void SetMethod(MethodType val);
		MethodType GetMethod() const { return mMethod; }
		void SetNumSteps(int val) { mNumSteps = val; }

		ClothCollisionHandler& GetCollisionHandler() { return mCollisionHandler; }

	private:
		bool isQuadMesh;
		ClothModel* mModel;
		std::shared_ptr<Geometry::Mesh> mMesh;
		float mMaxMass;
		Math::Vector3 mPosition;
		MethodType mMethod;
		int mNumSteps;

		ClothCollisionHandler mCollisionHandler;
	};
}

#endif // CLOTH_H