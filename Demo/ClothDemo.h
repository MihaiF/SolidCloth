#ifndef CLOTH_DEMO_H
#define CLOTH_DEMO_H

#include "Physics/ClothPatch.h"
#include "Geometry/SDF.h"
#include "Physics/Common.h"
#include "Physics/CollisionWorld.h"

enum DebugDrawFlags
{
	DDF_CONTACTS = 1,
	DDF_WARP_WEFT = 2,
	DDF_PARTICLES = 4,
	DDF_STRAIN = 8,
	DDF_LINKS = 16,
	DDF_TREE = 128,
	DDF_TRI_CONTACTS = 256,
	DDF_TREE_SELF = 512,
};

class Graphics3D;

// 3D cloth demo
class ClothDemo
{
public:
	ClothDemo();
	void Create(int type);
	void Init();
	void Draw(Graphics3D* graphics3D, bool showDebug, int debugDrawFlags);
	void DrawUI();
	void Update(float dt);
	void OnMouseMove(int x, int y);
	void OnMouseDown(int x, int y);
	void OnMouseUp(int x, int y);

private:
	void InitCloth(Physics::ClothPatch& cloth, Math::Vector3 offset);
	void SetupCloth(Physics::ClothPatch& cloth);

public:
	enum DemoType
	{
		CLOTH_DEMO_DEFAULT,
		CLOTH_DEMO_SPHERE,
		CLOTH_DEMO_CAPSULE,
		CLOTH_DEMO_MESH,
		CLOTH_DEMO_SDF,
	};
	
	enum AvatarAsset
	{
		AVATAR_ASSET_NONE,
		AVATAR_ASSET_BUNNY,
		AVATAR_ASSET_BUDDHA,
		AVATAR_ASSET_DRAGON,
		AVATAR_ASSET_ARMADILLO,
		AVATAR_ASSET_TEAPOT,
		AVATAR_ASSET_SPHERE,
	};
	

private:
	Physics::CollisionWorld mCollWorld;
	Physics::ClothPatch mCloth;
	
	int mDivisions;
	bool mHorizontal;
	bool mAttached;
	int mDemoType;
	int mAvatarAsset;
	Geometry::Mesh mMesh;
	Geometry::Mesh mMeshCopy;
	std::shared_ptr<Physics::CollisionMesh> mCollisionMesh;
	
	// mouse picking
	Vector3 mMouse;
	Vector3 mPick;
	int mViewWidth;
	int mViewHeight;
	float mViewFOV;
	int mSelected;
	int mSelTri[3];
	Math::BarycentricCoords mSelCoords;

	std::unique_ptr<class Texture> mCheckerTexture;

	bool mWireframeCloth = false;

	Graphics3D* mGraphics3D = nullptr;

	int mSelectedTriangle = -1;
	Vector3 mEye;
	int mClothDraw1 = -1;
	int mClothDraw2 = -1;

	Geometry::SDF mSDF;

	int mFrame = 0;
};

#endif // CLOTH_DEMO_H
