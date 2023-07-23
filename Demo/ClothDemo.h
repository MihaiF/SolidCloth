#ifndef CLOTH_DEMO_H
#define CLOTH_DEMO_H

#include "Physics/ClothPatch.h"
#include "Graphics3D/Camera.h"
#include "Geometry/Mesh.h"
#include "Geometry/SDF.h"
#include "Physics/Common.h"

enum DebugDrawFlags
{
	DDF_CONTACTS = 1,
	DDF_WARP_WEFT = 2,
	DDF_PARTICLES = 4,
	DDF_STRAIN = 8,
	DDF_LINKS = 16,
	//DDF_SELF_TRIS = 32,
	//DDF_SELF_EDGES = 64,
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
	bool LoadFromXml(XMLElement* xCloth);
	void Init();
	void Draw(Graphics3D* graphics3D, bool showDebug, int debugDrawFlags);
	void DrawUI();
	void Update(float dt);
	void OnMouseMove(int x, int y);
	void OnMouseDown(int x, int y);
	void OnMouseUp(int x, int y);

public:
	enum DemoType
	{
		CLOTH_DEMO_DEFAULT,
		CLOTH_DEMO_SPHERE,
		CLOTH_DEMO_CAPSULE,
		CLOTH_DEMO_MESH,
		CLOTH_DEMO_SDF,
	};

	enum ClothAsset
	{
		CLOTH_ASSET_DEFAULT,
		CLOTH_ASSET_PATCH,
		CLOTH_ASSET_KATJA,
		CLOTH_ASSET_MD,
		CLOTH_ASSET_BUNNY, 
		CLOTH_ASSET_BUDDHA,
		CLOTH_ASSET_DRAGON,
		CLOTH_ASSET_SPHERE,
	};

private:
	Physics::ClothPatch mCloth;
	int mDivisions;
	bool mHorizontal;
	bool mAttached;
	int mDemoType;
	int mClothAsset;
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
};

#endif // CLOTH_DEMO_H
