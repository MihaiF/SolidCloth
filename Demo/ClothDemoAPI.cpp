#include "ClothDemoAPI.h"
#include "Geometry/Collision3D.h"
#include "Geometry/Assets.h"
#include "Geometry/AabbTree.h"
#include "Geometry/MarchingCubes.h"
#include "Graphics3D/Graphics3D.h"
#include "Graphics2D/Graphics2D.h"
#include "Math/Utils.h"

#include <imgui.h>

using namespace Geometry;
using namespace Math;

ClothDemoAPI::ClothDemoAPI()
	: mDemoType(CLOTH_DEMO_DEFAULT)
	, mAvatarAsset(AVATAR_ASSET_BUDDHA)
	, mHorizontal(false)
	, mAttached(false)
	, mSelected(-1)
	, mDivisions(30)
{
	mCollWorld = scCreateCollisionWorld();
	mCloth = scCreateClothPatch(mCollWorld);

	mCheckerTexture.reset(new Texture());
}

ClothDemoAPI::~ClothDemoAPI()
{
	scFreeCollisionWorld(mCollWorld);
	scFreeClothPatch(mCloth);
}

void ClothDemoAPI::Create(int type)
{
	if (!mCheckerTexture->LoadTexture("../Res/checker.bmp"))
		Printf("Failed to load texture\n");

	mDemoType = type;
	scModel model = scGetClothModel(mCloth);
	scSetCollisionFlags(model, scCollisionFlagWalls | scCollisionFlagVertices | scCollisionFlagTriangles | scCollisionFlagEdges | scCollisionFlagSelf);
}

void ClothDemoAPI::Init()
{
	mFrame = 0;

	Aabb3 walls(Vector3(-60), Vector3(60));
	scClearCollidables(mCollWorld);
	scHandle wallsColl = scCreateWalls(walls);
	scAddCollidable(mCollWorld, wallsColl);

	if (mDemoType == CLOTH_DEMO_SPHERE)
	{
		scHandle sph = scCreateSphere(Vector3(0, -10, 0), 25);
		scAddCollidable(mCollWorld, sph);
	}
	else if (mDemoType == CLOTH_DEMO_CAPSULE)
	{
		Quaternion q;
		q.SetAxisAngle(1.3f, Vector3(1, 0.5f, 0));
		scHandle cap = scCreateCapsule(Vector3(0, -10, 0), 16, 15, q);
		scAddCollidable(mCollWorld, cap);
	}
	else if (mDemoType == CLOTH_DEMO_SDF)
	{
		mSDF.LoadFromFile("../Models/bunny.sdf");
		mSDF.ComputeGradient();

		scHandle sdf = scCreateSDF(&mSDF);

		auto eval = [&](int x, int y, int z) { return mSDF.GetValue(x, y, z); };
		auto grad = [&](const Vector3& p) { return mSDF.GetGrad(p); };
		int precision[3];
		mSDF.GetNumCellsPerSide(precision);
		mIsoMesh.Clear();
		CreateIsoSurface(mIsoMesh, eval, grad, mSDF.GetBox(), precision);
		scAddCollidable(mCollWorld, sdf);
	}
	else if (mDemoType == CLOTH_DEMO_MESH)
	{
		mMesh.Clear();
		bool ret = false;
		if (mAvatarAsset == AVATAR_ASSET_BUDDHA)
		{
			ret = LoadMesh("../Models/buddha.obj", mMesh, Vector3(0, -5, 0), 2.f);
		}
		else if (mAvatarAsset == AVATAR_ASSET_DRAGON)
		{
			ret = LoadMesh("../Models/dragon.obj", mMesh, Vector3(0, -80, 0), 5.f);
		}
		else if (mAvatarAsset == AVATAR_ASSET_ARMADILLO)
		{
			ret = LoadMesh("../Models/armadillo.obj", mMesh, Vector3(0, 0, 0), 50.f);
		}
		else if (mAvatarAsset == AVATAR_ASSET_TEAPOT)
		{
			ret = LoadMesh("../Models/teapot.obj", mMesh, Vector3(0, -10, 0), 10.f);
		}
		else if (mAvatarAsset == AVATAR_ASSET_SPHERE)
		{
			ret = LoadMesh("../Models/sphere.obj", mMesh, Vector3(0, -30, 0), 20.f);
		}
		else // default
		{
			ret = LoadMesh("../Models/bunny.obj", mMesh, Vector3(0, -25, 0), 2.f);
		}

		if (ret)
		{
			mMesh.ComputeNormals();
			mMesh.ConstructEdges();
			mMesh.ConstructEdgeOneRings();
			scHandle collMesh = scCreateMesh(&mMesh);
			scAddCollidable(mCollWorld, collMesh);
		}
		else
			Printf("Could not load mesh\n");
	}

	//SetupCloth(mCloth);
	//InitCloth(mCloth, Vector3(0, 60, 10));
	
	mHorizontal = true;
	mAttached = false;

	if (mDemoType == CLOTH_DEMO_DEFAULT)
	{
		mHorizontal = false;
		mAttached = true;
		scSetCollisionFlags(scGetClothModel(mCloth), 0);
	}

	scInitCloth(mCloth, mDivisions, mDivisions, 80.f / mDivisions, Vector3(0, 60, 10), mHorizontal, mAttached);
}

void DrawTree(Graphics3D* graphics3D, AabbTree* tree)
{
	if (!tree)
		return;
	if (!tree->triangles.empty() || !tree->vertices.empty())
		graphics3D->DrawWireCube(tree->box.GetCenter(), tree->box.GetExtent(), Matrix3::Identity());
	if (tree->left)
		DrawTree(graphics3D, tree->left);
	if (tree->right)
		DrawTree(graphics3D, tree->right);
}

void ClothDemoAPI::DrawUI()
{
#ifdef USE_IMGUI
	if (ImGui::CollapsingHeader("Cloth"))
	{
		ImGui::Checkbox("Wireframe on shaded", &mWireframeCloth);
		ImGui::Combo("Avatar asset", &mAvatarAsset, "None\0Bunny\0Buddha\0Dragon\0Armadillo\0Teapot\0Sphere\0");
		ImGui::InputInt("Divisions", &mDivisions);

		scModel model = scGetClothModel(mCloth);		
		float thickness = scGetThickness(model);
		if (ImGui::InputFloat("Thickness", &thickness))
			scSetThickness(model, thickness);

		float tolerance = scGetTolerance(model);
		if (ImGui::InputFloat("Tolerance", &tolerance))
			scSetTolerance(model, tolerance);

		float friction = scGetFriction(model);
		if (ImGui::InputFloat("Friction", &friction))
			scSetFriction(model, friction);

		ImGui::Checkbox("Horizontal", &mHorizontal);
		ImGui::Checkbox("Attached", &mAttached);
		
		int iters = scGetNumIterations(model);
		if (ImGui::InputInt("Iterations", &iters))
			scSetNumIterations(model, iters);

		bool useFem = scGetUseFEM(model);
		if (ImGui::Checkbox("Use FEM", &useFem))
			scSetUseFEM(model, useFem);
		
		int flags = scGetCollisionFlags(model);		
		
		bool collVertices = flags & CF_VERTICES;
		if (ImGui::Checkbox("Vertex collisions", &collVertices))
		{
			if (collVertices)
				scSetCollisionFlags(model, flags | CF_VERTICES);
			else
				scSetCollisionFlags(model, flags & ~CF_VERTICES);
		}

		bool collEdges = flags & CF_EDGES;
		if (ImGui::Checkbox("Edge collisions", &collEdges))
		{
			if (collEdges)
				scSetCollisionFlags(model, flags | CF_EDGES);
			else
				scSetCollisionFlags(model, flags & ~CF_EDGES);
		}

		bool collTriangles = flags & CF_TRIANGLES;
		if (ImGui::Checkbox("Triangle collisions", &collTriangles))
		{
			if (collTriangles)
				scSetCollisionFlags(model, flags | CF_TRIANGLES);
			else
				scSetCollisionFlags(model, flags & ~CF_TRIANGLES);
		}
	}
#endif
}

void ClothDemoAPI::Draw(Graphics3D* graphics3D, bool showDebug, int debugDrawFlags)
{
	mGraphics3D = graphics3D; // FIXME

	// store viewport properties for mouse picking
	mViewWidth = graphics3D->w;
	mViewHeight = graphics3D->h;
	mViewFOV = graphics3D->fov;

	scModel model = scGetClothModel(mCloth);
	std::vector<Vector3> colors(scGetNumParticles(model));
	int flags = 0;
	scUpdateClothMesh(mCloth);

	if (showDebug)
	{
		if (debugDrawFlags & DDF_PARTICLES)
		{
			for (size_t i = 0; i < scGetNumParticles(model); i++)
			{
				Vector3 pos;
				scGetParticlePos(model, i, pos);
				float im = scGetParticleInvMass(model, i);
				float m = im != 0 ? 1.f : 0.1f;
				graphics3D->SetColor(m * 0.83f, m * 0.67f, 0.f);
				graphics3D->DrawSphere(pos, scGetThickness(model));
			}
		}
		if (debugDrawFlags & DDF_LINKS)
		{
			graphics3D->SetColor(0.8f, 0.3f, 0.2f);
			int i1, i2;
			for (size_t i = 0; i < scGetNumLinks(model); i++)
			{
				scGetLink(model, i, i1, i2);
				Vector3 p1, p2;
				scGetParticlePos(model, i1, p1);
				scGetParticlePos(model, i2, p2);
				graphics3D->DrawLine(p1, p2);
			}
		}
		if (debugDrawFlags & DDF_CONTACTS)
		{
			for (size_t i = 0; i < scGetNumContacts(model); i++)
			{
				Vector3 point, normal;
				scGetContact(model, i, point, normal);
				graphics3D->SetColor(0, 0, 1);
				graphics3D->DrawLine(point, point + normal * 3.f);
			}
		}
		if (debugDrawFlags & DDF_TRI_CONTACTS)
		{
			for (size_t i = 0; i < scGetNumTriContacts(model); i++)
			{
				Vector3 point, normal;
				scGetTriContact(model, i, point, normal);
				graphics3D->SetColor(0, 1, 0);
				graphics3D->DrawLine(point, point + normal * 3.f);
			}
		}
		if (debugDrawFlags & DDF_WARP_WEFT)
		{
			for (size_t i = 0; i < scGetNumTris(model); i++)
			{
				int i1, i2, i3;
				scGetTriangle(model, i, i1, i2, i3);
				Vector3 p1, p2, p3;
				scGetParticlePos(model, i1, p1);
				scGetParticlePos(model, i2, p2);
				scGetParticlePos(model, i3, p3);
				Vector3 dx1 = p2 - p1;
				Vector3 dx2 = p3 - p1;

				float invDet = 0;
				Vector2 du, dv, s;
				scGetTriangleInfo(model, i, invDet, du, dv, s);
				Vector3 wu = invDet * (dv.y * dx1 - dv.x * dx2);
				Vector3 wv = invDet * (-du.y * dx1 + du.x * dx2);
				Vector3 c = (1.f / 3.f) * (p1 + p2 + p3);
				graphics3D->SetColor(1, 0, 0);
				graphics3D->DrawLine(c, c + wu * s.x);
				graphics3D->SetColor(0, 1, 0);
				graphics3D->DrawLine(c, c + wv * s.y);
			}
		}
		if (debugDrawFlags & DDF_STRAIN)
		{
			flags |= ShaderFlags::VERTEX_COLORS;
			scComputeStrainMap(mCloth, colors);
		}
		if (debugDrawFlags & DDF_TREE_SELF)
			DrawTree(graphics3D, scGetClothTree(model));
	}

	// draw cloth mesh
	{
		scMesh mesh = scGetClothMesh(mCloth);

		int nv = scGetMeshVertexCount(mesh);
		int ni = scGetMeshIndexCount(mesh);
		Vector3* vb = scGetMeshVertexBuffer(mesh);
		std::vector<Vector3> vertices(vb, vb + nv);
		Vector3* nb = scGetMeshNormalBuffer(mesh);
		std::vector<Vector3> normals(nb, nb + nv);
		uint32* ib = scGetMeshIndexBuffer(mesh);
		std::vector<uint32> indices(ib, ib + ni);

		graphics3D->SetFlags(flags);
		if (mWireframeCloth)
			graphics3D->SetRenderMode(RM_WIREFRAME_ON_SHADED);
		graphics3D->SetColor(0, 1, 1);
		int draw1 = graphics3D->DrawMesh(vertices, normals, colors, indices);
		if (draw1 > 0)
			mClothDraw1 = draw1;
		graphics3D->SetCulling(Graphics3D::CULL_FRONT);
		graphics3D->SetColor(0, 0, 1);
		graphics3D->SetFlipNormals(true);
		int draw2 = graphics3D->DrawMesh(vertices, normals, colors, indices);
		if (draw2 > 0)
			mClothDraw2 = draw2;
		graphics3D->SetFlipNormals(false);
		graphics3D->SetCulling(Graphics3D::CULL_BACK);
		graphics3D->SetRenderMode(RM_SHADED);
	}

	// draw collidables
	for (size_t i = 0; i < scGetNumCollidables(mCollWorld); i++)
	{
		scHandle coll = scGetCollidable(mCollWorld, i);
		int type = scGetCollidableType(coll);
		if (type == scCollidableTypeMesh)
		{
			graphics3D->SetColor(0.82f, 0.94f, 0.8f);
			const Mesh& mesh = mMesh;
			graphics3D->DrawMesh(mesh.vertices, mesh.normals, mesh.indices);
			if (showDebug && (debugDrawFlags & DDF_TREE))
				DrawTree(graphics3D, scGetMeshTree(coll));
		}
		if (type == scCollidableTypeSDF)
		{
			graphics3D->SetColor(0.82f, 0.94f, 0.8f);
			const Mesh& mesh = mIsoMesh;
			graphics3D->DrawMesh(mesh.vertices, mesh.normals, mesh.indices);
		}
		else if (type == scCollidableTypeSphere)
		{
			float radius;
			Vector3 center;
			scGetSphereInfo(coll, radius, center);
			graphics3D->DrawSphere(center, radius);
		}
		else if (type == scCollidableTypeCapsule)
		{			
			float radius, halfHeight;
			Vector3 center;
			Quaternion rot;
			scGetCapsuleInfo(coll, radius, halfHeight, center, rot);
			graphics3D->SetColor(0.82f, 0.74f, 0.8f);
			graphics3D->DrawCapsule(center, radius, halfHeight, rot.ToMatrix());
		}
		else if (type == scCollidableTypeWalls)
		{
			graphics3D->SetColor(0.f, 0.f, 0.f);
			Aabb3 box;
			scGetWallsInfo(coll, box);
			Vector3 c = box.GetCenter();
			Vector3 e = box.GetExtent();
			graphics3D->DrawWireCube(c, e, Matrix3::Identity());
			graphics3D->SetColor(1.f, 1.f, 1.f);
			graphics3D->DrawPlane(c - Vector3(0, 0.5f * e.y, 0), e.x, mCheckerTexture.get());
		}
	}

	// draw mouse spring
	if (mSelected >= 0)
	{
		graphics3D->SetColor(0, 0, 1);
		graphics3D->DrawSphere(mPick, 0.7f);
		graphics3D->SetColor(1, 0, 0);
		Vector3 p1, p2, p3;
		scGetParticlePos(model, mSelTri[0], p1);
		scGetParticlePos(model, mSelTri[1], p2);
		scGetParticlePos(model, mSelTri[2], p3);
		Vector3 p =  p1 * mSelCoords.x +
			p2 * mSelCoords.y +
			p3 * mSelCoords.z;
		graphics3D->DrawSphere(p, 0.7f);
		graphics3D->DrawLine(mPick, p);
	}
}

void ClothDemoAPI::Update(float dt)
{
	scStepCloth(mCloth, dt);
}

void ClothDemoAPI::OnMouseMove(int x, int y)
{
	if (mSelected < 0)
		return;
	Vector3 mouseOld = mMouse;
	mMouse = mGraphics3D->ComputeMousePoint(x, y);
	mEye = mGraphics3D->camera.GetPosition();
	Vector3 n = mGraphics3D->camera.GetViewDir();
	n.Normalize();
	float t;
	scModel model = scGetClothModel(mCloth);
	Vector3 p;
	scGetParticlePos(model, mSelected, p);
	bool hit = IntersectRayPlane(mEye, mMouse - mEye, n, 
		n.Dot(p), t, mPick);

	scUpdateMouseSpring(model, mPick);
}

void ClothDemoAPI::OnMouseDown(int x, int y)
{
	uint32 pixel = mGraphics3D->mGLS.ReadPickPixel(x, mGraphics3D->h - y);
	int primitive = (pixel & 0xffff) - 1;
	int drawLo = (pixel >> 16) & 0xff;
	int drawHi = 255 - ((pixel >> 24) & 0xff);
	int draw = (drawHi >> 8) | drawLo;

	mEye = mGraphics3D->camera.GetPosition();
	mMouse = mGraphics3D->ComputeMousePoint(x, y);

	// if cloth is clicked
	if (draw == mClothDraw1 || draw == mClothDraw2)
	{
		mSelectedTriangle = primitive;

		// find the primitive inside the render mesh
		scMesh clothMesh = scGetClothMesh(mCloth);
		Vector3* vertices = scGetMeshVertexBuffer(clothMesh);
		uint32* indices = scGetMeshIndexBuffer(clothMesh);

		int i0 = indices[primitive * 3];
		int i1 = indices[primitive * 3 + 1];
		int i2 = indices[primitive * 3 + 2];
		const Vector3& v0 = vertices[i0];
		const Vector3& v1 = vertices[i1];
		const Vector3& v2 = vertices[i2];

		Vector3 coords;
		float t;
		if (IntersectRayTriangle(mEye, mMouse, v0, v1, v2, coords, t, mPick))
		{
			mSelected = (int)i0;
			if (coords.y < coords.x)
				mSelected = (int)i1;
			if (coords.z > coords.x && coords.z > coords.y)
				mSelected = (int)i2;
			mSelTri[0] = i0;
			mSelTri[1] = i1;
			mSelTri[2] = i2;
			mSelCoords = coords;			
			scAddMouseSpring(scGetClothModel(mCloth), i0, i1, i2, coords, mPick);
		}
	}
}

void ClothDemoAPI::OnMouseUp(int x, int y)
{
	if (mSelected > 0)
	{
		scRemoveMouseSpring(scGetClothModel(mCloth));
		mSelected = -1;
	}
}

