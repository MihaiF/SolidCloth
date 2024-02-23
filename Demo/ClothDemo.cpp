#include "ClothDemo.h"
#include "Physics/ClothModel.h"
#include "Geometry/Collision3D.h"
#include "Geometry/Assets.h"
#include "Geometry/AabbTree.h"
#include "Geometry/MarchingCubes.h"
#include "Graphics3D/Graphics3D.h"
#include "Graphics2D/Graphics2D.h"
#include "Math/Utils.h"

using namespace Physics;
using namespace Geometry;

ClothDemo::ClothDemo()
	: mDemoType(CLOTH_DEMO_DEFAULT)
	, mAvatarAsset(AVATAR_ASSET_BUDDHA)
	, mHorizontal(false)
	, mAttached(false)
	, mSelected(-1)
	, mCloth(mCollWorld)
	, mDivisions(30)
{
	mCheckerTexture.reset(new Texture());
}

void ClothDemo::Create(int type)
{
	if (!mCheckerTexture->LoadTexture("../Res/checker.bmp"))
		Printf("Failed to load texture\n");

	mDemoType = type;
	mCloth.GetModel().SetCollisionFlags(CF_WALLS | CF_VERTICES | CF_TRIANGLES | CF_EDGES | CF_SELF);
}

void ClothDemo::Init()
{
	mFrame = 0;

	Aabb3 walls(Vector3(-60), Vector3(60));
	mCollWorld.ClearCollidables();
	mCollWorld.AddCollidable(std::shared_ptr<Collidable>(new Walls(walls)));

	if (mDemoType == CLOTH_DEMO_SPHERE)
	{
		mCollWorld.AddCollidable(std::shared_ptr<Collidable>(new Sphere(Vector3(0, -10, 0), 25)));
	}
	else if (mDemoType == CLOTH_DEMO_CAPSULE)
	{
		Quaternion q;
		q.SetAxisAngle(1.3f, Vector3(1, 0.5f, 0));
		mCollWorld.AddCollidable(std::shared_ptr<Collidable>(new Capsule(Vector3(0, -10, 0), 16, 15, q)));
	}
	else if (mDemoType == CLOTH_DEMO_SDF)
	{
		mSDF.LoadFromFile("../Models/bunny.sdf");
		mSDF.ComputeGradient();

		CollisionSDF* collSDF = new CollisionSDF(&mSDF);

		auto eval = [&](int x, int y, int z) { return mSDF.GetValue(x, y, z); };
		auto grad = [&](const Vector3& p) { return mSDF.GetGrad(p); };
		int precision[3];
		mSDF.GetNumCellsPerSide(precision);
		collSDF->isoMesh.Clear();
		CreateIsoSurface(collSDF->isoMesh, eval, grad, mSDF.GetBox(), precision);
		mCollWorld.AddCollidable(std::shared_ptr<Collidable>(collSDF));
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
			mMeshCopy = mMesh; // make a copy; TODO: why
			mCollisionMesh.reset(new CollisionMesh(&mMeshCopy));
			mCollWorld.AddCollidable(std::shared_ptr<Collidable>(mCollisionMesh));
		}
		else
			Printf("Could not load mesh\n");
	}

	SetupCloth(mCloth);
	InitCloth(mCloth, Vector3(0, 60, 10));

	// this is a hack to reinitialized the collision mesh AABB tree based on new params
	for (int i = 0; i < mCollWorld.GetNumCollidables(); i++)
	{
		if (mCollWorld.GetCollidable(i)->mType == CT_MESH)
		{
			CollisionMesh* mesh = (CollisionMesh*)mCollWorld.GetCollidable(i);
			mesh->invalidate = true;
		}
	}
}

void ClothDemo::SetupCloth(ClothPatch& cloth)
{
	// FIXME
	mHorizontal = true;
	mAttached = false;

	if (mDemoType == CLOTH_DEMO_DEFAULT)
	{
		mHorizontal = false;
		mAttached = true;
		cloth.GetModel().SetCollisionFlags(0);
	}
}

void ClothDemo::InitCloth(ClothPatch& cloth, Vector3 offset)
{
	cloth.Init(mDivisions, mDivisions, 80.f / mDivisions, offset, mHorizontal, mAttached);
}

void ComputeStrainMap(const Physics::ClothPatch& cloth, std::vector<Vector3>& colors)
{
	for (size_t i = 0; i < cloth.GetModel().GetNumParticles(); i++)
		colors[i].SetZero();
	float maxStrain = 0;
	float minStrain = 20;
	for (size_t i = 0; i < cloth.GetModel().GetNumTris(); i++)
	{
		Physics::Triangle tri = cloth.GetModel().GetTriangle(i);
		const Physics::Particle& p1 = cloth.GetModel().GetParticle(tri.i1);
		const Physics::Particle& p2 = cloth.GetModel().GetParticle(tri.i2);
		const Physics::Particle& p3 = cloth.GetModel().GetParticle(tri.i3);
		Vector3 dx1 = (p2.pos - p1.pos);
		Vector3 dx2 = (p3.pos - p1.pos);

		Vector3 wu = tri.invDet * (tri.dv2 * dx1 - tri.dv1 * dx2);
		Vector3 wv = tri.invDet * (-tri.du2 * dx1 + tri.du1 * dx2);

		tri.euu = 0.5f * ((wu * wu) - 1);
		tri.evv = 0.5f * ((wv * wv) - 1);
		tri.euv = wu * wv;

		float strain = sqrtf(tri.euu * tri.euu + tri.evv * tri.evv + tri.euv * tri.euv);
		maxStrain = max(strain, maxStrain);
		minStrain = min(strain, minStrain);

		float s = strain;

		Vector3 e1 = dx1;
		Vector3 e2 = p3.pos - p2.pos;
		Vector3 e3 = -dx2;
		e1.Normalize();
		e2.Normalize();
		e3.Normalize();
		float t1 = acos(-(e1 * e3));
		float t2 = acos(-(e1 * e2));
		float t3 = acos(-(e2 * e3));

		s *= 10.f;
		Vector3 col(s, 0, s);
		colors[tri.i1] += (t1 / PI) * col;
		colors[tri.i2] += (t2 / PI) * col;
		colors[tri.i3] += (t3 / PI) * col;
	}
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

void ClothDemo::DrawUI()
{
#ifdef USE_IMGUI
	if (ImGui::CollapsingHeader("Cloth"))
	{
		ImGui::Checkbox("Wireframe on shaded", &mWireframeCloth);
		ImGui::Combo("Avatar asset", &mAvatarAsset, "None\0Bunny\0Buddha\0Dragon\0Armadillo\0Teapot\0Sphere\0");
		ImGui::InputInt("Divisions", &mDivisions);

		float thickness = mCloth.GetModel().GetThickness();
		if (ImGui::InputFloat("Thickness", &thickness))
			mCloth.GetModel().SetThickness(thickness);

		float tolerance = mCloth.GetModel().GetTolerance();
		if (ImGui::InputFloat("Tolerance", &tolerance))
			mCloth.GetModel().SetTolerance(tolerance);

		float friction = mCloth.GetModel().GetFriction();
		if (ImGui::InputFloat("Friction", &friction))
			mCloth.GetModel().SetFriction(friction);

		ImGui::Checkbox("Horizontal", &mHorizontal);
		ImGui::Checkbox("Attached", &mAttached);
		int iters = mCloth.GetModel().GetNumIterations();
		if (ImGui::InputInt("Iterations", &iters))
			mCloth.GetModel().SetNumIterations(iters);
		bool useFem = mCloth.GetModel().GetFEM();
		if (ImGui::Checkbox("Use FEM", &useFem))
			mCloth.GetModel().SetFEM(useFem);
		
		int flags = mCloth.GetModel().GetCollisionFlags();		
		
		bool collVertices = flags & CF_VERTICES;
		if (ImGui::Checkbox("Vertex collisions", &collVertices))
		{
			if (collVertices)
				mCloth.GetModel().SetCollisionFlags(flags | CF_VERTICES);
			else
				mCloth.GetModel().SetCollisionFlags(flags & ~CF_VERTICES);
		}

		bool collEdges = flags & CF_EDGES;
		if (ImGui::Checkbox("Edge collisions", &collEdges))
		{
			if (collEdges)
				mCloth.GetModel().SetCollisionFlags(flags | CF_EDGES);
			else
				mCloth.GetModel().SetCollisionFlags(flags & ~CF_EDGES);
		}

		bool collTriangles = flags & CF_TRIANGLES;
		if (ImGui::Checkbox("Triangle collisions", &collTriangles))
		{
			if (collTriangles)
				mCloth.GetModel().SetCollisionFlags(flags | CF_TRIANGLES);
			else
				mCloth.GetModel().SetCollisionFlags(flags & ~CF_TRIANGLES);
		}
	}
#endif
}

void ClothDemo::Draw(Graphics3D* graphics3D, bool showDebug, int debugDrawFlags)
{
	mGraphics3D = graphics3D; // FIXME

	// store viewport properties for mouse picking
	mViewWidth = graphics3D->w;
	mViewHeight = graphics3D->h;
	mViewFOV = graphics3D->fov;

	std::vector<Vector3> colors(mCloth.GetModel().GetNumParticles());
	int flags = 0;
	mCloth.UpdateMesh();
	if (showDebug)
	{
		if (debugDrawFlags & DDF_PARTICLES)
		{
			for (size_t i = 0; i < mCloth.GetModel().GetNumParticles(); i++)
			{
				Vector3 pos = mCloth.GetModel().GetParticle(i).pos;
				float im = mCloth.GetModel().GetParticle(i).invMass;
				float m = im != 0 ? 1.f : 0.1f;
				graphics3D->SetColor(m * 0.83f, m * 0.67f, 0.f);
				graphics3D->DrawSphere(pos, mCloth.GetModel().GetThickness());
			}
		}
		if (debugDrawFlags & DDF_LINKS)
		{
			graphics3D->SetColor(0.8f, 0.3f, 0.2f);
			for (size_t i = 0; i < mCloth.GetModel().GetNumLinks(); i++)
			{
				const Physics::Link& link = mCloth.GetModel().GetLink(i);
				graphics3D->DrawLine(mCloth.GetModel().GetParticle(link.i1).pos, mCloth.GetModel().GetParticle(link.i2).pos);
			}
		}
		if (debugDrawFlags & DDF_CONTACTS)
		{
			for (size_t i = 0; i < mCloth.GetModel().GetNumContacts(); i++)
			{
				const Physics::Contact contact = mCloth.GetModel().GetContact(i);
				graphics3D->SetColor(0, 0, 1);
				Vector3 p = mCloth.GetModel().GetParticle(contact.idx).pos;
				graphics3D->DrawLine(p, p + contact.normal * 3.f);
			}
		}
		if (debugDrawFlags & DDF_TRI_CONTACTS)
		{
			for (size_t i = 0; i < mCloth.GetModel().GetNumTriContacts(); i++)
			{
				const Physics::TriContact contact = mCloth.GetModel().GetTriContact(i);
				graphics3D->SetColor(0, 1, 0);
				Vector3 p = contact.point;
				graphics3D->DrawLine(p, p + contact.normal * 3.f);
			}
		}
		if (debugDrawFlags & DDF_WARP_WEFT)
		{
			for (size_t i = 0; i < mCloth.GetModel().GetNumTris(); i++)
			{
				Physics::Triangle tri = mCloth.GetModel().GetTriangle(i);
				Physics::Particle& p1 = mCloth.GetModel().GetParticle(tri.i1);
				Physics::Particle& p2 = mCloth.GetModel().GetParticle(tri.i2);
				Physics::Particle& p3 = mCloth.GetModel().GetParticle(tri.i3);
				Vector3 dx1 = (p2.pos - p1.pos);
				Vector3 dx2 = (p3.pos - p1.pos);
				Vector3 wu = tri.invDet * (tri.dv2 * dx1 - tri.dv1 * dx2);
				Vector3 wv = tri.invDet * (-tri.du2 * dx1 + tri.du1 * dx2);
				Vector3 c = (1.f / 3.f) * (p1.pos + p2.pos + p3.pos);
				graphics3D->SetColor(1, 0, 0);
				graphics3D->DrawLine(c, c + wu * tri.su);
				graphics3D->SetColor(0, 1, 0);
				graphics3D->DrawLine(c, c + wv * tri.sv);
			}
		}
		if (debugDrawFlags & DDF_STRAIN)
		{
			flags |= ShaderFlags::VERTEX_COLORS;
			ComputeStrainMap(mCloth, colors);
		}
		if (debugDrawFlags & DDF_TREE_SELF)
			DrawTree(graphics3D, mCloth.GetModel().GetTree());
	}

	// draw cloth mesh
	{
		graphics3D->SetFlags(flags);
		if (mWireframeCloth)
			graphics3D->SetRenderMode(RM_WIREFRAME_ON_SHADED);
		graphics3D->SetColor(0, 1, 1);
		int draw1 = graphics3D->DrawMesh(mCloth.GetMesh().vertices, mCloth.GetMesh().normals, colors, mCloth.GetMesh().indices);
		if (draw1 > 0)
			mClothDraw1 = draw1;
		graphics3D->SetCulling(Graphics3D::CULL_FRONT);
		graphics3D->SetColor(0, 0, 1);
		graphics3D->SetFlipNormals(true);
		int draw2 = graphics3D->DrawMesh(mCloth.GetMesh().vertices, mCloth.GetMesh().normals, colors, mCloth.GetMesh().indices);
		if (draw2 > 0)
			mClothDraw2 = draw2;
		graphics3D->SetFlipNormals(false);
		graphics3D->SetCulling(Graphics3D::CULL_BACK);
		graphics3D->SetRenderMode(RM_SHADED);
	}

	// draw collidables
	for (size_t i = 0; i < mCollWorld.GetNumCollidables(); i++)
	{
		if (mCollWorld.GetCollidable(i)->mType == CT_MESH)
		{
			graphics3D->SetColor(0.82f, 0.94f, 0.8f);
			const CollisionMesh* collMesh = (const CollisionMesh*)mCollWorld.GetCollidable(i);
			const Mesh* mesh = collMesh->mesh;
			graphics3D->DrawMesh(mesh->vertices, mesh->normals, mesh->indices);
			if (showDebug && (debugDrawFlags & DDF_TREE))
				DrawTree(graphics3D, collMesh->tree);
		}
		if (mCollWorld.GetCollidable(i)->mType == CT_SDF)
		{
			graphics3D->SetColor(0.82f, 0.94f, 0.8f);
			const CollisionSDF* collSDF = (const CollisionSDF*)mCollWorld.GetCollidable(i);
			const SDF* sdf = collSDF->sdf;
			const Mesh* mesh = &collSDF->isoMesh;
			graphics3D->DrawMesh(mesh->vertices, mesh->normals, mesh->indices);
		}
		else if (mCollWorld.GetCollidable(i)->mType == CT_SPHERE)
		{
			const Sphere* sph = (const Sphere*)mCollWorld.GetCollidable(i);
			graphics3D->SetColor(0.82f, 0.74f, 0.8f);
			graphics3D->DrawSphere(sph->center, sph->radius);
		}
		else if (mCollWorld.GetCollidable(i)->mType == CT_CAPSULE)
		{
			const Capsule* cap = (const Capsule*)mCollWorld.GetCollidable(i);
			graphics3D->SetColor(0.82f, 0.74f, 0.8f);
			graphics3D->DrawCapsule(cap->center, cap->r, cap->hh, cap->rot.ToMatrix());
		}
		else if (mCollWorld.GetCollidable(i)->mType == CT_WALLS)
		{
			graphics3D->SetColor(0.f, 0.f, 0.f);
			const Walls* walls = (const Walls*)mCollWorld.GetCollidable(i);
			Vector3 c = walls->mBox.GetCenter();
			Vector3 e = walls->mBox.GetExtent();
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
		Vector3 p = mCloth.GetModel().GetParticle(mSelTri[0]).pos * mSelCoords.x +
			mCloth.GetModel().GetParticle(mSelTri[1]).pos * mSelCoords.y +
			mCloth.GetModel().GetParticle(mSelTri[2]).pos * mSelCoords.z;
		graphics3D->DrawSphere(p, 0.7f);
		graphics3D->DrawLine(mPick, p);
	}
}

void ClothDemo::Update(float dt)
{
	mCloth.Step(dt);
}

void ClothDemo::OnMouseMove(int x, int y)
{
	if (mSelected < 0)
		return;
	Vector3 mouseOld = mMouse;
	mMouse = mGraphics3D->ComputeMousePoint(x, y);
	mEye = mGraphics3D->camera.GetPosition();
	Vector3 n = mGraphics3D->camera.GetViewDir();
	n.Normalize();
	float t;
	bool hit = IntersectRayPlane(mEye, mMouse - mEye, n, n.Dot(mCloth.GetModel().GetParticle(mSelected).pos), t, mPick);

	mCloth.GetModel().UpdateMouseSpring(mPick);
}

void ClothDemo::OnMouseDown(int x, int y)
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
		const Mesh& clothMesh = mCloth.GetMesh();
		int i0 = clothMesh.indices[primitive * 3];
		int i1 = clothMesh.indices[primitive * 3 + 1];
		int i2 = clothMesh.indices[primitive * 3 + 2];
		const Vector3& v0 = clothMesh.vertices[i0];
		const Vector3& v1 = clothMesh.vertices[i1];
		const Vector3& v2 = clothMesh.vertices[i2];

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
			mCloth.GetModel().AddMouseSpring(i0, i1, i2, coords, mPick);
		}
	}
}

void ClothDemo::OnMouseUp(int x, int y)
{
	if (mSelected > 0)
	{
		mCloth.GetModel().RemoveMouseSpring();
		mSelected = -1;
	}
}

