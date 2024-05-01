#include "ClothAPI.h"

// for API
#include "Physics/Common.h"
#include "Physics/CollisionWorld.h"
#include "Physics/ClothPatch.h"
#include "Physics/ClothModel.h"

using namespace Physics;
using namespace Geometry;
using namespace Math;

scCollisionWorld scCreateCollisionWorld()
{
	scCollisionWorld world;
	world.ptr = new CollisionWorld();
	return world;
}

void scFreeCollisionWorld(scCollisionWorld collWorld)
{
	CollisionWorld* pCollWorld = (CollisionWorld*)collWorld.ptr;
	delete pCollWorld;
}

void scClearCollidables(scCollisionWorld collWorld)
{
	CollisionWorld* pCollWorld = (CollisionWorld*)collWorld.ptr;
	pCollWorld->ClearCollidables();
}

int scGetNumCollidables(scCollisionWorld collWorld)
{
	CollisionWorld* pCollWorld = (CollisionWorld*)collWorld.ptr;
	return pCollWorld->GetNumCollidables();
}

void scAddCollidable(scCollisionWorld collWorld, scHandle collidable)
{
	CollisionWorld* pCollWorld = (CollisionWorld*)collWorld.ptr;
	Collidable* pCollidable = (Collidable*)collidable;
	pCollWorld->AddCollidable(std::shared_ptr<Collidable>(pCollidable)); // memory will be fred by shared_ptr
}

scHandle scGetCollidable(scCollisionWorld collWorld, int i)
{
	CollisionWorld* pCollWorld = (CollisionWorld*)collWorld.ptr;
	return pCollWorld->GetCollidable(i);
}

scHandle scCreateWalls(Aabb3 box)
{
	return new Walls(box);
}

scHandle scCreateSphere(Vector3 center, float radius)
{
	return new Sphere(center, radius);
}

scHandle scCreateCapsule(Vector3 c, float rad, float halfHeight, Quaternion quat)
{
	return new Capsule(c, rad, halfHeight, quat);
}

scHandle scCreateSDF(SDF* s)
{
	return new CollisionSDF(s);
}

scHandle scCreateMesh(Mesh* m)
{
	return new CollisionMesh(m);
}

int scGetCollidableType(scHandle collidable)
{
	Collidable* pColl = (Collidable*)collidable;
	return pColl->mType;
}

AabbTree* scGetMeshTree(scHandle collMesh)
{
	Collidable* pColl = (Collidable*)collMesh;
	if (pColl->mType != CT_MESH)
		return nullptr;
	
	CollisionMesh* pCollMesh = (CollisionMesh*)collMesh;
	return pCollMesh->tree;
}

void scGetWallsInfo(scHandle walls, Aabb3& box)
{
	Collidable* pColl = (Collidable*)walls;
	if (pColl->mType != CT_WALLS)
		return;

	Walls* pWalls = (Walls*)walls;
	box = pWalls->mBox;
}

void scGetSphereInfo(scHandle sphere, float& radius, Vector3& center)
{
	Collidable* pColl = (Collidable*)sphere;
	if (pColl->mType != CT_SPHERE)
		return;

	Sphere* pSphere = (Sphere*)sphere;
	center = pSphere->center;
	radius = pSphere->radius;
}

void scGetCapsuleInfo(scHandle capsule, float& radius, float& halfHeight, Vector3& center, Quaternion& rot)
{
	Collidable* pColl = (Collidable*)capsule;
	if (pColl->mType != CT_CAPSULE)
		return;

	Capsule* pCapsule = (Capsule*)capsule;
	radius = pCapsule->r;
	halfHeight = pCapsule->hh;
	center = pCapsule->center;
	rot = pCapsule->rot;
}

scClothPatch scCreateClothPatch(scCollisionWorld world)
{
	scClothPatch patch;
	CollisionWorld* pWorld = (CollisionWorld*)world.ptr;
	patch.ptr = new ClothPatch(*pWorld);
	return patch;
}

void scFreeClothPatch(scClothPatch patch)
{
	ClothPatch* pPatch = (ClothPatch*)patch.ptr;
	delete pPatch;
}

void scInitCloth(scClothPatch patch, int divX, int divY, float inc, Vector3 offset, bool horizontal, bool attached)
{
	ClothPatch* pPatch = (ClothPatch*)patch.ptr;
	pPatch->Init(divX, divY, inc, offset, horizontal, attached);
}

void scStepCloth(scClothPatch patch, float dt)
{
	ClothPatch* pPatch = (ClothPatch*)patch.ptr;
	pPatch->Step(dt);
}

void scUpdateClothMesh(scClothPatch patch)
{
	ClothPatch* pPatch = (ClothPatch*)patch.ptr;
	pPatch->UpdateMesh();
}

void scComputeStrainMap(scClothPatch patch, std::vector<Vector3>& colors)
{
	ClothPatch* pPatch = (ClothPatch*)patch.ptr;
	pPatch->ComputeStrainMap(colors);
}

scMesh scGetClothMesh(scClothPatch patch)
{
	ClothPatch* pPatch = (ClothPatch*)patch.ptr;
	scMesh mesh;
	mesh.ptr = pPatch->GetMeshPtr();
	return mesh;
}

scModel scGetClothModel(scClothPatch patch)
{
	ClothPatch* pPatch = (ClothPatch*)patch.ptr;
	scModel model;
	model.ptr = pPatch->GetModelPtr();
	return model;
}

int scGetNumParticles(scModel model)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	return pModel->GetNumParticles();
}

void scGetParticlePos(scModel model, int i, Vector3& pos)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	pos = pModel->GetParticle(i).pos;
}

float scGetParticleInvMass(scModel model, int i)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	return pModel->GetParticle(i).invMass;
}

int scGetNumTris(scModel model)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	return pModel->GetNumTris();
}

int scGetNumLinks(scModel model)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	return pModel->GetNumLinks();
}

void scGetLink(scModel model, int i, int& i1, int& i2)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	Link link = pModel->GetLink(i);
	i1 = link.i1;
	i2 = link.i2;
}

void scGetTriangle(scModel model, int i, int& i1, int& i2, int& i3)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	Triangle tri = pModel->GetTriangle(i);
	i1 = tri.i1;
	i2 = tri.i2;
	i3 = tri.i3;
}

void scGetTriangleInfo(scModel model, int i, float& invDet, Vector2& du, Vector2& dv, Vector2& s)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	Triangle tri = pModel->GetTriangle(i);
	invDet = tri.invDet;
	du.Set(tri.du1, tri.du2);
	dv.Set(tri.dv1, tri.dv2);
	s.Set(tri.su, tri.sv);
}

int scGetNumQuads(scModel model)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	return pModel->GetNumQuads();
}

void scGetQuad(scModel model, int i, int& i1, int& i2, int& i3, int& i4)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	Quad quad = pModel->GetQuad(i);
	i1 = quad.i1;
	i2 = quad.i2;
	i3 = quad.i3;
	i4 = quad.i4;
}

int scGetNumContacts(scModel model)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	return pModel->GetNumContacts();
}

void scGetContact(scModel model, int i, Vector3& point, Vector3& normal)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	const Contact& c = pModel->GetContact(i);
	point = c.point;
	normal = c.normal;
}

int scGetNumTriContacts(scModel model)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	return pModel->GetNumTriContacts();
}

void scGetTriContact(scModel model, int i, Vector3& point, Vector3& normal)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	const TriContact& c = pModel->GetTriContact(i);
	point = c.point;
	normal = c.normal;
}

int scGetCollisionFlags(scModel model)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	return pModel->GetCollisionFlags();
}

void scSetCollisionFlags(scModel model, int val)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	pModel->SetCollisionFlags(val);
}

float scGetThickness(scModel model)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	return pModel->GetThickness();
}

void scSetThickness(scModel model, float val)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	pModel->SetThickness(val);
}

float scGetTolerance(scModel model)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	return pModel->GetTolerance();
}

void scSetTolerance(scModel model, float val)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	pModel->SetTolerance(val);
}

float scGetFriction(scModel model)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	return pModel->GetFriction();
}

void scSetFriction(scModel model, float val)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	pModel->SetFriction(val);
}

int scGetNumIterations(scModel model)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	return pModel->GetNumIterations();
}

void scSetNumIterations(scModel model, int val)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	pModel->SetNumIterations(val);
}

bool scGetUseFEM(scModel model)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	return pModel->GetFEM();
}

void scSetUseFEM(scModel model, bool val)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	pModel->SetFEM(val);
}

Geometry::AabbTree* scGetClothTree(scModel model)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	return pModel->GetTree();
}


void scAddMouseSpring(scModel model, int i1, int i2, int i3, Math::Vector3 coords, Math::Vector3 p)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	pModel->AddMouseSpring(i1, i2, i3, coords, p);
}

void scUpdateMouseSpring(scModel model, Vector3 p)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	pModel->UpdateMouseSpring(p);
}

void scRemoveMouseSpring(scModel model)
{
	ClothModel* pModel = (ClothModel*)model.ptr;
	pModel->RemoveMouseSpring();
}

int scGetMeshVertexCount(scMesh mesh)
{
	Mesh* pMesh = (Mesh*)mesh.ptr;
	return pMesh->vertices.size();
}

Vector3* scGetMeshVertexBuffer(scMesh mesh)
{
	Mesh* pMesh = (Mesh*)mesh.ptr;
	return pMesh->vertices.data();
}

Vector3* scGetMeshNormalBuffer(scMesh mesh)
{
	Mesh* pMesh = (Mesh*)mesh.ptr;
	return pMesh->normals.data();
}

int scGetMeshIndexCount(scMesh mesh)
{
	Mesh* pMesh = (Mesh*)mesh.ptr;
	return pMesh->indices.size();
}

unsigned int* scGetMeshIndexBuffer(scMesh mesh)
{
	Mesh* pMesh = (Mesh*)mesh.ptr;
	return pMesh->indices.data();
}
