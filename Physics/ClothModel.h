#ifndef CLOTH_MODEL_H
#define CLOTH_MODEL_H

#include <Math/Vector3.h>
#include <Math/Vector2.h>
#include <Engine/Types.h>
#include "ClothTypes.h"
#include "Geometry/Collision3D.h"
#include "Geometry/Mesh.h"
#include "Geometry/SDF.h"
#include "Geometry/ClosestPointOnMesh.h"
#include "Common.h"
#include "ClothCollisionSDF.h"
#include "Geometry/ClosestPointsBetweenMeshes.h"

#include <memory>
#include <set>

namespace Physics
{
	class ClothPatch;

	enum SolverType
	{
		SOLVER_JACOBI,
		SOLVER_GAUSS_SEIDEL,
		SOLVER_JACOBI_CR,
		SOLVER_IMPLICIT,
		SOLVER_EXPLICIT
	};

	struct SolverStats
	{
		int iterations = 0;
		float linksError = 0;
		float bendError = 0;
		float selfVTError = 0;
		float selfEEError = 0;
		int selfEECount = 0;
	};

	class ClothModel
	{
	public:
		ClothModel(ClothPatch* owner);
		virtual ~ClothModel() { }
		
		void SetNumParticles(size_t val) { mParticles.resize(val); }
		size_t GetNumParticles() const { return mParticles.size(); }
		void AddParticle(Math::Vector3 pos, float invMass, Math::Vector2 uv);
		const Particle& GetParticle(size_t i) const { return mParticles[i]; }
		Particle& GetParticle(size_t i) { return mParticles[i]; }
		void Clear();

		size_t GetNumTris() const { return mTriangles.size(); }
		const Triangle& GetTriangle(size_t i) const { return mTriangles[i]; }
		void AddTriangle(size_t i1, size_t i2, size_t i3, Math::Vector2 uv1, Math::Vector2 uv2, Math::Vector2 uv3);
		void AddTriangle(size_t i1, size_t i2, size_t i3);
		
		uint32 GetNumQuads() const { return mQuads.size(); }
		void AddQuad(uint32 i1, uint32 i2, uint32 i3, uint32 i4) { mQuads.push_back({ i1, i2, i3, i4 }); }
		const Quad& GetQuad(uint32 i) const { return mQuads[i]; }

		size_t GetNumLinks() const { return mLinks.size(); }
		const Link& GetLink(size_t i) const { return mLinks[i]; }
		void AddLink(uint32 i1, uint32 i2, float stiffness);

		size_t GetNumContacts() const { return mContacts.size(); }
		const Contact& GetContact(size_t i) const { return mContacts[i]; }
		Contact& GetContact(size_t i) { return mContacts[i]; }
		int AddContact(size_t idx, Math::Vector3 p, Math::Vector3 n, Math::Vector3 vel, int tri = -1, 
			const Geometry::Mesh* mesh = nullptr);

		size_t GetNumEdgeContacts() const { return mEdgeContacts.size(); }
		const EdgeContact& GetEdgeContact(size_t i) const { return mEdgeContacts[i]; }
		EdgeContact& GetEdgeContact(size_t i) { return mEdgeContacts[i]; }
		int AddEdgeContact(int i1, int i2, Math::Vector3 p, Math::Vector3 n, Math::Vector2 coords, 
			Math::Vector3 vel, int edge, const Geometry::Mesh* mesh);
		
		size_t GetNumTriContacts() const { return mTriContacts.size(); }
		const TriContact& GetTriContact(size_t i) const { return mTriContacts[i]; }
		TriContact& GetTriContact(size_t i) { return mTriContacts[i]; }
		int AddTriContact(int i1, int i2, int i3, Math::Vector3 p, Math::Vector3 n, Math::Vector3 bar, 
			Math::Vector3 vel, int vtx, const Geometry::Mesh* mesh);

		size_t GetNumSelfTris() const { return mSelfTris.size(); }
		const SelfContact& GetSelfTriangle(size_t i) const { return mSelfTris[i]; }
		SelfContact& GetSelfTriangle(size_t i) { return mSelfTris[i]; }
		void AddSelfTriangle(const SelfContact& selfTri) { mSelfTris.push_back(selfTri); }
		std::vector<SelfContact>& GetSelfTris() { return mSelfTris; }
		
		size_t GetNumSelfEdges() const { return mSelfEdges.size(); }
		const SelfContact& GetSelfEdge(size_t i) const { return mSelfEdges[i]; }
		SelfContact& GetSelfEdge(size_t i) { return mSelfEdges[i]; }
		void AddSelfEdge(const SelfContact& selfEdge) { mSelfEdges.push_back(selfEdge); }
		std::vector<SelfContact>& GetSelfEdges() { return mSelfEdges; }

		void SetPositions(const Math::Vector3* positions);
		void SetMasses(const float* masses);

		const std::vector<Geometry::Mesh::Edge>& GetEdges() const { return mEdges; }
		const Geometry::Mesh::Edge& GetEdge(int i) const { return mEdges[i]; }
		void AddEdge(const Geometry::Mesh::Edge& e) { mEdges.push_back(e); }

		void AddBendConstraint(const Geometry::Mesh::Edge& e, bool useLink);
		
		virtual void Step(float h) = 0;		
		virtual void Init() {}

		float GetStretchStiffness() const { return mStretchStiff; }
		void SetStretchStiffness(float val) { mStretchStiff = val; }
		float GetShearStiffness() const { return mShearStiff; }
		void SetShearStiffness(float val) { mShearStiff = val; }
		float GetBendStiffness() const { return mBendStiff; }
		void SetBendStiffness(float val) { mBendStiff = val; }
		void SetThickness(float val) { mThickness = val; }
		float GetThickness() const { return mThickness; }
		void SetTolerance(float val) { mTolerance = val; }
		float GetTolerance() const { return mTolerance; }
		void SetNumIterations(int val) { mNumIterations = val; }
		int GetNumIterations() const { return mNumIterations; }
		bool GetDihedral() const { return mDihedral; }
		void SetDihedral(bool val) { mDihedral = val; }
		bool GetFEM() const { return mFEM; }
		void SetFEM(bool val) { mFEM = val; }
		float GetFriction() const { return mFriction; }
		void SetFriction(float val) { mFriction = val; }
		float GetPoisson() const { return mPoisson; }
		void SetPoisson(float val) { mPoisson = val; }
		float GetArea() const { return mArea; }
		void SetArea(float val) { mArea = val; }
		float GetUnit() const { return mUnit; }
		void SetUnit(float val) { mUnit = val; }
		float GetSelfThreshold() const { return mSelfCollThreshold; }
		float GetNewmarkAlpha() const { return mAlpha; }
		void SetNewmarkAlpha(float val) { mAlpha = val; }
		SolverType GetSolver() const { return mSolver; }
		void SetSolver(SolverType val) { mSolver = val; }
		void SetCollisionFlags(int val) { mCollFlags = val; }
		int GetCollisionFlags() const { return mCollFlags; }
		float GetSelfShrink() const { return mSelfShrink; }
		void SetSelfShrink(float val) { mSelfShrink = val; }
		float GetSelfSoften() const { return mSelfSoften; }
		void SetSelfSoften(float val) { mSelfSoften = val; }
		float GetDragCoeff() const { return mDragCoeff; }
		void SetDragCoeff(float val) { mDragCoeff = val; }

		Geometry::AabbTree*& GetTree() { return mTree; }

		void AddMouseSpring(int i1, int i2, int i3, Math::BarycentricCoords coords, Math::Vector3 p);
		void RemoveMouseSpring() { mMouseSpring.active = false; }
		void UpdateMouseSpring(Math::Vector3 p) { if (mMouseSpring.active) mMouseSpring.point = p; }

		void SetUseCL(bool val) { mUseCL = val; }
		void PrintInfo();

		void ResetConstraints();
		void DetectCollisions();

		int GetCacheVT(int i) const { return mCacheVT[i]; }
		int GetCacheTV(int i) const { return mCacheTV[i]; }
		int GetCacheEE(int i) const { return mCacheEE[i]; }

		void UpdateMesh(Geometry::Mesh& mesh, bool isQuadMesh, Math::Vector3 position, bool usePrev = false);

		void SignalStop() { mSignal = true; }
		bool CheckSignal();

	protected:
		void ExternalCollisions();
		void WallCollisions(const Geometry::Aabb3& walls);
		void SphereCollisions(Math::Vector3 sphPos, float sphRad);
		void MeshCollisions(const Geometry::Mesh& mesh, Geometry::AabbTree* triangleTree, int collFlags);

		template<typename SDFType>
		void SDFCollisions(const SDFType& sdf, const Geometry::Mesh* mesh = nullptr, bool handleInside = false, bool handleCurrent = false, bool handleBorder = false);

		void HandleMouseSpring(float h);

		void ComputeTree(int flags, int maxLevel, float tol);
		void SplitNode(Geometry::AabbTree* root, int level, int maxLevel, int flags, float tol);

	protected:
		ClothPatch* mOwnerPatch;

		float mUnit;
		// params
		float mThickness; // also radii of particles; used for collision detection
		float mTolerance; // used for collision detection
		float mShearStiff;
		float mBendStiff;
		float mStretchStiff;
		float mPoisson;
		float mFriction;
		float mArea;
		float mSelfCollThreshold;
		float mDragCoeff = 1.f;
		int mNumIterations;
		int mCollFlags;
		SolverType mSolver;
		// TODO: flags
		bool mDihedral;
		bool mFEM;
		float mSelfShrink;
		float mSelfSoften;
		int mBendFreq;
		// Newmark
		float mAlpha;
		// topology
		std::vector<Triangle> mTriangles;
		std::vector<Quad> mQuads;
		std::vector<Geometry::Mesh::Edge> mEdges;
		// model
		std::vector<Particle> mParticles;
		std::vector<Link> mLinks;
		std::vector<BendConstraint> mBends;
		std::vector<Contact> mContacts;
		std::vector<EdgeContact> mEdgeContacts;
		std::vector<TriContact> mTriContacts;
		std::vector<SelfContact> mSelfTris;
		std::vector<SelfContact> mSelfEdges;
		MouseSpring mMouseSpring;
		// collision
		Geometry::AabbTree* mTree;

		int mFrames;
		bool mUseCL;		
		std::vector<PrimitivePair> mPotentialContacts;
		std::vector<PrimitivePair> mPotentialTriContacts;

		bool mSolveSDFContacts = false;

		std::vector<int> mCacheVT;
		std::vector<int> mCacheEE;
		std::vector<int> mCacheTV;

		std::vector<std::vector<int>> mSetVT;
		std::vector<std::vector<int>> mSetTV;

		Geometry::MeshClosestPoints mClosestPoints;

		bool mSignal = false;

#ifdef ENABLE_CL
		NarrowPhaseCL collCL;
#endif

	public: // TODO: move back out
		std::vector<int> mMap;
		SolverStats mStats;
	};
};

#include "ClothModel.inl"

#endif