namespace Physics
{
	inline void ClothModel::AddParticle(Math::Vector3 pos, float invMass, Math::Vector2 uv)
	{
		Particle p;
		p.pos = pos;
		p.invMass = invMass;
		p.uv = uv;
		mParticles.push_back(p);
	}

	inline void ClothModel::AddLink(uint32 i1, uint32 i2, float stiffness)
	{
		if (mParticles[i1].invMass == 0 && mParticles[i2].invMass == 0)
			return;
		if (stiffness == 0)
			return;
		// TODO: constructor
		Link link;
		link.i1 = i1;
		link.i2 = i2;
		link.stiffness = stiffness;
		link.len = (mParticles[i1].pos - mParticles[i2].pos).Length();
		link.disp.SetZero();
		//const float h = 0.016f; // FIXME
		//const float k = h * h * mArea * mUnit;
		//float epsilon = link.len / (k * link.stiffness);
		//float mu = 1.f / (mParticles[i1].invMass + mParticles[i2].invMass);
		//link.omega = 1.f / (1.f + mu * epsilon);
		mLinks.push_back(link);
	}

	inline int ClothModel::AddContact(size_t idx, Math::Vector3 p, Math::Vector3 n, Math::Vector3 vel, 
		int tri, const Geometry::Mesh* mesh)
	{
		Contact contact;
		contact.idx = (unsigned)idx;
		contact.point = p;
		contact.normal = n;
		contact.vel = vel;
		contact.feature = tri;
		contact.mesh = mesh;
		mContacts.push_back(contact);
		return (int)(mContacts.size() - 1);
	}

	inline int ClothModel::AddEdgeContact(int i1, int i2, Math::Vector3 p, Math::Vector3 n, Math::Vector2 coords, 
		Math::Vector3 vel, int edge, const Geometry::Mesh* mesh)
	{
		EdgeContact contact;
		contact.normal = n;
		contact.point = p;
		contact.i1 = i1;
		contact.i2 = i2;
		contact.w1 = coords.x;
		contact.w2 = coords.y;
		contact.feature = edge;
		contact.mesh = mesh;
		contact.vel = vel;
		mEdgeContacts.push_back(contact);
		return (int)(mEdgeContacts.size() - 1);
	}

	inline int ClothModel::AddTriContact(int i1, int i2, int i3, Math::Vector3 p, Math::Vector3 n, 
		Math::Vector3 bar, Math::Vector3 vel, int vtx, const Geometry::Mesh* mesh)
	{
		ASSERT(bar.Length() > 0);
		TriContact contact;
		contact.normal = n;
		contact.point = p;
		contact.i1 = i1;
		contact.i2 = i2;
		contact.i3 = i3;
		contact.w1 = bar.x;
		contact.w2 = bar.y;
		contact.w3 = bar.z;
		contact.vel = vel;
		contact.feature = vtx;
		contact.mesh = mesh;
		mTriContacts.push_back(contact);
		return (int)(mTriContacts.size() - 1);
	}

	inline void ClothModel::AddMouseSpring(int i1, int i2, int i3, Math::Vector3 coords, Math::Vector3 p)
	{
		mMouseSpring.active = true;
		mMouseSpring.len = 0; // float((mParticles[i].pos - p).Length());
		mMouseSpring.stiffness = 100.f;
		mMouseSpring.i1 = i1;
		mMouseSpring.i2 = i2;
		mMouseSpring.i3 = i3;
		mMouseSpring.a1 = coords.x;
		mMouseSpring.a2 = coords.y;
		mMouseSpring.a3 = coords.z;
		mMouseSpring.point = p;
	}

	inline void ClothModel::AddTriangle(size_t i1, size_t i2, size_t i3)
	{
		AddTriangle(i1, i2, i3, mParticles[i1].uv, mParticles[i2].uv, mParticles[i3].uv);
	}

	inline void ClothModel::Clear()
	{
		mParticles.clear();
		mLinks.clear();
		mEdges.clear();
		mTriangles.clear();
		mQuads.clear();
		mBends.clear();
		mFrames = 0;
		mSelfEdges.clear();
		mSelfTris.clear();
		mTriContacts.clear();
		mCacheVT.clear();
		mCacheEE.clear();
		mCacheTV.clear();
		mClosestPoints.Clear();
	}

	inline void ClothModel::UpdateMesh(Geometry::Mesh& mesh, bool isQuadMesh, Math::Vector3 position, bool usePrev)
	{
		for (size_t i = 0; i < mesh.vertices.size(); i++)
		{
			const auto& p = GetParticle(isQuadMesh ? i : mMap[i]);
			mesh.vertices[i] = usePrev ? p.prev : p.pos + position;
		}
	}

	inline bool ClothModel::CheckSignal()
	{
		if (mSignal)
		{
			mSignal = false;
			return true;
		}
		return false;
	}

}