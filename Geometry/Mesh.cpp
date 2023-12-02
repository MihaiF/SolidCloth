#include "Mesh.h"
#include "Engine/Utils.h"

#define ANGLE_WEIGHTED

namespace Geometry
{
	void Mesh::ComputeNormals(bool flip)
	{
		normals.resize(vertices.size());
		triangles.resize(indices.size() / 3);
		for (size_t i = 0; i < vertices.size(); i++)
			normals[i].SetZero();
		for (size_t i = 0; i < indices.size(); i += 3)
		{
			const int i1 = indices[i];
			const int i2 = indices[i + 1];
			const int i3 = indices[i + 2];
			const Vector3& v1 = vertices[i1];
			const Vector3& v2 = vertices[i2];
			const Vector3& v3 = vertices[i3];
			Vector3 n = (v2 - v1).Cross(v3 - v1);
			Vector3 nt = n;
			nt.Normalize();
			triangles[i / 3].n = nt;
#ifdef ANGLE_WEIGHTED
			Vector3 e1 = v2 - v1;
			Vector3 e2 = v3 - v2;
			Vector3 e3 = v1 - v3;
			e1.Normalize();
			e2.Normalize();
			e3.Normalize();
			float c1 = -e1.Dot(e3);
			float c2 = -e1.Dot(e2);
			float c3 = -e2.Dot(e3);
			float s1 = e1.Cross(-e3).Length();
			float s2 = e2.Cross(-e1).Length();
			float s3 = e3.Cross(-e1).Length();
			float t1 = fabsf(c1) > 1 ? fabsf(atan2(s1, c1)) : acosf(c1);
			float t2 = fabsf(c2) > 1 ? fabsf(atan2(s2, c2)) : acosf(c2);
			float t3 = fabsf(c3) > 1 ? fabsf(atan2(s3, c3)) : acosf(c3);
			ASSERT(t1 >= 0 && t2 >= 0 && t3 >= 0);
			ASSERT(fabs(t1 + t2 + t3 - PI) < 1e-3f);
			normals[i1] += t1 * nt;
			normals[i2] += t2 * nt;
			normals[i3] += t3 * nt;
#else
			normals[i1] += n;
			normals[i2] += n;
			normals[i3] += n;
#endif
		}
		for (size_t i = 0; i < vertices.size(); i++)
		{
			normals[i].Normalize();
			if (flip)
				normals[i].Flip();
		}
	}

	bool Mesh::ConstructEdges()
	{
		if (indices.empty())
			return false;
		// build unique edges array
		std::vector<Mesh::Edge> duplicates(indices.size());
		triangles.resize(indices.size() / 3);
		for (size_t i = 0; i < indices.size(); i++)
		{
			// construct edge
			int i1 = indices[i];
			int i2 = indices[i + 1];
			if ((i + 1) % 3 == 0)
			{
				i2 = indices[i - 2];
				Triangle& tri = triangles[i / 3];
				tri.e[2] = (uint32)i;
				tri.e[1] = (uint32)(i - 1);
				tri.e[0] = (uint32)(i - 2);				
			}
			
			bool swapped = false;
			if (i1 > i2)
			{
				std::swap(i1, i2);
				swapped = true;
			}

			Mesh::Edge edge(i1, i2);
			edge.swapped = swapped;
			edge.t1 = (int)(i / 3);
			edge.n = triangles[i / 3].n; // initially, one single normal; TODO: this requires ComputeNormals!!!
			edge.origIdx = (int)i;
			duplicates[i] = edge;
		}
		// sort it so we can find duplicates
		std::sort(duplicates.begin(), duplicates.end(), CompareEdges);

		// add only unique edges
		Mesh::Edge currEdge = duplicates[0];
		std::vector<int> map(duplicates.size(), -1);
		map[currEdge.origIdx] = 0;
		for (size_t i = 1; i < duplicates.size(); i++)
		{
			if (duplicates[i].i1 == duplicates[i - 1].i1 && duplicates[i].i2 == duplicates[i - 1].i2)
			{
				map[duplicates[i].origIdx] = (int)edges.size();
				currEdge.t2 = duplicates[i].t1;
				currEdge.n = triangles[currEdge.t1].n + triangles[currEdge.t2].n; // unweighted average
				currEdge.n.Normalize();
				continue;
			}
			edges.push_back(currEdge);
			currEdge = duplicates[i];
			map[currEdge.origIdx] = (int)edges.size();
		}
		edges.push_back(currEdge);

		// remap triangle edges
		for (size_t i = 0; i < triangles.size(); i++)
		{
			for (int j = 0; j < 3; j++)
				triangles[i].e[j] = map[triangles[i].e[j]];
		}

		return true;
	}

	void Mesh::ConstructVertexOneRings()
	{
		ASSERT(edges.size() != 0);
		if (edges.size() != 0)
		{
			vtxOneRings.resize(vertices.size());
			for (size_t i = 0; i < edges.size(); i++)
			{
				int i1 = edges[i].i1;
				int i2 = edges[i].i2;
				vtxOneRings[i1].push_back(i2);
				vtxOneRings[i2].push_back(i1);
			}
		}
	}

	void Mesh::ConstructEdgeOneRings()
	{
		ASSERT(edges.size() != 0);
		if (edges.size() != 0)
		{
			edgeOneRings.resize(vertices.size());
			for (size_t i = 0; i < edges.size(); i++)
			{
				int i1 = edges[i].i1;
				int i2 = edges[i].i2;
				edgeOneRings[i1].push_back((int)i);
				edgeOneRings[i2].push_back((int)i);
			}
		}
	}

	void Mesh::ConstructTriangleOneRings()
	{
		triOneRings.resize(vertices.size());
		for (int i = 0; i < GetNumTriangles(); i++)
		{
			for (int j = 0; j < 3; j++)
			{
				int vtx = indices[i * 3 + j];
				triOneRings[vtx].push_back(i);
			}
		}
	}

	size_t Mesh::RemoveDuplicatedVertices()
	{
		std::vector<int> map(vertices.size(), -1);
		std::vector<Vector3> newVertices;
		std::vector<Vector3> newNormals;
		std::vector<Vector2> newUVs;
		// brute force
		for (size_t i = 0; i < vertices.size(); i++)
		{
			if (map[i] >= 0)
				continue;

			for (size_t j = i + 1; j < vertices.size(); j++)
			{
				Vector3 delta = vertices[i] - vertices[j];
				// TODO: check if position or UVs differ
				if (delta.Length() < 1e-10f)
				{
					map[j] = (int)newVertices.size();
				}
			}

			map[i] = (int)newVertices.size();
			newVertices.push_back(vertices[i]);
			if (!normals.empty())
				newNormals.push_back(normals[i]);
			if (!uvs.empty())
				newUVs.push_back(uvs[i]);
		}

		// replace vertices
		size_t oldCount = vertices.size();
		vertices = newVertices;
		normals = newNormals;
		uvs = newUVs;

		// remap indices
		for (size_t i = 0; i < indices.size(); i++)
		{
			ASSERT(map[indices[i]] >= 0);
			indices[i] = map[indices[i]];
		}

		return oldCount - newVertices.size();
	}

	size_t Mesh::RemoveDuplicatedNormals()
	{
		std::vector<int> map(normals.size(), -1);
		std::vector<Vector3> newVertices;
		std::vector<Vector3> newNormals;
		std::vector<Vector2> newUVs;
		// brute force
		for (size_t i = 0; i < normals.size(); i++)
		{
			if (map[i] >= 0)
				continue;

			for (size_t j = i + 1; j < normals.size(); j++)
			{
				Vector3 delta = normals[i] - normals[j];
				// TODO: check if UVs differ
				if (delta.Length() < 1e-10f)
				{
					map[j] = (int)newNormals.size();
				}
			}

			map[i] = (int)newNormals.size();
			newNormals.push_back(normals[i]);
			if (!vertices.empty())
				newVertices.push_back(vertices[i]);
			if (!uvs.empty())
				newUVs.push_back(uvs[i]);
		}

		// replace vertices
		size_t oldCount = normals.size();
		vertices = newVertices;
		normals = newNormals;
		uvs = newUVs;

		// remap indices
		for (size_t i = 0; i < indices.size(); i++)
		{
			ASSERT(map[indices[i]] >= 0);
			indices[i] = map[indices[i]];
		}

		return oldCount - newNormals.size();
	}

	void Mesh::Transform(const Matrix4& mat, bool calcVel)
	{
		velocities.resize(vertices.size());
		for (size_t i = 0; i < vertices.size(); i++)
		{
			Vector3 v = mat.Transform(vertices[i]);
			if (calcVel)
				velocities[i] = v - vertices[i];
			vertices[i] = v;
			if (!normals.empty())
				normals[i] = mat.TransformRay(normals[i]);
		}
	}

}