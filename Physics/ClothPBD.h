#pragma once
#include "ClothModel.h"

namespace Physics
{
	class ClothModelPBD : public ClothModel
	{
	public:
		ClothModelPBD(ClothPatch* owner) : ClothModel(owner) { }
		ClothModelPBD(const ClothModel& model) : ClothModel(model) { }
		void Step(float h);
		void Init();
	private:
		void SolveGS(float h);
		void SolveJacobi(float h);
		void SolveCR(float h, float alpha0);
		void SolveLinks(float h, float omega);
		void SolveLinksJacobi(float h, float omega);
		int SolveSelfTris();
		int SolveSelfEdges();
		int SolveContacts(float h);
		void SolveTriContacts();
		void SolveTriangles(float h);
		void SolveTrianglesEnergy(float h);
		void SolveBends(float h);
	};
}