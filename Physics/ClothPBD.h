#pragma once
#include "ClothModel.h"

namespace Physics
{
	class ClothModelPBD : public ClothModel
	{
	public:
		ClothModelPBD(ClothPatch* owner) : ClothModel(owner) { }
		ClothModelPBD(const ClothModel& model) : ClothModel(model) { }
		void Step(float h) override;
		void Init() override;
	private:
		void SolveGS(float h);
		void SolveJacobi(float h);
		void SolveCR(float h, float alpha0);
		void SolveLinks(float h, float omega);
		void SolveLinksJacobi(float h, float omega);
		void SolveTriangles(float h);
		void SolveTrianglesEnergy(float h);
		void SolveBends(float h);
	};
}