#pragma once

#include "ClothDemo.h"
#include "Graphics2D/Graphics2D.h"
#include "Graphics3D/Graphics3D.h"


class DemoHost : public Engine
{
public:
	DemoHost()
	{
		canResize = true;
		is3D = true;
		graphics->SetBgColor(0xff9f9f9f);
	}

	void OnCreate() override;

	void OnUpdate(float dt) override;

	void OnDraw3D() override
	{
		mClothDemo.Draw(graphics3D, mShowDebug, mDebugDrawFlags);
	}

	void OnDrawUI() override;

	void Reset()
	{
		mClothDemo.Init();
	}

private:
	ClothDemo mClothDemo;
	bool pausePhysics = true;
	bool stepByStep = false;
	float timeStep = 0.016f;
	bool mShowDebug = false;
	int mDebugDrawFlags = 0;
};
