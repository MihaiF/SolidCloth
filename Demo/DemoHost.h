#pragma once

#include "ClothDemo.h"
#include "Engine/Engine.h"

class DemoHost : public Engine
{
public:
	DemoHost();

	void OnCreate() override;
	void OnUpdate(float dt) override;
	void OnDraw3D() override;
	void OnDrawUI() override;
	void OnMouseMove(int x, int y) override;
	void OnMouseDown(int x, int y, MouseButton mb) override;
	void OnMouseUp(int x, int y, MouseButton mb) override;

private:
	void Reset();

private:
	ClothDemo mClothDemo;
	bool pausePhysics = true;
	bool stepByStep = false;
	float timeStep = 0.016f;
	bool mShowDebug = false;
	int mDebugDrawFlags = 0;
	int mDemoType = ClothDemo::CLOTH_DEMO_MESH;
	bool mLightFollowsCamera = true;
};
