#pragma once

#include "ClothDemoAPI.h"

#include "Engine/Engine.h"

class DemoHost : public Engine // TODO: rename to DemoHostAPI
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
	ClothDemoAPI mClothDemo;
	bool pausePhysics = true;
	bool stepByStep = false;
	float timeStep = 0.016f;
	bool mShowDebug = false;
	int mDebugDrawFlags = 0;
	int mDemoType = ClothDemoAPI::CLOTH_DEMO_DEFAULT;
	bool mLightFollowsCamera = true;
};
