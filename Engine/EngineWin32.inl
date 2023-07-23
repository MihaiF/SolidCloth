inline uint8 Engine::GetKeyState(int keyCode)
{
	ASSERT(keyCode < MAX_KEYS);
	const int state = keyStates[keyCode];
	keyStates[keyCode] = ((GetAsyncKeyState(keyCode) >> 14 ) & 2) | (state >> 1);
	return keyStates[keyCode];
}

inline bool Engine::isKeyPressed(int keyCode)
{
	HWND hActive = GetFocus();
	if (hWnd != hActive)
		return false;
#ifdef USE_IMGUI
	if (ImGui::GetCurrentContext() != nullptr &&
		(ImGui::GetIO().WantCaptureMouse || ImGui::GetIO().WantCaptureKeyboard))
		return false;
#endif
		//return (GetAsyncKeyState(keyCode) & 0x8000) != 0;
	return GetKeyState(keyCode) == KEY_PRESSED;
}

inline bool Engine::isKeyDown(int keyCode)
{
	HWND hActive = GetFocus();
	if (hWnd != hActive)
		return false;
#ifdef USE_IMGUI
	if (ImGui::GetCurrentContext() != nullptr &&
		(ImGui::GetIO().WantCaptureMouse || ImGui::GetIO().WantCaptureKeyboard))
		return false;
#endif
	return (GetKeyState(keyCode) & 2) != 0;
}

inline bool Engine::wasKeyReleased(int keyCode)
{
	HWND hActive = GetFocus();
	if (hWnd != hActive)
		return false;
#ifdef USE_IMGUI
	if (ImGui::GetCurrentContext() != nullptr &&
		(ImGui::GetIO().WantCaptureMouse || ImGui::GetIO().WantCaptureKeyboard))
		return false;
#endif
	return GetKeyState(keyCode) == KEY_RELEASED;
}

inline void Engine::GetMousePosition(POINT& point)
{
	GetCursorPos(&point);
	ScreenToClient(hWnd, &point);
}

uint64 Engine::GetTimer()
{
	LARGE_INTEGER timer;
	QueryPerformanceCounter(&timer);
	return timer.QuadPart;
}

inline void Engine::Quit()
{
	PostQuitMessage(0);
}
