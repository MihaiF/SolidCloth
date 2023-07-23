#ifdef WIN32

inline Profiler::Profiler()
{
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	clockFreq = (double)freq.QuadPart;
	
	startTime = 0;
	
	mLevel = -1;
	mAddCounter = 0;
	mCurrentRoot = &mRoot;
}

inline double Profiler::GetTimeMs()
{
	LARGE_INTEGER counter;
	QueryPerformanceCounter(&counter);
	return ((double)counter.QuadPart * 1000.0 / clockFreq);
}

inline double Profiler::GetTimeMsStatic()
{
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	double clockFreq = (double)freq.QuadPart;

	LARGE_INTEGER counter;
	QueryPerformanceCounter(&counter);
	return ((double)counter.QuadPart * 1000.0 / clockFreq);
}

#else // WIN32

#ifdef ANDROID_NDK
#include <time.h>
#else
#include <sys/time.h>
#endif

inline Profiler::Profiler()
{
}

inline double Profiler::GetTimeMs()
{
    struct timeval  now;
    gettimeofday(&now, NULL);
    return (now.tv_sec * 1000.0 + now.tv_usec * 0.001);
}

#endif // !WIN32

inline size_t Profiler::BeginProfile(const char* name, int thread)
{
	mLevel++;
	// TODO: CRC32 + sorted array? something at compile time?
	size_t idx = profilerTimes[thread].size();
	std::map<std::string, size_t>::iterator it = nameMap.find(name);
	if (it != nameMap.end())
	{
		idx = it->second;
	}
	else
	{
		AddProfile(name, GetRandomColor(100));
	}
	float dt = (float)(GetTimeMs() - startTime);
	profilerTimes[thread][idx].start.push_back(dt);
	return idx;
}

inline void Profiler::EndProfile(size_t idx, int thread)
{
	mLevel--;
	float dt = (float)(GetTimeMs() - startTime);
	profilerTimes[thread][idx].stop.push_back(dt);
	
	//if (mAddCounter > 0)
	//{
	//	mCurrentRoot = mCurrentRoot->mParent;
	//	mAddCounter--;
	//}
}

inline void Profiler::Start(double start)
{
	startTime = start;
}
