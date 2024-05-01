#include <Engine/Base.h>
#include <Engine/Profiler.h>
#include <Graphics2D/Graphics2D.h>

void Profiler::Draw(Graphics2D* graphics, double stopTime)
{
	const float refFrameLen = 30.0f;
	const float left = 10;
	const float top = 50;
	const float height = 30;

	// draw profiler names and aggregated times
	int thread = 0;
	const int MAX_IDENT = 32;
	char* ident[MAX_IDENT];
	graphics->SetColor(0xff000000);
	{
		int yOff = 0;
		for (std::map<std::string, size_t>::const_iterator it = nameMap.begin(); it != nameMap.end(); ++it)
		{
			if (drawTimes[thread].empty())
				return;
			size_t i = it->second;
			if (i >= drawTimes[thread].size())
				break;
			if (drawTimes[thread][i].start.empty())
				continue;
			memset(ident, ' ', MAX_IDENT);
			ident[drawTimes[thread][i].level] = '\0';
			graphics->DrawFormattedString(left + thread * graphics->w * 0.5f, top + 2 * (height + 15) + yOff, "%s %s: %.2fms", 
				ident, it->first.c_str(), drawTimes[thread][i].total);
			yOff += 12;
		}
	}

	float barWidth = graphics->w - 20.0f;
	float profileWidth = (float)(stopTime - prevStartTime) / refFrameLen * barWidth;
	graphics->SetColor(0xff808080);
	graphics->FillRect(left, top, profileWidth, height);
	graphics->FillRect(left, top + height + 5, profileWidth, height);
	for (int k = 0; k < NUM_THREADS; k++) 
	{
		for (size_t i = 0; i < drawTimes[k].size(); i++) 
		{
			graphics->SetColor(drawTimes[k][i].color);
			for (size_t j = 0; j < drawTimes[k][i].start.size(); j++) 
			{
				float len = drawTimes[k][i].stop[j] - drawTimes[k][i].start[j];
				profileWidth = len / refFrameLen * barWidth;
				float start = drawTimes[k][i].start[j];
				float profileX = start / refFrameLen * barWidth;
				graphics->FillRect(left + profileX, top + k * (height + 5), profileWidth, height);
			}
		}
	}
}

void Profiler::Stop()
{
	prevStartTime = startTime;
	for (int k = 0; k < NUM_THREADS; k++) 
	{
		for (size_t i = 0; i < profilerTimes[k].size(); i++) 
		{
			profilerTimes[k][i].total = 0.f;
			ASSERT(profilerTimes[k][i].stop.size() == profilerTimes[k][i].start.size());
			for (size_t j = 0; j < profilerTimes[k][i].start.size(); j++) 
			{
				float len = profilerTimes[k][i].stop[j] - profilerTimes[k][i].start[j];
				profilerTimes[k][i].total += len;
			}
		}
		drawTimes[k] = profilerTimes[k];
		for (size_t i = 0; i < profilerTimes[k].size(); i++) 
		{
			profilerTimes[k][i].start.clear();
			profilerTimes[k][i].stop.clear();
		}
	}
}

int Profiler::AddProfile(const char* name, uint32 c)
{
	// add name
	const int thread = 0;
	nameMap.insert(std::pair<std::string, size_t>(std::string(name), profilerTimes[thread].size()));

	// create profile data
	ProfileData pd;
	pd.color = c;
	pd.level = mLevel;
	profilerTimes[thread].push_back(pd);
	size_t idx = profilerTimes[thread].size() - 1;

	// construct tree
	//ProfileTree* node = new ProfileTree();
	//node->mName = name;
	//node->mParent = mCurrentRoot;
	//node->mData = &profilerTimes[thread][idx];
	//mCurrentRoot->mChildren.push_back(node);
	//mCurrentRoot = node;
	//mAddCounter++;

	return (int)idx;
}
