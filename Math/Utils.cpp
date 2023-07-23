#include <Engine/Base.h>
#include <Engine/xml.h>

namespace Math
{
	Vector3 ReadVector3(XMLElement* xPos)
	{
		char str[32];
		xPos->FindVariableZ("x")->GetValue(str);
		float x = (float)atof(str);
		xPos->FindVariableZ("y")->GetValue(str);
		float y = (float)atof(str);
		xPos->FindVariableZ("z")->GetValue(str);
		float z = (float)atof(str);
		return Math::Vector3(x, y, z);
	}
}
