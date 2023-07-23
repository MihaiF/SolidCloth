#version 330

uniform int gDrawIndex;
//uniform uint gObjectIndex;

out vec4 FragColor;

void main()
{
	int id = gl_PrimitiveID + 1;
	int r = id & 0xff;
	int g = id >> 8;
	int b = (gDrawIndex) & 0xff;
	int a = (gDrawIndex) >> 8;
	FragColor = vec4(float(r) / 255.0f, float(g) / 255.f, float(b) / 255.f, float(255 - a) / 255.f);
}