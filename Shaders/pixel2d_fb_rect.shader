#version 140

in vec2 UV;

//layout(location = 0) 
out vec3 colorOut;
 
uniform sampler2DRect texSampler;
//uniform int width;
//uniform int height;
uniform int size;
uniform float kernel[80];

vec3 KernelX(float x, float y)
{
	vec3 val = vec3(0);
	int h = size / 2;	
	for (int i = -h; i <= h; i++)
	{
		val += kernel[i + h] * texture(texSampler, vec2(x + i, y)).rgb;
	}
	return val;
}

vec3 KernelY(float x, float y)
{
	vec3 val = vec3(0);
	int h = -size / 2;
	for (int i = -h; i <= h; i++)
	{
		val += kernel[i + h] * texture(texSampler, vec2(x, y + i)).rgb;
	}
	return val;
}

void main()
{ 	
	if (size > 0)
		colorOut = KernelX(UV.x, UV.y);
	else
		colorOut = KernelY(UV.x, UV.y);
		
	//colorOut = texture(texSampler, UV).rgb;
}