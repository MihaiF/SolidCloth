#version 140

in vec2 UV;

//layout(location = 0) 
//out vec3 colorOut;
 
uniform sampler2D texSampler;
uniform int width;
uniform int height;
uniform int size;
uniform float kernel[80];

vec3 KernelX(float x, float y)
{
	vec3 val = vec3(0);
	int h = size / 2;
	float inc = 1.0 / width;
	for (int i = -h; i <= h; i++)
	{
		val += kernel[i + h] * texture(texSampler, vec2(x + i * inc, y)).rgb;
	}
	return val;
}

vec3 KernelY(float x, float y)
{
	vec3 val = vec3(0);
	int h = -size / 2;
	float inc = 1.0 / height;
	for (int i = -h; i <= h; i++)
	{
		val += kernel[i + h] * texture(texSampler, vec2(x, y + i * inc)).rgb;
	}
	return val;
}

void main()
{ 	
	if (size > 0)
		gl_FragColor = vec4(KernelX(UV.x, UV.y), 1);
	else
		gl_FragColor = vec4(KernelY(UV.x, UV.y), 1);
}