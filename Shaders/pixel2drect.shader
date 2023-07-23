#version 140

in vec2 UV;

uniform sampler2DRect texSampler;
uniform int has_texture;
uniform vec4 color;
uniform int size;
uniform float kernel[80];


vec3 Kernel(vec2 UV)
{
	vec3 val = vec3(0);
	int h = size / 2;	
	for (int i = -h; i <= h; i++)
	{
		for (int j = -h; j <= h; j++)
		{
			val += kernel[i + h] * kernel[j + h] * texture(texSampler, vec2(UV.x + i, UV.y + j)).rgb;
		}
	}
	return val;
}

void main()
{ 
	if (has_texture == 1)
	{
		// TODO: RGBA textures		
		gl_FragColor = vec4(texture(texSampler, UV).rgb, color.w);
	}
	else if (has_texture == 2)
		gl_FragColor = texture(texSampler, UV).a * color;
	else if (has_texture == 3)
		gl_FragColor = vec4(Kernel(UV), color.w);
	else
		gl_FragColor = color;
}