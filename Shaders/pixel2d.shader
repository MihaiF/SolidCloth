#version 140

in vec2 UV;

uniform sampler2D texSampler;
uniform sampler2D texSampler2;
uniform sampler2D texSampler3;
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
			val += kernel[i + h] * kernel[j + h] * texture2D(texSampler, vec2(UV.x + i, UV.y + j)).rgb;
		}
	}
	return val;
}

void main()
{ 
	if (has_texture == 1)
	{
		vec4 texel = texture2D(texSampler, UV);
		gl_FragColor = vec4(texel.rgb, texel.a * color.w);
	}
	else if (has_texture == 2)
		gl_FragColor = texture2D(texSampler, UV).a * color;
	else if (has_texture == 3)
		gl_FragColor = vec4(Kernel(UV), 1);//color.w);
	else if (has_texture == 4)
		gl_FragColor = color.r * texture2D(texSampler, UV) + color.g * texture2D(texSampler2, UV) + color.b * texture2D(texSampler3, UV);
	else
		gl_FragColor = color;
}