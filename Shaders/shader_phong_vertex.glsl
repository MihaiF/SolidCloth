#version 330

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec3 in_color;
layout(location = 3) in vec2 in_uv;

uniform mat4 model_matrix, view_proj_matrix;
uniform mat4 vp_shadow_matrix;

out vec3 color;
out vec4 ShadowCoord;

out VertexData
{
	vec3 p; // position in world space	
	vec3 n; // normal in world space
	vec2 uv;
} vs_out;

void main(){

	// compute vectors in WORLD SPACE
	vec3 position = (model_matrix * vec4(in_position,1)).xyz;
	vec3 normal = normalize(mat3(model_matrix) * in_normal);
	color = in_color;
	vec2 texcoord = in_uv;

	mat4 MVP = view_proj_matrix * model_matrix;
	gl_Position = MVP * vec4(in_position, 1); 

	ShadowCoord = vp_shadow_matrix * model_matrix * vec4(in_position, 1);	
	
	vs_out.p = position;
	vs_out.n = normal;
	vs_out.uv = texcoord;
}
