#version 330

layout(location = 0) in vec3 in_position;

uniform mat4 model_matrix, view_proj_matrix;

out vec3 position;

void main()
{
	mat4 MVP = view_proj_matrix * model_matrix;
	gl_Position = MVP * vec4(in_position, 1); 
}
