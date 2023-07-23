#version 140

// Input vertex data, different for all executions of this shader.
in vec3 vertexPosition;
in vec2 vertexUV;

// Output data ; will be interpolated for each fragment.
out vec2 UV;
 
// Values that stay constant for the whole mesh.
uniform mat4 MVP;

void main()
{
	gl_Position = MVP * vec4(vertexPosition, 1);
	UV = vertexUV;
}

