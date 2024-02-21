#include "Engine/Base.h"
#include "Engine/Engine.h"
#include "OpenGLSData.h"

#include <fstream>
#include <string>

#define SHADOWS_PCF

bool LoadShader(const char* path, GLenum type, GLuint& ShaderID)
{
	ShaderID = glCreateShader(type);
	// Read the Vertex Shader code from the file
	std::string VertexShaderCode;
	std::ifstream VertexShaderStream(path, std::ios::in);
	if (VertexShaderStream.is_open())
	{
		std::string Line = "";
		while (std::getline(VertexShaderStream, Line))
			VertexShaderCode += "\n" + Line;
		VertexShaderStream.close();
	}
	else
	{
		Printf("Could not open file %s\n", path);
		return false;
	}

	GLint Result = GL_FALSE;
	int InfoLogLength;

	// Compile Vertex Shader
	Printf("Compiling shader : %s\n", path);
	char const* VertexSourcePointer = VertexShaderCode.c_str();
	glShaderSource(ShaderID, 1, &VertexSourcePointer, NULL);
	glCompileShader(ShaderID);

	// Check Vertex Shader
	glGetShaderiv(ShaderID, GL_COMPILE_STATUS, &Result);
	glGetShaderiv(ShaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
	if (!Result && InfoLogLength)
	{
		std::vector<char> VertexShaderErrorMessage(InfoLogLength);
		glGetShaderInfoLog(ShaderID, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
		Printf("%s\n", &VertexShaderErrorMessage[0]);
		return false;
	}

	return true;
}

bool LoadShaders(const char* vertex_file_path, const char* fragment_file_path, GLuint& ProgramID)
{
	bool ret = true;
	GLuint VertexShaderID, FragmentShaderID;

	if (!LoadShader(vertex_file_path, GL_VERTEX_SHADER, VertexShaderID))
		return false;

	if (!LoadShader(fragment_file_path, GL_FRAGMENT_SHADER, FragmentShaderID))
		return false;

	// Link the program
	Printf("Linking program\n");
	ProgramID = glCreateProgram();
	glAttachShader(ProgramID, VertexShaderID);
	glAttachShader(ProgramID, FragmentShaderID);
	glLinkProgram(ProgramID);

	// Check the program
	GLint Result = GL_FALSE;
	int InfoLogLength;
	glGetProgramiv(ProgramID, GL_LINK_STATUS, &Result);
	glGetProgramiv(ProgramID, GL_INFO_LOG_LENGTH, &InfoLogLength);
	if (!Result && InfoLogLength)
	{
		std::vector<char> ProgramErrorMessage(max(InfoLogLength, int(1)));
		glGetProgramInfoLog(ProgramID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
		Printf("%s\n", &ProgramErrorMessage[0]);
		ret = false;
	}

	glDeleteShader(VertexShaderID);
	glDeleteShader(FragmentShaderID);

	return ret;
}

bool LoadShadersEx(const char* vertex_file_path, const char* geometry_file_path, const char* fragment_file_path, GLuint& ProgramID)
{
	bool ret = true;
	GLuint VertexShaderID, GeometryShaderID, FragmentShaderID;

	if (!LoadShader(vertex_file_path, GL_VERTEX_SHADER, VertexShaderID))
		return false;

	if (!LoadShader(geometry_file_path, GL_GEOMETRY_SHADER, GeometryShaderID))
		return false;

	if (!LoadShader(fragment_file_path, GL_FRAGMENT_SHADER, FragmentShaderID))
		return false;

	// Link the program
	Printf("Linking program\n");
	ProgramID = glCreateProgram();
	glAttachShader(ProgramID, VertexShaderID);
	glAttachShader(ProgramID, GeometryShaderID);
	glAttachShader(ProgramID, FragmentShaderID);
	glLinkProgram(ProgramID);

	// Check the program
	GLint Result = GL_FALSE;
	int InfoLogLength;
	glGetProgramiv(ProgramID, GL_LINK_STATUS, &Result);
	glGetProgramiv(ProgramID, GL_INFO_LOG_LENGTH, &InfoLogLength);
	if (!Result && InfoLogLength)
	{
		std::vector<char> ProgramErrorMessage(max(InfoLogLength, int(1)));
		glGetProgramInfoLog(ProgramID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
		Printf("%s\n", &ProgramErrorMessage[0]);
		ret = false;
	}

	glDeleteShader(VertexShaderID);
	glDeleteShader(FragmentShaderID);

	return ret;
}

bool LoadShadersEx(const char* vertex_file_path, const char* ctrl_file_path, const char* eval_file_path, const char* fragment_file_path, GLuint& ProgramID)
{
	bool ret = true;
	GLuint VertexShaderID, CtrlShaderID, EvalShaderID, FragmentShaderID;

	if (!LoadShader(vertex_file_path, GL_VERTEX_SHADER, VertexShaderID))
		return false;

	if (!LoadShader(ctrl_file_path, GL_TESS_CONTROL_SHADER, CtrlShaderID))
		return false;

	if (!LoadShader(eval_file_path, GL_TESS_EVALUATION_SHADER, EvalShaderID))
		return false;

	if (!LoadShader(fragment_file_path, GL_FRAGMENT_SHADER, FragmentShaderID))
		return false;

	// Link the program
	Printf("Linking program\n");
	ProgramID = glCreateProgram();
	glAttachShader(ProgramID, VertexShaderID);
	glAttachShader(ProgramID, CtrlShaderID);
	glAttachShader(ProgramID, EvalShaderID);
	glAttachShader(ProgramID, FragmentShaderID);
	glLinkProgram(ProgramID);

	// Check the program
	GLint Result = GL_FALSE;
	int InfoLogLength;
	glGetProgramiv(ProgramID, GL_LINK_STATUS, &Result);
	glGetProgramiv(ProgramID, GL_INFO_LOG_LENGTH, &InfoLogLength);
	if (!Result && InfoLogLength)
	{
		std::vector<char> ProgramErrorMessage(max(InfoLogLength, int(1)));
		glGetProgramInfoLog(ProgramID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
		Printf("%s\n", &ProgramErrorMessage[0]);
		ret = false;
	}

	glDeleteShader(VertexShaderID);
	glDeleteShader(FragmentShaderID);

	return ret;
}

bool OpenGLSData::LoadNormalProgram()
{
	if (!LoadShaders("../Shaders/shader_phong_vertex.glsl", "../Shaders/shader_phong_fragment.glsl", program.programID))
			return false;

	// Get a handle for our uniforms; TODO: check each of them
	program.MatrixID = glGetUniformLocation(program.programID, "view_proj_matrix");
	program.shadowVPMatrixID = glGetUniformLocation(program.programID, "vp_shadow_matrix");
	program.ModelMatrixID = glGetUniformLocation(program.programID, "model_matrix");
	program.eyePosID = glGetUniformLocation(program.programID, "eye_position");
	program.LightPosID = glGetUniformLocation(program.programID, "light_position");
	program.LightDirID = glGetUniformLocation(program.programID, "light_dir");
	program.diffColorID = glGetUniformLocation(program.programID, "material_kd");
	program.specColorID = glGetUniformLocation(program.programID, "material_ks");
	program.glossID = glGetUniformLocation(program.programID, "material_shininess");
	program.flipID = glGetUniformLocation(program.programID, "flip");
	program.flagsID = glGetUniformLocation(program.programID, "flags");

	return true;
}

bool OpenGLSData::LoadWireframeProgram()
{
	if (!LoadShadersEx("../Shaders/shader_phong_vertex.glsl", "../Shaders/shader_phong_geometry.glsl", "../Shaders/shader_phong_fragment.glsl", wireframeProgram.programID))
		return false;

	// Get a handle for our uniforms; TODO: check each of them
	wireframeProgram.MatrixID = glGetUniformLocation(wireframeProgram.programID, "view_proj_matrix");
	wireframeProgram.shadowVPMatrixID = glGetUniformLocation(wireframeProgram.programID, "vp_shadow_matrix");
	wireframeProgram.ModelMatrixID = glGetUniformLocation(wireframeProgram.programID, "model_matrix");
	wireframeProgram.eyePosID = glGetUniformLocation(wireframeProgram.programID, "eye_position");
	wireframeProgram.LightPosID = glGetUniformLocation(wireframeProgram.programID, "light_position");
	wireframeProgram.LightDirID = glGetUniformLocation(program.programID, "light_dir");
	wireframeProgram.diffColorID = glGetUniformLocation(wireframeProgram.programID, "material_kd");
	wireframeProgram.specColorID = glGetUniformLocation(wireframeProgram.programID, "material_ks");
	wireframeProgram.glossID = glGetUniformLocation(wireframeProgram.programID, "material_shininess");
	wireframeProgram.flipID = glGetUniformLocation(wireframeProgram.programID, "flip");
	wireframeProgram.flagsID = glGetUniformLocation(wireframeProgram.programID, "flags");
	wireframeProgram.winScaleId = glGetUniformLocation(wireframeProgram.programID, "WIN_SCALE");

	return true;
}

bool OpenGLSData::LoadPickProgram()
{
	if (!LoadShaders("../Shaders/shader_pick_vertex.glsl", "../Shaders/shader_pick_fragment.glsl", pickProgram.programID))
		return false;

	// Get a handle for our uniforms; TODO: check each of them
	pickProgram.MatrixID = glGetUniformLocation(pickProgram.programID, "view_proj_matrix");
	pickProgram.ModelMatrixID = glGetUniformLocation(pickProgram.programID, "model_matrix");

	return true;
}


bool OpenGLSData::Init()
{
	mMesh.Create();
	glGenBuffers(1, &vertexbuffer);
	glGenBuffers(1, &normalbuffer);
	glGenBuffers(1, &colorbuffer);
	glGenBuffers(1, &uvbuffer);
	glGenBuffers(1, &linesBuffer);
	glGenBuffers(1, &elementbuffer);

	if (!LoadNormalProgram())
		return false;

	if (!LoadWireframeProgram())
		return false;

	if (!LoadPickProgram())
		return false;

	currProgram = nullptr;

	CreateSphere(20, 20);
	CreatePlane();
	CreateCapsule();

	// Set up shadows
	InitShadowMap();

	// Load shadow shaders
	if (!LoadShaders("../Shaders/shader_shadow_vertex.glsl", "../Shaders/shader_shadow_fragment.glsl", shadowProgramID))
		return false;

	shadowModelID = glGetUniformLocation(shadowProgramID, "model");

	float ext = 100.f;
	projShadow = Matrix4::Ortographic(-ext, ext, ext, -ext, -1000.f, 1000.f);

	CheckGlError();

	return true;
}

void OpenGLSData::Resize(int width, int height)
{
	mWidth = width;
	mHeight = height;

	// Set up screen capture
	if (!InitScreenMap())
		Printf("Failed to create screen FBO.\n");

	// Set up picking
	if (!InitPickMap())
		Printf("Failed to create pick FBO.\n");
}

void OpenGLSData::PrepareForDraw(const Vector3& lightPos, const Vector3& lightDir, const Vector3& eye)
{
	mDrawIndex = 0;
	isShadowPass = false;
	const Matrix4 VP = Projection * View;

	glUseProgram(program.programID);

	// set uniform values
	glUniformMatrix4fv(program.MatrixID, 1, GL_FALSE, VP.GetData());
	glUniform3f(program.LightPosID, lightPos.X(), lightPos.Y(), lightPos.Z());
	glUniform3f(program.LightDirID, lightDir.X(), lightDir.Y(), lightDir.Z());
	glUniform1i(program.glossID, 15);
	glUniform1i(program.flipID, 1);
	glUniform1i(program.flagsID, flags);
	glUniform3f(program.diffColorID, 1, 1, 1);
	glUniform3f(program.specColorID, 0, 0, 0);
	glUniform3f(program.eyePosID, eye.X(), eye.Y(), eye.Z());

	glUseProgram(wireframeProgram.programID);

	glUniformMatrix4fv(wireframeProgram.MatrixID, 1, GL_FALSE, VP.GetData());
	glUniform3f(wireframeProgram.LightPosID, lightPos.X(), lightPos.Y(), lightPos.Z());
	glUniform1i(wireframeProgram.glossID, 15);
	glUniform1i(wireframeProgram.flipID, 1);
	glUniform1i(wireframeProgram.flagsID, flags);
	glUniform2f(wireframeProgram.winScaleId, mWidth * 0.5f, mHeight * 0.5f);
	glUniform3f(wireframeProgram.diffColorID, 1, 1, 1);
	glUniform3f(wireframeProgram.specColorID, 0, 0, 0);
	glUniform3f(wireframeProgram.eyePosID, eye.X(), eye.Y(), eye.Z());

	glUseProgram(pickProgram.programID);
	glUniformMatrix4fv(pickProgram.MatrixID, 1, GL_FALSE, VP.GetData());

	SetProgram(false);

	glCullFace(GL_BACK);

	CheckGlError();
}

void OpenGLSData::SetFlipNormals(bool val)
{
	if (!isShadowPass && !isPickPass)// TODO: use isPickPass everywhere
		glUniform1i(currProgram->flipID, val ? -1 : 1);
}

void OpenGLSData::SetFlags(int val)
{
	if (!isShadowPass && currProgram != &pickProgram)
	{
		flags = val;
		glUniform1i(currProgram->flagsID, flags);
	}
}

void OpenGLSData::ResetFlags()
{
	if (!isShadowPass && currProgram != &pickProgram)
	{
		flags = shadowsActive ? ShaderFlags::SHADOWS : ShaderFlags::NONE;
		glUniform1i(currProgram->flagsID, flags);
	}
}


void OpenGLSData::PrepareForShadow()
{
	isShadowPass = true;

	// set the shadow program
	glUseProgram(shadowProgramID);

	// TODO: store uniform locations; why separate matrices?
	glUniformMatrix4fv(glGetUniformLocation(shadowProgramID, "projShadow"), 1, GL_FALSE, projShadow.GetData());
	glUniformMatrix4fv(glGetUniformLocation(shadowProgramID, "viewShadow"), 1, GL_FALSE, viewShadow.GetData());

	// set the render target 
	glBindFramebuffer(GL_FRAMEBUFFER, shadowFramebuffer);
	glViewport(0, 0, 1024, 1024); // TODO: store texture size
	//glClearDepth(1.0f); // why not 0?
	glClear(GL_DEPTH_BUFFER_BIT);

	// this creates an artefact, is it really needed?
	// however, if I disable it for the bunny it leaves holes in the shadow
	glCullFace(GL_FRONT);

	CheckGlError();
}

void OpenGLSData::DrawAxes()
{
	Matrix4 Model = Matrix4::Identity();
	glUniformMatrix4fv(currProgram->ModelMatrixID, 1, GL_FALSE, Model.GetData());
	Matrix4 MVP = Projection * View * Model;
	glUniformMatrix4fv(currProgram->MatrixID, 1, GL_FALSE, MVP.GetData());

	float l = 20.f;
	float lines[] = { 0, 0, 0, l, 0, 0, 0, 0, 0, 0, l, 0, 0, 0, 0, 0, 0, l };

	glBindBuffer(GL_ARRAY_BUFFER, linesBuffer);
	glBufferData(GL_ARRAY_BUFFER, 6 * sizeof(Vector3), &lines[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

	// Draw the triangles !
	glDrawArrays(GL_LINES, 0, 6);

	glDisableVertexAttribArray(0);
	CheckGlError();
}

void OpenGLSData::DrawPlane(const Vector3& pos, float scale, Texture* tex)
{
	Matrix4 Model = Matrix4::Translation(pos.X(), pos.Y(), pos.Z()) * Matrix4::Scale(scale, scale, scale);
	mPlane.SetTransform(&Model);
	mPlane.SetTexture(tex);
	DrawMesh(mPlane);
	mPlane.SetTexture(nullptr);
}


int OpenGLSData::DrawSphere(const Vector3& pos, float radius)
{
	Matrix4 Model = Matrix4::Translation(pos.X(), pos.Y(), pos.Z()) * Matrix4::Scale(radius, radius, radius);
	mSphere.SetTransform(&Model);
	return DrawMesh(mSphere);
}

void OpenGLSData::DrawSphereTex(const Vector3& pos, float radius, const Matrix3& rot, Texture* tex)
{
	Matrix4 Model = Matrix4(rot, pos) * Matrix4::Scale(radius, radius, radius);
	mSphere.SetTransform(&Model);
	mSphere.SetTexture(tex);
	DrawMesh(mSphere);
	mSphere.SetTexture(nullptr);
}

void OpenGLSData::DrawTriangle(const Vector3& a, const Vector3& b, const Vector3& c, bool fill)
{
	// TODO: banish 'fill', use render mode instead
	if (!fill)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); // for wireframe
	Matrix4 Model = Matrix4::Identity();
	glUniformMatrix4fv(currProgram->ModelMatrixID, 1, GL_FALSE, Model.GetData());

	GLfloat buffer[9];
	for (int i = 0; i < 3; i++)
	{
		buffer[i] = a[i];
		buffer[i + 3] = b[i];
		buffer[i + 6] = c[i];
	}
	Vector3 n = cross(b - a, c - a);
	n.Normalize();
	GLfloat normals[] = {
		n.X(), n.Y(), n.Z(),
		n.X(), n.Y(), n.Z(),
		n.X(), n.Y(), n.Z(),
	};

	// The following commands will talk about our 'vertexbuffer' buffer
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	// Give our vertices to OpenGL.
	glBufferData(GL_ARRAY_BUFFER, sizeof(buffer), buffer, GL_STATIC_DRAW);
	// 1rst attribute buffer : vertices
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(
		0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
	);

	glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(normals), normals, GL_STATIC_DRAW);
	// 2nd attribute buffer : normals
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(
		1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
		3,                                // size
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
	);

	// Draw the triangle !
	glDrawArrays(GL_TRIANGLES, 0, 3); // Starting from vertex 0; 3 vertices total -> 1 triangle

	//glDisableVertexAttribArray(0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	CheckGlError();
}

void OpenGLSData::DrawCube(const Vector3& pos, const Vector3& ext, const Matrix3& rot)
{
	Matrix4 Model = Matrix4(rot, pos) * Matrix4::Scale(0.5f * ext.X(), 0.5f * ext.Y(), 0.5f * ext.Z());
	GL_CALL(glUniformMatrix4fv(isShadowPass ? shadowModelID : currProgram->ModelMatrixID, 1, GL_FALSE, Model.GetData()));

	std::vector<Vector3> buffer;
	std::vector<Vector3> normals;
	for (int i = 0; i < 3; i++) // the current axis
	{
		for (int j = 0; j < 2; j++) // the direction of the axis
		{
			for (int k = 0; k < 2; k++) // the current triangle
			{
				Vector3 a, b, c;
				const int sgnDir = -1 + 2 * j;
				const int sgnTr = -1 + 2 * k;
				Vector3 n;
				a[i] = sgnDir; b[i] = sgnDir; c[i] = sgnDir;
				n[i] = sgnDir;
				int axis1 = (i + 1) % 3;
				int axis2 = (i + 2) % 3;
				if (sgnDir < 0)
				{
					a[axis1] = -1; a[axis2] = -1;
					b[axis1] = -sgnTr; b[axis2] = 1;
					c[axis1] = 1; c[axis2] = sgnTr;
				}
				else
				{
					a[axis1] = -1; a[axis2] = -1;
					b[axis1] = 1; b[axis2] = sgnTr;
					c[axis1] = -sgnTr; c[axis2] = 1;
				}

				buffer.push_back(a);
				buffer.push_back(b);
				buffer.push_back(c);

				normals.push_back(n);
				normals.push_back(n);
				normals.push_back(n);
			}
		}
	}

	// The following commands will talk about our 'vertexbuffer' buffer
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	// Give our vertices to OpenGL.
	glBufferData(GL_ARRAY_BUFFER, buffer.size() * sizeof(Vector3), &buffer[0], GL_STATIC_DRAW);

	// 1rst attribute buffer : vertices
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(
		0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
	);

	glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
	glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(Vector3), &normals[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(
		1,                                // attribute
		3,                                // size
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
	);

	// Draw the triangle !
	glDrawArrays(GL_TRIANGLES, 0, 12 * 3); // Starting from vertex 0; 3 vertices total -> 1 triangle

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);

	CheckGlError();
}

void OpenGLSData::DrawCapsule(const Vector3& pos, float radius, float height, const Matrix3& rot)
{
	Matrix4 Model = Matrix4(rot, pos);
	CreateCapsule(10, 5, radius, height);
	mCapsule.SetTransform(&Model);
	DrawMesh(mCapsule);
}

void OpenGLSData::DrawLine(const Vector3& a, const Vector3& b)
{
	Matrix4 Model = Matrix4::Identity();
	glUniformMatrix4fv(!isShadowPass ? currProgram->ModelMatrixID : shadowModelID, 1, GL_FALSE, Model.GetData());

	float l = 20.f;
	float lines[] = { a.X(), a.Y(), a.Z(), b.X(), b.Y(), b.Z() };

	glBindBuffer(GL_ARRAY_BUFFER, linesBuffer);
	glBufferData(GL_ARRAY_BUFFER, 2 * sizeof(Vector3), &lines[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	CheckGlError();

	glDrawArrays(GL_LINES, 0, 2);
	CheckGlError();

	glDisableVertexAttribArray(0);
	CheckGlError();
}

void OpenGLSData::DrawLines(std::vector<Vector3> points)
{
	Matrix4 Model = Matrix4::Identity();
	glUniformMatrix4fv(!isShadowPass ? currProgram->ModelMatrixID : shadowModelID, 1, GL_FALSE, Model.GetData());

	glBindBuffer(GL_ARRAY_BUFFER, linesBuffer);
	glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(Vector3), points.data(), GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
	CheckGlError();

	glDrawArrays(GL_LINES, 0, (GLsizei)points.size());
	CheckGlError();

	glDisableVertexAttribArray(0);
	CheckGlError();
}

int OpenGLSData::DrawMesh(const RenderMesh& mesh, bool useVAO, bool hasColors)
{
	int mapID = -1;
	int oldFlags = flags;

	if (currProgram == &pickProgram)
	{
		int loc = glGetUniformLocation(pickProgram.programID, "gDrawIndex"); // FIXME
		mDrawIndex++;
		glUniform1i(loc, mDrawIndex);
	}
	else if (!isShadowPass && mesh.mTexture != nullptr)
	{
		mapID = glGetUniformLocation(currProgram->programID, "map_color"); // FIXME
		SetFlags(flags | ShaderFlags::TEXTURED);
	}

	if (hasColors)
		SetFlags(flags | ShaderFlags::VERTEX_COLORS);

	mesh.Draw(!isShadowPass ? currProgram->ModelMatrixID : shadowModelID, mapID, useVAO, hasColors);
	
	SetFlags(oldFlags);

	return mDrawIndex;
}

int OpenGLSData::DrawMesh(const std::vector<Vector3>& vertices, const std::vector<Vector3>& normals, const std::vector<uint32>& indices, const Matrix4* model)
{
	if (vertices.empty() || normals.empty() || indices.empty())
		return -1; // TODO: error codes

	mMesh.SetVertexData(vertices);
	mMesh.SetNormalData(normals);
	mMesh.SetIndexData(indices);
	mMesh.SetTransform(model);

	return DrawMesh(mMesh, false, false);
}

int OpenGLSData::DrawMesh(const std::vector<Vector3>& vertices, const std::vector<Vector3>& normals, const std::vector<Vector3>& colors, const std::vector<uint32>& indices, const Matrix4* model)
{
	if (vertices.empty() || normals.empty() || indices.empty())
		return -1;

	mMesh.SetVertexData(vertices);
	mMesh.SetNormalData(normals);
	mMesh.SetIndexData(indices);
	mMesh.SetColorData(colors);
	mMesh.SetTransform(model);

	return DrawMesh(mMesh, false, true);
}

int OpenGLSData::DrawMesh(const std::vector<Vector3>& vertices, const std::vector<Vector3>& normals, const std::vector<uint32>& indices, const std::vector<Vector2>& uvs, Texture* tex, const Matrix4* model)
{
	if (vertices.empty() || normals.empty())
		return -1;

	mMesh.SetVertexData(vertices);
	mMesh.SetNormalData(normals);
	mMesh.SetIndexData(indices);
	if (uvs.size() != 0)
	{
		mMesh.SetUVData(uvs);
		mMesh.SetTexture(tex);
	}
	mMesh.SetTransform(model);

	return DrawMesh(mMesh, false, false);
}

void OpenGLSData::DrawTetrahedronEdgeAsBBCurve(const Vector3& c1, const Vector3& c2, const size_t nodesperedge, const Vector3* edge)
{
	std::vector<Vector3> nodes; nodes.resize(2 + nodesperedge);
	nodes[0] = c1;
	for (size_t i = 0; i < nodesperedge; i++)
	{
		nodes[1 + i] = edge[i];
	}
	nodes[nodes.size() - 1] = c2;

	auto binom = [](int n, int k) -> int
	{
		int val = 1;

		// Utilize that C(n, k) = C(n, n-k) 
		if (k > n - k)
			k = n - k;

		for (int i = 0; i < k; ++i)
		{
			val *= (n - i);
			val /= (i + 1);
		}

		return val;
	};

	auto evalBB = [&binom](float t, size_t noNodes, const Vector3* nodes) -> Vector3
	{
		// t \in [0,1]
		// order = noNodes - 1
		int order = (int)(noNodes - 1);
		Vector3 val(0, 0, 0);
		for (size_t i = 0; i < noNodes; i++)
		{
			val += nodes[i] * float(binom(order, (int)i)) * std::pow((1.f - t), order - i) * std::pow(t, i);
		}
		return val;
	};

	const size_t subdivisions = 10; // ie how many line segments, min(1)
	std::vector<Vector3> vtx;
	vtx.resize(subdivisions * 2);
	for (size_t s = 0; s < subdivisions; s++)
	{
		vtx[s * 2] = evalBB(float(s) / float(subdivisions), nodes.size(), &nodes[0]);
		if (s > 0) vtx[(s * 2) - 1] = vtx[s * 2];

		if ((s + 1) == subdivisions)
		{
			vtx[(s * 2) + 1] = evalBB(1.f, nodes.size(), &nodes[0]);
		}
	}

	// Draw the line buffer
	Matrix4 Model = Matrix4::Identity();
	glUniformMatrix4fv(!isShadowPass ? currProgram->ModelMatrixID : shadowModelID, 1, GL_FALSE, Model.GetData());

	glBindBuffer(GL_ARRAY_BUFFER, linesBuffer);
	glBufferData(GL_ARRAY_BUFFER, vtx.size() * sizeof(Vector3), &vtx[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

	// Draw the triangles !
	glDrawArrays(GL_LINES, 0, (GLsizei)vtx.size());

	glDisableVertexAttribArray(0);
	CheckGlError();
}

void OpenGLSData::DrawTetrahedron(const Vector3& v0, const Vector3& v1, const Vector3& v2, const Vector3& v3)
{
	Vector3 vtx[12];
	vtx[0] = v0;
	vtx[1] = v1;
	vtx[2] = v1;
	vtx[3] = v2;
	vtx[4] = v2;
	vtx[5] = v0;
	vtx[6] = v3;
	vtx[7] = v0;
	vtx[8] = v3;
	vtx[9] = v1;
	vtx[10] = v3;
	vtx[11] = v2;

	Matrix4 Model = Matrix4::Identity();
	glUniformMatrix4fv(!isShadowPass ? currProgram->ModelMatrixID : shadowModelID, 1, GL_FALSE, Model.GetData());

	glBindBuffer(GL_ARRAY_BUFFER, linesBuffer);
	glBufferData(GL_ARRAY_BUFFER, 12 * sizeof(Vector3), &vtx[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

	// Draw the triangles !
	glDrawArrays(GL_LINES, 0, 12);

	glDisableVertexAttribArray(0);
	CheckGlError();
}

bool OpenGLSData::InitShadowMap()
{
	// The framebuffer, which regroups 0, 1, or more textures, and 0 or 1 depth buffer.	
	glGenFramebuffers(1, &shadowFramebuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, shadowFramebuffer);

	// Depth texture. Slower than a depth buffer, but you can sample it later in your shader
	GLuint depthTexture;
	glGenTextures(1, &depthTexture);
	glBindTexture(GL_TEXTURE_2D, depthTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT16, 1024, 1024, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	// this is needed for sampler2DShadow
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);

#if (RENDERER == OPENGL)
	shadowMap.ID = depthTexture;
	shadowMap.width = 1024;
	shadowMap.height = 1024;
#endif

	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depthTexture, 0);

	glDrawBuffer(GL_NONE); // No color buffer is drawn to.
	glReadBuffer(GL_NONE);

	// Always check that our framebuffer is ok
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		return false;

	GL_CALL(glBindFramebuffer(GL_FRAMEBUFFER, 0));

	return true;
}

void OpenGLSData::SetColor(float r, float g, float b)
{
	if (isShadowPass)
		return;
	if (currProgram != &pickProgram)
		glUniform3f(currProgram->diffColorID, r, g, b);
}

void OpenGLSData::SetProgram(bool wireframe)
{
	if (isShadowPass)
		return;
	currProgram = wireframe ? &wireframeProgram : &program;
	glUseProgram(currProgram->programID);
	if (wireframe)
		SetFlags(ShaderFlags::WIREFRAME);

	// shadow matrix
	// TODO: use bias matrix	
	const Matrix4 shadowVP = projShadow * viewShadow;
	glUniformMatrix4fv(currProgram->shadowVPMatrixID, 1, GL_FALSE, shadowVP.GetData());

	// FIXME - this take up to 5 ms!!!
	// shadow map
	//glActiveTexture(GL_TEXTURE1);
	//glBindTexture(GL_TEXTURE_2D, shadowMap.ID);
	//GLint shadowMapID = glGetUniformLocation(currProgram->programID, "shadowMap"); // TODO: pre-store
	//glUniform1i(shadowMapID, 1);

#ifdef SHADOWS_PCF
	// this is needed for sampler2DShadow
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);
#endif
}

void OpenGLSData::UsePickProgram()
{
	currProgram = &pickProgram;
	glUseProgram(pickProgram.programID);
}

void OpenGLSData::CreateSphere(int divsTheta, int divsPhi)
{
	std::vector<Vector3> vertices;
	std::vector<Vector3> normals;
	std::vector<unsigned int> indices; // TODO: uint16 or less

	float theta = 0;
	const float dt = 2 * PI / divsTheta;
	const float dp = PI / (divsPhi - 1);
	std::vector<Vector2> uvs;

	for (int i = 0; i < divsTheta; i++)
	{
		float phi = -PI / 2;
		for (int j = 0; j < divsPhi; j++)
		{
			Vector3 v1(cos(theta) * cos(phi), sin(phi), sin(theta) * cos(phi)); // TODO: sin/cos of sum?
			uvs.push_back(Vector2(theta / (2 * PI), (phi + PI / 2) / PI));
			phi += dp;
			vertices.push_back(v1);
			v1.Normalize();
			normals.push_back(v1);
		}
		theta += dt;
	}

	for (int i = 0; i < divsTheta; i++)
	{
		for (int j = 0; j < divsPhi - 1; j++)
		{
			int ni = (i + 1) % divsTheta;
			int nj = (j + 1) % divsPhi;
			int i1 = (i * divsPhi + j);
			int i2 = (ni * divsPhi + j);
			int i3 = (i * divsPhi + nj);
			int i4 = (ni * divsPhi + nj);
			indices.push_back(i1);
			indices.push_back(i3);
			indices.push_back(i2);
			indices.push_back(i2);
			indices.push_back(i3);
			indices.push_back(i4);
		}
	}

	mSphere.Create();
	mSphere.SetVertexData(vertices);
	mSphere.SetNormalData(normals);
	mSphere.SetUVData(uvs);
	mSphere.SetIndexData(indices);
	mSphere.CreateVAO(false, true);
}

void OpenGLSData::CreatePlane(int divsX, int divsY)
{
	std::vector<Vector3> vertices;
	std::vector<Vector3> normals;
	std::vector<unsigned int> indices; // TODO: uint16 or less

	std::vector<Vector2> uvs;

	Vector3 normal(0, 1, 0);
	for (int i = 0; i < divsX; i++)
	{
		float x = (float)i / (float)divsX;
		for (int j = 0; j < divsY; j++)
		{
			float y = (float)j / (float)divsY;
			Vector3 v1(x - 0.5f, 0, y - 0.5f);
			uvs.push_back(Vector2(i, j));
			vertices.push_back(v1);
			normals.push_back(normal);
		}
	}

	for (int i = 0; i < divsX; i++)
	{
		for (int j = 0; j < divsY - 1; j++)
		{
			int ni = (i + 1);
			int nj = (j + 1);
			int i1 = (i * divsX + j);
			int i2 = (ni * divsY + j);
			int i3 = (i * divsX + nj);
			int i4 = (ni * divsY + nj);
			indices.push_back(i1);
			indices.push_back(i3);
			indices.push_back(i2);
			indices.push_back(i2);
			indices.push_back(i3);
			indices.push_back(i4);
		}
	}

	mPlane.Create();
	mPlane.SetVertexData(vertices);
	mPlane.SetNormalData(normals);
	mPlane.SetUVData(uvs);
	mPlane.SetIndexData(indices);
	mPlane.CreateVAO(false, true);
}

void OpenGLSData::CreateCapsule(int divsTheta, int divsPhi, float radius, float height)
{
	float theta = 0;
	const float dt = 2 * PI / divsTheta;
	const float dp = PI / (divsPhi - 1) / 2;
	std::vector<Vector2> uvs;
	std::vector<Vector3> vertices;
	std::vector<Vector3> normals;
	std::vector<uint32> indices;

	for (int i = 0; i < divsTheta; i++)
	{
		float phi = 0;//PI / 2;
		for (int j = 0; j < divsPhi; j++)
		{
			Vector3 v1(cos(theta) * cos(phi), sin(phi), sin(theta) * cos(phi)); // TODO: sin/cos of sum?
			uvs.push_back(Vector2(theta / (2 * PI), (phi + PI / 2) / PI));
			phi += dp;
			vertices.push_back(v1 * radius + Vector3(0, height, 0));
			v1.Normalize();
			normals.push_back(v1);
		}
		theta += dt;
	}
	size_t half = vertices.size();

	theta = 0;
	for (int i = 0; i < divsTheta; i++)
	{
		float phi = 0;//-PI / 2;
		for (int j = 0; j < divsPhi; j++)
		{
			Vector3 v1(cos(theta) * cos(phi), sin(phi), sin(theta) * cos(phi)); // TODO: sin/cos of sum?
			uvs.push_back(Vector2(theta / (2 * PI), (phi + PI / 2) / PI));
			phi -= dp;
			vertices.push_back(v1 * radius - Vector3(0, height, 0));
			v1.Normalize();
			normals.push_back(v1);
		}
		theta += dt;
	}

	for (int i = 0; i < divsTheta; i++)
	{
		for (int j = 0; j < divsPhi - 1; j++)
		{
			int ni = (i + 1) % divsTheta;
			int nj = (j + 1) % divsPhi;
			int i1 = (i * divsPhi + j);
			int i2 = (ni * divsPhi + j);
			int i3 = (i * divsPhi + nj);
			int i4 = (ni * divsPhi + nj);
			indices.push_back(i1);
			indices.push_back(i3);
			indices.push_back(i2);
			indices.push_back(i2);
			indices.push_back(i3);
			indices.push_back(i4);

			indices.push_back((uint32)(i1 + half));
			indices.push_back((uint32)(i2 + half));
			indices.push_back((uint32)(i3 + half));

			indices.push_back((uint32)(i2 + half));
			indices.push_back((uint32)(i4 + half));
			indices.push_back((uint32)(i3 + half));
		}
	}

	for (int i = 0; i < divsTheta; i++)
	{
		int ni = (i + 1) % divsTheta;
		indices.push_back((uint32)(i * divsPhi));
		indices.push_back((uint32)(ni * divsPhi));
		indices.push_back((uint32)(i * divsPhi + half));

		indices.push_back((uint32)(ni * divsPhi + half));
		indices.push_back((uint32)(i * divsPhi + half));
		indices.push_back((uint32)(ni * divsPhi));
	}

	mCapsule.Create();
	mCapsule.SetVertexData(vertices);
	mCapsule.SetNormalData(normals);
	mCapsule.SetUVData(uvs);
	mCapsule.SetIndexData(indices);
	mCapsule.CreateVAO(false, true);
}

void RenderMesh::Create()
{
	if (!glIsBuffer(mVertexVBO))
		glGenBuffers(1, &mVertexVBO);
	if (!glIsBuffer(mNormalVBO))
		glGenBuffers(1, &mNormalVBO);
	if (!glIsBuffer(mColorVBO))
		glGenBuffers(1, &mColorVBO); // optional
	if (!glIsBuffer(mUvVBO))
		glGenBuffers(1, &mUvVBO); // optional
	if (!glIsBuffer(mIndexVBO))
		glGenBuffers(1, &mIndexVBO);
}

void RenderMesh::Destroy()
{
	if (glIsBuffer(mIndexVBO))
		glDeleteBuffers(1, &mIndexVBO);
	if (glIsBuffer(mUvVBO))
		glDeleteBuffers(1, &mUvVBO);
	if (glIsBuffer(mColorVBO))
		glDeleteBuffers(1, &mColorVBO);
	if (glIsBuffer(mNormalVBO))
		glDeleteBuffers(1, &mNormalVBO);
	if (glIsBuffer(mVertexVBO))
		glDeleteBuffers(1, &mVertexVBO);
}

void RenderMesh::SetVertexData(const std::vector<Vector3>& vertices)
{
	glBindBuffer(GL_ARRAY_BUFFER, mVertexVBO);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vector3), vertices.data(), GL_STATIC_DRAW);
}

void RenderMesh::SetNormalData(const std::vector<Vector3>& normals)
{
	ASSERT(!normals.empty());
	glBindBuffer(GL_ARRAY_BUFFER, mNormalVBO);
	glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(Vector3), normals.data(), GL_STATIC_DRAW);
}

void RenderMesh::SetColorData(const std::vector<Vector3>& colors)
{
	glBindBuffer(GL_ARRAY_BUFFER, mColorVBO);
	glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(Vector3), &colors[0], GL_STATIC_DRAW);
}

void RenderMesh::SetUVData(const std::vector<Vector2>& uvs)
{
	glBindBuffer(GL_ARRAY_BUFFER, mUvVBO);
	glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(Vector2), &uvs[0], GL_STATIC_DRAW);
}

void RenderMesh::SetIndexData(const std::vector<uint32>& indices)
{
	//mElementBuffer = (void*)&indices[0];
	//mElementBufferSize = indices.size() * sizeof(uint32);
	mNumElements = indices.size();

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexVBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(uint32), &indices[0], GL_STATIC_DRAW);
}

void RenderMesh::PrepareVBOAttributes(bool hasColors, bool hasUVs) const
{
	glBindBuffer(GL_ARRAY_BUFFER, mVertexVBO);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vector3), (void*)0);

	glBindBuffer(GL_ARRAY_BUFFER, mNormalVBO);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vector3), 0);

	if (hasColors)
	{
		glBindBuffer(GL_ARRAY_BUFFER, mColorVBO);
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vector3), 0);
	}

	if (hasUVs)
	{
		glBindBuffer(GL_ARRAY_BUFFER, mUvVBO);
		glEnableVertexAttribArray(3);
		glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(Vector2), 0);
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIndexVBO);
}

void RenderMesh::CreateVAO(bool hasColors, bool hasUVs)
{
	// create the vertex array object (VAO)
	glGenVertexArrays(1, &mVAO);
	glBindVertexArray(mVAO);

	PrepareVBOAttributes(hasColors, hasUVs);

	glBindVertexArray(0);
}

void RenderMesh::Draw(int modelID, int mapID, bool useVAO, bool hasColors) const
{
	if (mVAO == 0)
		useVAO = false;

	bool hasTexture = mTexture != nullptr && mapID >= 0;

	if (useVAO)
		glBindVertexArray(mVAO);
	else
		PrepareVBOAttributes(hasColors, hasTexture);

	Matrix4 Model;
	if (mModel != nullptr)
		Model = *mModel;
	else
		Model = Matrix4::Identity();
	glUniformMatrix4fv(modelID, 1, GL_FALSE, Model.GetData());

	if (hasTexture)
	{
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, mTexture->ID);
		glUniform1i(mapID, 0);
	}
	else
	{
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	glDrawElements(GL_TRIANGLES, (GLsizei)mNumElements, GL_UNSIGNED_INT, (void*)0);

	if (useVAO)
		glBindVertexArray(0);

	CheckGlError();
}

bool OpenGLSData::InitScreenMap()
{
	if (!mScreenFBOActive)
		return true;

	glGenFramebuffers(1, &mScreenFBO);
	glBindFramebuffer(GL_FRAMEBUFFER, mScreenFBO);

	mScreenMap.Create(mWidth, mHeight);

	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, mScreenMap.ID, 0);

	bool ret = true;
	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE)
	{
		ret = false;
		if (status == GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT)
			Printf("FBO incomplete attachment\n");
		else if (status == GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT)
			Printf("FBO missing attachment\n");
		else if (status == GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER)
			Printf("FBO incomplete draw buffer\n");
		else if (status == GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER)
			Printf("FBO incomplete read buffer\n");		
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	CheckGlError();

	return ret;
}

bool OpenGLSData::InitPickMap()
{
	if (!mPickFBOActive)
		return true;

	if (mPickFBO <= 0)
		glGenFramebuffers(1, &mPickFBO);
	glBindFramebuffer(GL_FRAMEBUFFER, mPickFBO);

	mPickMap.Create(mWidth, mHeight);

	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, mPickMap.ID, 0);

	// TODO: create depth texture in Texture

	// Depth texture. Slower than a depth buffer, but you can sample it later in your shader
	if (depthTexture > 0)
	{
		glDeleteTextures(1, &depthTexture);
		depthTexture = 0;
	}
	glGenTextures(1, &depthTexture);
	glBindTexture(GL_TEXTURE_2D, depthTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT16, mWidth, mHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);

	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthTexture, 0);

	// Disable reading to avoid problems with older GPUs
	glReadBuffer(GL_NONE);

	glDrawBuffer(GL_COLOR_ATTACHMENT0);

	bool ret = true;
	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE)
	{
		ret = false;
		if (status == GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT)
			Printf("FBO incomplete attachment\n");
		else if (status == GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT)
			Printf("FBO missing attachment\n");
		else if (status == GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER)
			Printf("FBO incomplete draw buffer\n");
		else if (status == GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER)
			Printf("FBO incomplete read buffer\n");
	}

	glBindTexture(GL_TEXTURE_2D, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	CheckGlError();

	return ret;
}

void OpenGLSData::DrawToFBO()
{
	glBindFramebuffer(GL_FRAMEBUFFER, mScreenFBO);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	Engine::getInstance()->OnDraw3D();

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void OpenGLSData::DrawToPickFBO()
{
	if (!mPickFBOActive)
		return;

	isPickPass = true;
	glBindFramebuffer(GL_FRAMEBUFFER, mPickFBO);
	glClearColor(0, 0, 0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	UsePickProgram();
	Engine::getInstance()->OnDraw3D();

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	isPickPass = false;
}

uint32 OpenGLSData::ReadPickPixel(unsigned int x, unsigned int y)
{

	glBindFramebuffer(GL_READ_FRAMEBUFFER, mPickFBO);
	glReadBuffer(GL_COLOR_ATTACHMENT0);

	uint32 pixel;
	glReadPixels(x, y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, &pixel);

	glReadBuffer(GL_NONE);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

	return pixel;
}

