#include <Graphics2D\Graphics.h>

// TODO: Graphics2DGLS.inl

inline void Graphics2D::Translate(float x, float y) 
{
#if defined(FIXED_PIPELINE)
	glTranslatef(x, y, 0); 
#endif
	View = View * Matrix4::Translation(x, y, 0);
}

inline void Graphics2D::Rotate(float angle) 
{
#ifdef FIXED_PIPELINE
	glRotatef(angle, 0, 0, 1);
#endif
	View = View * Matrix4::RotationZ(RADIAN(angle));
}

inline void Graphics2D::Scale(float s, float t) 
{ 
#if defined(FIXED_PIPELINE)
	glScalef(s, t, 0); // TODO: disable for CPU zooming
#endif
	View = View * Matrix4::Scale(s, t, 1);
}

inline void Graphics2D::PushMatrix() 
{
#ifdef FIXED_PIPELINE
	glPushMatrix();
#endif
}

inline void Graphics2D::PopMatrix() 
{
#ifdef FIXED_PIPELINE
	glPopMatrix();
#endif
}

inline void Graphics2D::ResetTransform()
{
#if defined(FIXED_PIPELINE)
	glLoadIdentity();
	glTranslatef(0.f, (float)h, 0.f);
	glScalef(1, -1, 1);
#endif
	View = Matrix4::Identity();
}

inline Matrix4 Graphics2D::GetTransform() const
{
#ifdef FIXED_PIPELINE
	glPushMatrix();
	return Matrix4::Identity();
#endif
	return View;
}

inline void Graphics2D::SetTransform(const Matrix4& m)
{
#if defined(FIXED_PIPELINE)
	glPopMatrix();
#else
	View = m;
#endif
}

inline void Graphics2D::SetColor(uint32 c)
{
	int red = (c >> 16) & 0xff;
	int green = (c >> 8) & 0xff;
	int blue = c & 0xff;
	int alpha = (c >> 24) & 0xff;
#ifdef FIXED_PIPELINE
	glColor4ub(red, green, blue, alpha);
#else
	color[0] = red / 255.f;
	color[1] = green / 255.f;
	color[2] = blue / 255.f;
	color[3] = alpha / 255.f;
#endif
}

inline void Graphics2D::SetBgColor(uint32 c)
{
	bgColor[0] = ((c >> 16) & 0xff) / 255.f;
	bgColor[1] = ((c >> 8) & 0xff) / 255.f;
	bgColor[2] = (c & 0xff) / 255.f;
	//glClearColor(red, green, blue, 1.f);
}

inline void Graphics2D::SetFiltering(Texture::FilteringType val)
{
	Texture::SetFiltering(val);
}

// TODO: const Texture*
inline void Graphics2D::DrawImage(Texture* image, float x_dest, float y_dest, float scale, int anchor)
{
	DrawRegion(image, x_dest, y_dest, 0, 0, image->Width(), image->Height(), scale, anchor);
}

// TODO: add them to the class
inline void PrepareStencil()
{
	glEnable(GL_STENCIL_TEST);
	glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
	glStencilFunc(GL_NEVER, 1, 0xFF);
	glStencilOp(GL_REPLACE, GL_KEEP, GL_KEEP);  // draw 1s on test fail (always)

	// draw stencil pattern
	glStencilMask(0xFF);
	glClear(GL_STENCIL_BUFFER_BIT);  // needs mask=0xFF	

	CheckGlError();
}

inline void ActivateStencil()
{
	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
	glStencilMask(0x00);
	// draw only where stencil's value is 1
	glStencilFunc(GL_EQUAL, 1, 0xFF);

	CheckGlError();
}

inline void DisableStencil()
{
	glDisable(GL_STENCIL_TEST);	// draw images or set volumes
}

inline void Graphics2D::SetLineWidth(float val)
{
	glLineWidth(val);
	lineWidth = val;
}

inline void Graphics2D::SetLineStyle(int val) {}
