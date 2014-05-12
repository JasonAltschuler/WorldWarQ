#include "texture.h"

R3Material **materials = new R3Material*[4];

void MakeMaterials(R3Material **materials) 
{
	// ROCK
	R3Material* rock = new R3Material();
	rock->ka = R3Rgb(0.0, 0.0, 0.0, 0.0);
	rock->kd = R3Rgb(0.5, 0.5, 0.5, 0.0);
	rock->ks = R3Rgb(0.5, 0.5, 0.5, 0.0);
	rock->kt = R3Rgb(0.0, 0.0, 0.0, 0.0);
	rock->emission = R3Rgb(0, 0, 0, 0);
	rock->shininess = 10;
	rock->indexofrefraction = 1;
	rock->id = 1;
	 
	// Read texture image
	rock->texture = new R2Image();
	rock->texture->Read(rock_string);

    materials[ROCK] = rock;

    // GRASS
    R3Material* grass = new R3Material();
	grass->ka = R3Rgb(0.0, 0.0, 0.0, 0.0);
	grass->kd = R3Rgb(0.5, 0.5, 0.5, 0.0);
	grass->ks = R3Rgb(0.5, 0.5, 0.5, 0.0);
	grass->kt = R3Rgb(0.0, 0.0, 0.0, 0.0);
	grass->emission = R3Rgb(0, 0, 0, 0);
	grass->shininess = 10;
	grass->indexofrefraction = 1;
	grass->id = 2;
	 
	// Read texture image
	grass->texture = new R2Image();
	grass->texture->Read(grass_string);

    materials[GRASS] = grass;

    // SNOW
    R3Material* snow = new R3Material();
	snow->ka = R3Rgb(0.0, 0.0, 0.0, 0.0);
	snow->kd = R3Rgb(0.5, 0.5, 0.5, 0.0);
	snow->ks = R3Rgb(0.5, 0.5, 0.5, 0.0);
	snow->kt = R3Rgb(0.0, 0.0, 0.0, 0.0);
	snow->emission = R3Rgb(0, 0, 0, 0);
	snow->shininess = 10;
	snow->indexofrefraction = 1;
	snow->id = 3;
	 
	// Read texture image
	snow->texture = new R2Image();
	snow->texture->Read(snow_string);

    materials[SNOW] = snow;

	// DIRT
	R3Material* dirt = new R3Material();
	dirt->ka = R3Rgb(0.0, 0.0, 0.0, 0.0);
	dirt->kd = R3Rgb(0.5, 0.5, 0.5, 0.0);
	dirt->ks = R3Rgb(0.5, 0.5, 0.5, 0.0);
	dirt->kt = R3Rgb(0.0, 0.0, 0.0, 0.0);
	dirt->emission = R3Rgb(0, 0, 0, 0);
	dirt->shininess = 10;
	dirt->indexofrefraction = 1;	
	dirt->id = 4;
	 
	// Read texture image
	dirt->texture = new R2Image();
	dirt->texture->Read(dirt_string);

    materials[DIRT] = dirt;
}











void LoadMaterial(R3Material *material) 
{
	GLfloat c[4];

	// Check if same as current
	static R3Material *cached = NULL;

	if (material == cached) 
    return;
	cached = material;

	// Compute "opacity"
	double opacity = 1 - material->kt.Luminance();

	// Load ambient
	c[0] = material->ka[0];
	c[1] = material->ka[1];
	c[2] = material->ka[2];
	c[3] = opacity;
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, c);

	// Load diffuse
	c[0] = material->kd[0];
	c[1] = material->kd[1];
	c[2] = material->kd[2];
	c[3] = opacity;
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, c);

	// Load specular
	c[0] = material->ks[0];
	c[1] = material->ks[1];
	c[2] = material->ks[2];
	c[3] = opacity;
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);

	// Load emission
	c[0] = material->emission.Red();
	c[1] = material->emission.Green();
	c[2] = material->emission.Blue();
	c[3] = opacity;
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, c);

	// Load shininess
	c[0] = material->shininess;
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, c[0]);

	// Load texture
	if (material->texture) 
	{
		if (material->texture_index <= 0) 
		{
			// Create texture in OpenGL
			GLuint texture_index;
			glGenTextures(1, &texture_index);
			material->texture_index = (int) texture_index;
			glBindTexture(GL_TEXTURE_2D, material->texture_index); 
			R2Image *image = material->texture;
			int npixels = image->NPixels();
			R2Pixel *pixels = image->Pixels();
			GLfloat *buffer = new GLfloat [ 4 * npixels ];
			R2Pixel *pixelsp = pixels;
			GLfloat *bufferp = buffer;
			for (int j = 0; j < npixels; j++) 
			{ 
				*(bufferp++) = pixelsp->Red();
				*(bufferp++) = pixelsp->Green();
				*(bufferp++) = pixelsp->Blue();
				*(bufferp++) = pixelsp->Alpha();
				pixelsp++;
			}
			glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_NEAREST);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
			glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
			gluBuild2DMipmaps( GL_TEXTURE_2D, 3, image->Width(), image->Height(), GL_RGBA, GL_FLOAT, buffer);
			delete [] buffer;
		}

		// Select texture
		glBindTexture(GL_TEXTURE_2D, material->texture_index); 
		glEnable(GL_TEXTURE_2D);
	}
	else 
	{
		glDisable(GL_TEXTURE_2D);
	}

	// Enable blending for transparent surfaces
	if (opacity < 1) 
	{
		glDepthMask(false);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_BLEND);
	}
	else 
	{
		glDisable(GL_BLEND);
		glBlendFunc(GL_ONE, GL_ZERO);
		glDepthMask(true);
	}
}
