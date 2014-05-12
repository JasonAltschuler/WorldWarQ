#ifndef __TEXTURE_H__
#define __TEXTURE_H__

typedef enum 
{
	ROCK,
	GRASS,
	SNOW,
	DIRT
} R3Texture;

#include "strings.h"
#include "R3Scene.h"

extern R3Material **materials;

void MakeMaterials(R3Material **materials);

void LoadMaterial(R3Material *material);

#endif