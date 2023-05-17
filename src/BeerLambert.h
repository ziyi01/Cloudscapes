#ifndef BEER_LAMBERT
#define BEER_LAMBERT

#include <glm/glm.hpp>
#include <math.h>
#include <vector>
#include <random>
#include <cmath>
#include <limits>
#include "Textures.h"

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::dot;
using glm::distance;

float BeerLambertIteration(float density, float attenuation, float delta)
{
	return delta * density * attenuation;
}

float BeerLambert (vector<vec3> samplePoints, float delta, float attenuation)
{
	return 0;
}

//Finds pixels in a 3D textures by looking at the object relative coordinates (ranging from 0 to 1)
float Tex3DLookup (Texture3D tex, vec3 relativelocalpos)
{	
	vec3 pixelPosition((int)relativelocalpos.x * tex.width, 
						(int)relativelocalpos.y * tex.height, 
						(int)relativelocalpos.z * tex.depth);
	return GetPixel(tex, pixelPosition);
}

//Get Tex3D opacity from Tex3D
float DensityLookup(Texture3D tex, vec3 scale, vec3 localposition)
{
	return Tex3DLookup(tex, localposition/scale);
}

float GetTransmittance(float absorbance)
{
	return std::pow(10, -absorbance);
}


#endif
