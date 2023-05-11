#ifndef BEER_LAMBERT
#define BEER_LAMBERT

#include <glm/glm.hpp>
#include <math.h>
#include <vector>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::dot;
using glm::distance;

struct Texture3D
{
	int width;
	int height;
	int depth;
	vector<float> data_;
};

void GetPixel (Texture3D tex, vec3 coordinate)
{
	
}

void GenerateNoiseCube ()
{

}


float BeerLambertIteration(float density, float attenuation, float delta)
{
	return delta * density * attenuation;
}

float BeerLambert (vector<vec3> samplePoints, float delta, float attenuation)
{

	return 0;
}

float DensityLookup (vec3 position)
{
	int posX = position.x;
	int posY = position.y;
	int posZ = position.z;
	return 0;
}

#endif