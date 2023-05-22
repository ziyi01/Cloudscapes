#ifndef Textures
#define Textures

#include <glm/glm.hpp>
#include <math.h>
#include <vector>
#include <random>
#include <cmath>
#include <limits>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::dot;
using glm::distance;
using glm::ivec3;



struct Texture3D
{
	int width;
	int height;
	int depth;
	vector<float> data;
};

vec3 generateRandomPoint(vec3 maxes) {
    std::random_device rdx;
    std::mt19937 genx(rdx());
    std::random_device rdy;
    std::mt19937 geny(rdy());
    std::random_device rdz;
    std::mt19937 genz(rdz());
    std::uniform_real_distribution<float> distX(0, maxes.x);
    std::uniform_real_distribution<float> distY(0, maxes.y);
    std::uniform_real_distribution<float> distZ(0, maxes.z);
    
    vec3 point;
    point.x = distX(rdx);
    point.y = distY(rdy);
    point.z = distZ(rdz);
    
    return point;
}

int PixelCoordToIndex(Texture3D tex, vec3 coordinate)
{
	int posX = coordinate.x;
	int posY = coordinate.y;
	int posZ = coordinate.z;
	return coordinate.x + tex.width*(coordinate.y + tex.height*coordinate.z);
}

bool ValidPixelCoordinate (Texture3D tex, vec3 coordinate)
{
    return (coordinate.x > 0) && (coordinate.x < tex.width) &&
            (coordinate.y > 0) && (coordinate.y < tex.height) &&
            (coordinate.z > 0) && (coordinate.z < tex.depth);
}

float GetPixel (Texture3D tex, vec3 coordinate)
{
    if(!ValidPixelCoordinate(tex, coordinate))
        return 0;
	return tex.data[PixelCoordToIndex(tex, coordinate)];
}

void SetPixel (Texture3D &tex, vec3 coordinate, float newValue)
{
    if(!ValidPixelCoordinate(tex, coordinate))
        return;
	int index = PixelCoordToIndex(tex, coordinate);
	tex.data[index] = newValue;
}


void generateWorleyNoise(Texture3D &tex) {
    int width = tex.width;
    int height = tex.height;
    int depth = tex.depth;
    // Set the density of points
    int numPoints = 10;

    // Generate random points within the texture boundaries
    std::vector<vec3> points(numPoints);
    for (int i = 0; i < numPoints; i++) {
        points[i] = generateRandomPoint(vec3(width, height, depth));
        std::cout << points[i].x << " " << points[i].y << " " << points[i].z << endl;
    }

    std::cout << "-------------------------------------------" << endl << endl << endl;
    float maximum = std::max(std::max(width, height), depth);
    // Traverse the texture and calculate the Worley noise for each texel
    for (int z = 0; z < depth; z++) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                float minDist = 999999999999;
                for(int i = 0; i < numPoints; i++)
                {
                    float d = glm::distance(vec3(x, y, z), points[i]);
                    if(d < minDist)
                    {
                        minDist = d;
                    }
                }

                float d = minDist*2 / maximum;
                d = 1-d;
                if(d < 0)
                {
                    d = 0;
                }
                if(d > 1)
                {
                    d = 1;
                }
                std::cout << d << "-";
                SetPixel(tex, vec3(x, y, z), d);
            }
            std::cout << endl << "------------------------------------------------------------------"<< endl;
        }
    }

}


void GenerateNoiseCube (Texture3D &tex)
{
	generateWorleyNoise(tex);
}

Texture3D GenerateTexture3D (int width, int height, int depth)
{
	struct Texture3D tex;
	tex.width = width;
	tex.height = height;
	tex.depth = depth;
	tex.data = vector<float>(width*height*depth);

	GenerateNoiseCube(tex);

	return tex;
}



#endif
