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

class NoiseTexture3D
{
    public:
       	int width;
        int height;
        int depth; 
        vector<float> data;
    
    int PixelCoordToIndex(ivec3 coordinate)
    {
        int posX = coordinate.x;
        int posY = coordinate.y;
        int posZ = coordinate.z;
        return coordinate.x + width*(coordinate.y + height*coordinate.z);
    }

    bool ValidPixelCoordinate (ivec3 coordinate)
    {
        return (coordinate.x >= 0) && (coordinate.x < width) &&
                (coordinate.y >= 0) && (coordinate.y < height) &&
                (coordinate.z >= 0) && (coordinate.z < depth);
    }
    float GetPixel (ivec3 coordinate)
    {
        if(!ValidPixelCoordinate(coordinate))
            return 0;
        return data[PixelCoordToIndex(coordinate)];
    }
    void SetPixel (ivec3 coordinate, float newValue)
    {
        if(!ValidPixelCoordinate(coordinate))
            return;
        int index = PixelCoordToIndex(coordinate);
        data[index] = newValue;
    }

    void generateWorleyNoise() {
        // Set the density of points
        int numPoints = 10;

        // Generate random points within the texture boundaries
        std::vector<vec3> points(numPoints);
        for (int i = 0; i < numPoints; i++) {
            points[i] = generateRandomPoint(vec3(width, height, depth));
            std::cout << points[i].x << " " << points[i].y << " " << points[i].z << endl;
        }

        //std::cout << "-------------------------------------------" << endl << endl << endl;
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
                    //std::cout << d << "-";
                    int dPow = 1;
                    float dTemp = d;
                    for(int i = 0; i < dPow; i++)
                        d*=dTemp;
                    SetPixel(ivec3(x, y, z), d);
                }
                //std::cout << endl << "------------------------------------------------------------------"<< endl;
            }
        }
    }

    void GenerateTexture3D (int w, int h, int d)
    {
        width = w;
        height = h;
        depth = d;
        data = vector<float>(w*h*d);

        generateWorleyNoise();
    }
};




#endif
