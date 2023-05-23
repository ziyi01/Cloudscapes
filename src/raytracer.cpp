#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "BeerLambert.h"
#include <math.h>
#include "CloudModel.h"
#include "Textures.h"

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::dot;
using glm::distance;
using glm::acos;
using glm::length;

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 150;
const int SCREEN_HEIGHT = 150;
const float OBJECT_MARCH_SAMPLE_STEP_SIZE = 2.0f;
const float LIGHT_MARCH_SAMPLE_STEP_SIZE = 2.0f;
SDL_Surface* screen;
int t;

struct Intersection
{
    vec3 position;
    float distance;
    int triangleIndex;
};

vector<Triangle> triangles;
float focalLength = SCREEN_HEIGHT/2;
vec3 cameraPos( 0,0,-110);

// 4, Rotation variables
mat3 R;
float rotationAngle = 0;

// Illumination variables
vec3 lightPos( 0, -0.5, -0.7 );
float lightBrightness = 0.2;
float radius = 200;
float ambientLight = 0.6;
float dampLight = 0.9;

// Anti Aliasing
int sampleCount = 2;

int counter = 0;

NoiseTexture3D tex;
vec3 boundsMin = vec3(-50, -50, -50);
vec3 boundsMax = vec3(50, 50, 50);
const float ATTENUATION = 0.02f;
// ----------------------------------------------------------------------------
// FUNCTIONS

void Update();
void Draw();
float GetTransmittance(float absorbance);
float DensityLookup(vec3 scale, vec3 localposition);
float Tex3DLookup (vec3 relativelocalpos);
float Henyey(float theta, float g);
float AngleSunRay(vec3 r_dir, vec3 s_dir);
float Phase(vec3 r_dir, vec3 position);

int main( int argc, char* argv[] )
{
    
	screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT );
	t = SDL_GetTicks();	// Set start value for timer.
    LoadTestModel(triangles);


    // Setting default angle for the camera
    vec3 c1 = vec3(1.0, 0.0, 0.0);
    vec3 c2 = vec3(0.0, 1.0, 0.0);
    vec3 c3 = vec3(0.0, 0.0, 1.0);
    R = mat3(c1, c2, c3);
    tex.GenerateTexture3D(25, 25, 25);
	while( NoQuitMessageSDL() )
	{
        Update();
	    Draw();
	}

	SDL_SaveBMP( screen, "screenshot.bmp" );
	return 0;
}

// Gets the distance between two vectors
float Distance(vec3 a, vec3 b){
    vec3 diff = b - a; 
    return sqrtf(dot(diff, diff));
}

bool BoxIntersection(const vec3 origin, const vec3 dir, const vec3 boundMin, const vec3 boundMax, float& tmin, float& tmax){
    
    // template: https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection.html
    
    vec3 invdir;
    invdir.x = 1 / dir.x;
    invdir.y = 1 / dir.y;
    invdir.z = 1 / dir.z;

    

    if(invdir.x >= 0) {
        tmin = (boundMin.x - origin.x) * invdir.x;
        tmax = (boundMax.x - origin.x) * invdir.x;
    }
    else {
        tmin = (boundMax.x - origin.x) * invdir.x;
        tmax = (boundMin.x - origin.x) * invdir.x;
    }

    float tymin;
    float tymax;

    if(invdir.y >= 0){
        tymin = (boundMin.y - origin.y) * invdir.y;
        tymax = (boundMax.y - origin.y) * invdir.y;
    }
    else{
        tymin = (boundMax.y - origin.y) * invdir.y;
        tymax = (boundMin.y - origin.y) * invdir.y;
    }

    if ((tmin > tymax) || (tymin > tmax))
        return false;
    
    vec3 t0 = (boundMin - origin) / dir;
    vec3 t1 = (boundMax - origin) / dir;

    if (tymin > tmin)
        tmin = tymin;
    if (tymax < tmax)
        tmax = tymax;


    float tzmin;
    float tzmax;

    if(invdir.z >= 0){
        tzmin = (boundMin.z - origin.z) * invdir.z;
        tzmax = (boundMax.z - origin.z) * invdir.z;
    }
    else{
        tzmin = (boundMax.z - origin.z) * invdir.z;
        tzmax = (boundMin.z - origin.z) * invdir.z;
    }
    

    if ((tmin > tzmax) || (tzmin > tmax))
        return false;

    if (tzmin > tmin)
        tmin = tzmin;
    if (tzmax < tmax)
        tmax = tzmax;

        if (tmin < 0) {
            tmin = tmax;
            if (tmax < 0) return false;
        }

        return true;

}

void Update()
{
	// Compute frame time:
	int t2 = SDL_GetTicks();
	float dt = float(t2-t);
	t = t2;
	cout << "Render time: " << dt << " ms." << endl;
    vec3 forward = R * vec3(0, 0, 1);
    vec3 right = R * vec3(1, 0, 0);
    Uint8* keystate = SDL_GetKeyState( 0 );
    if( keystate[SDLK_UP] )
    {
        cameraPos += vec3(R[2][0], R[2][1], R[2][2]) * 1.0f;
    }
    if( keystate[SDLK_DOWN] )
    {
        cameraPos -= vec3(R[2][0], R[2][1], R[2][2]) * 1.0f;
    }
    if( keystate[SDLK_LEFT] )
    {
        rotationAngle += 0.02;
        R[0][0] = cos(rotationAngle);
        R[0][2] = sin(rotationAngle);
        R[2][0] = -sin(rotationAngle);
        R[2][2] = cos(rotationAngle);
    }
    if( keystate[SDLK_RIGHT] )
    {
        rotationAngle -= 0.02;
        R[0][0] = cos(rotationAngle);
        R[0][2] = sin(rotationAngle);
        R[2][0] = -sin(rotationAngle);
        R[2][2] = cos(rotationAngle);
    }

    if( keystate[SDLK_w] )
        lightPos += vec3(R[2][0], R[2][1], R[2][2]) * 1.f;
    if( keystate[SDLK_s] )
        lightPos -= vec3(R[2][0], R[2][1], R[2][2]) * 1.f;
    if( keystate[SDLK_a] )
        lightPos -= vec3(R[0][0], R[0][1], R[0][2]) * 1.f;
    if( keystate[SDLK_d] )
        lightPos += vec3(R[0][0], R[0][1], R[0][2]) * 1.f;
    if( keystate[SDLK_e] )
        lightPos += vec3(R[1][0], R[1][1], R[1][2]) * 1.f;
    if( keystate[SDLK_q] )
        lightPos -= vec3(R[1][0], R[1][1], R[1][2]) * 1.f;
    
    if(keystate[SDLK_SPACE])
        tex.GenerateTexture3D(25, 25, 25);
}

//inner function for SampleAbsorbance. performs light pass from current position towards lightsource
float SampleLightAbsorbance (vec3 position, vec3 lightposition, vec3 scale, vec3 minBound, vec3 maxBound)
{
    vec3 direction = lightposition - position;
    vec3 localPosition = position - minBound;
    localPosition = localPosition/scale;

    float absorbance = 0;
    for(int i = 0; i < 10; i++)
    {
        float density = Tex3DLookup(localPosition);
        absorbance += BeerLambertIteration(density, ATTENUATION, LIGHT_MARCH_SAMPLE_STEP_SIZE);
        position += direction*LIGHT_MARCH_SAMPLE_STEP_SIZE;
    }
    return absorbance;
}

float SampleAbsorbance (vec3 position, vec3 direction, vec3 scale, vec3 minBound, vec3 maxBound, float &shadeAbsorption)
{
    //std::cout << "------------------Debug for Absorbance sample------------------" << endl;
    //std::cout << "Real position" << position.x << " " << position.y << " " << position.z << endl;
    vec3 localPosition = position - minBound;
    //std::cout << "Relative position" << localPosition.x << " " << localPosition.y << " " << localPosition.z << endl;
    //std::cout << "Scale" << scale.x << " " << scale.y << " " << scale.z << endl;
    localPosition = localPosition/scale;
    //std::cout << "Local position" << localPosition.x << " " << localPosition.y << " " << localPosition.z << endl;
    //std::cout << "---------------------------------------------------------------" << endl;

    float density = Tex3DLookup(localPosition);
    float beerLambertAbsorbance = BeerLambertIteration(density, ATTENUATION, OBJECT_MARCH_SAMPLE_STEP_SIZE);
    float phase = Phase(direction, localPosition);
    shadeAbsorption = SampleLightAbsorbance(position, lightPos, scale, minBound, maxBound);
    return beerLambertAbsorbance * phase;
}



void Draw()
{
    Intersection intrs;
	if( SDL_MUSTLOCK(screen) )
		SDL_LockSurface(screen);

	for( int y=0; y<SCREEN_HEIGHT; ++y )
	{
		for( int x=0; x<SCREEN_WIDTH; ++x )
		{
            vec3 color( 0.7, 0.8, 0.9);
            
            vec3 dir(x-SCREEN_WIDTH/2, y-SCREEN_HEIGHT/2, focalLength);
            dir = R*dir;
            dir = glm::normalize(dir);
            float distToBox;
            float distToExit;
            vec3 scale = boundsMax - boundsMin;
            float shadeAbsorption = 0;

            if(BoxIntersection(cameraPos, dir, boundsMin, boundsMax, distToBox, distToExit)){
                vec3 entry = cameraPos + dir * distToBox;
                vec3 exit = cameraPos + dir * distToExit;
                //std::cout << "---------------------------------------------------------------" << endl;
                //std::cout << entry.x << " " << entry.y << " " << entry.z << endl;
                //std::cout << exit.x << " " << exit.y << " " << exit.z << endl;
                //std::cout << "---------------------------------------------------------------" << endl;
                
                float distance = glm::distance(entry, exit);
                vec3 position = entry;
                float absorbance = 0;
                
                //Set a higher bound so we don't lose control of this loop
                for(int i = 0; i < 100; i++)
                {
                    if(distance < 0)
                    {
                        //cout << "i: " << i << endl;
                        break;
                    }
                    absorbance += SampleAbsorbance(position, dir, scale, boundsMin, boundsMax, shadeAbsorption);

                    position += OBJECT_MARCH_SAMPLE_STEP_SIZE*dir;
                    distance -= OBJECT_MARCH_SAMPLE_STEP_SIZE;
                }
                
                if(absorbance != 0)
                {
                    //std::cout << "Absorbance: " << absorbance << endl;
                }
            
                float transmittance = GetTransmittance(absorbance);
                float shadeTransmittance = GetTransmittance(shadeAbsorption);
                color += color*(1-transmittance)*dampLight;
                //color -= vec3(1-shadeTransmittance); Does not work in the slightest
            }
            

			PutPixelSDL( screen, x, y, color );
		}
	}

	if( SDL_MUSTLOCK(screen) )
		SDL_UnlockSurface(screen);

	SDL_UpdateRect( screen, 0, 0, 0, 0 );
}

//Finds pixels in a 3D textures by looking at the object relative coordinates (ranging from 0 to 1)
float Tex3DLookup (vec3 relativelocalpos)
{	
	ivec3 pixelPosition;
    pixelPosition.x = relativelocalpos.x * tex.width;
	pixelPosition.y	= relativelocalpos.y * tex.height;
	pixelPosition.z = relativelocalpos.z * tex.depth;
    float pix = tex.GetPixel(pixelPosition);
    //cout << pixelPosition.x << " " << pixelPosition.y << " " << pixelPosition.z << endl << "********* " << pix << " **********"<< endl;
	return pix;
}

//Get Tex3D opacity from Tex3D
float DensityLookup(vec3 scale, vec3 localposition)
{
	return Tex3DLookup(localposition/scale);
}

float GetTransmittance(float absorbance)
{
	return std::pow(10, -absorbance);
}

// Scattering Functions
/**
 * @brief 
 * 
 * @param r_dir Ray tracer direction
 * @param s_dir Sun/Light source direction
 * @return float 
 */
float Phase(vec3 r_dir, vec3 position) {
    // g - [-1,+1] preferred scattering direction.
    // g > 0 is forward scattering dominant
    // g = 0 is sideways dominant
    // g < 0 is backwards scattering dominant
    vec3 lightDir = lightPos-position;
    float g1 = 0.899f; // Forward 
    float g2 = -0.3f; // Back
    float w_1 = 0.7;  // Distribution of weight, sum of w_i = 1
    float w_2 = 0.3;
    float theta = AngleSunRay(r_dir, lightDir);
    
    // Implement extinction
    return lightBrightness * ((w_1*Henyey(theta,g1)) + (w_2*Henyey(theta,g2))) + ambientLight; // Accumulate functions to fitted scattering
}

float AngleSunRay(vec3 r_dir, vec3 s_dir) {
    // Dot angle between ray from camera and sun light direction
    return acos(dot(r_dir,s_dir)/(length(r_dir)*length(s_dir)));
}

/**
 * @brief Single particle scattering events based on Mie theory
 * 
 * @param theta - Phase angle between incoming and outgoing directions
 */
float Henyey(float theta, float g) { // Adjust g to add more scattering
    float my = cos(theta);
    float g2 = g*g;
    float hg = ((1 - g2) / (4 * M_PI * pow(1 + g2 - (2 * g * my), 3/2)));
    return hg;
}
