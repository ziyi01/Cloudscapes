#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "BeerLambert.h"
#include <math.h>
#include "TestModel.h"
#include "Textures.h"


using namespace std;
using glm::vec3;
using glm::mat3;
using glm::dot;
using glm::distance;

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 150;
const int SCREEN_HEIGHT = 150;
const int SAMPLE_STEP_SIZE = 0.1f;
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
vec3 cameraPos( 0,0,-2);

// 4, Rotation variables
mat3 R;
float rotationAngle = 0;

// 5. Illumination variables
vec3 lightPos( 0, -0.5, -0.7 );
vec3 lightColor = 14.f * vec3( 1, 1, 1 );
float radius = 200;

// Inderect lighting
vec3 indirectLight = 0.5f*vec3( 1, 1, 1 );

// Anti Aliasing
int sampleCount = 2;

int counter = 0;


Texture3D tex;
// ----------------------------------------------------------------------------
// FUNCTIONS

void Update();
void Draw();
float SampleAbsorbance (Texture3D texture, vec3 direction, vec3 scale, vec3 entry, vec3 exit);

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
    tex = GenerateTexture3D(50, 50, 50);
	while( NoQuitMessageSDL() )
	{
        //break;
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

    tmin = (boundMin.x - origin.x) * invdir.x;
    tmax = (boundMax.x - origin.x) * invdir.x;
    float tymin = (boundMin.y - origin.y) * invdir.y;
    float tymax = (boundMax.y - origin.y) * invdir.y;

    if ((tmin > tymax) || (tymin > tmax))
        return false;
    
    vec3 t0 = (boundMin - origin) / dir;
    vec3 t1 = (boundMax - origin) / dir;

    if (tymin > tmin)
        tmin = tymin;
    if (tymax < tmax)
        tmax = tymax;

    float tzmin = (boundMin.z - origin.z) * invdir.z;
    float tzmax = (boundMax.z - origin.z) * invdir.z;

    if ((tmin > tzmax) || (tzmin > tmax))
        return false;

    if (tzmin > tmin)
        tmin = tzmin;
    if (tzmax < tmax)
        tmax = tzmax;

        float t = tmin;

        if (t < 0) {
            t = tmax;
            if (t < 0) return false;
        }

        return true;

}


/*
vec3 DirectLight(const Intersection& i){

    // n is the normal of the triangle surface
    vec3 n = triangles[i.triangleIndex].normal;

    // r is the vector from the surface point to the light source
    vec3 r =  lightPos - i.position;

    // lightDir is the vector from the surface point to the light source
    vec3 lightDir = (i.position - lightPos);

    //evaluate if shadow
    Intersection closest;

    if( ClosestIntersection(lightPos, lightDir, triangles, closest) && 
        Distance(lightPos, closest.position) +0.01 < Distance(lightPos, i.position) &&
        closest.triangleIndex != i.triangleIndex )
    {
        vec3 zero(0,0,0);
        return zero;
        
    }

    // The surface area of the sphere
    float A = 4*M_PI*glm::length(r)*glm::length(r);
    
    // the power per area (W/m^2)
    vec3 B = lightColor / A;

    // AAA dotn understand entirely
    float nr = dot(r,n);

    vec3 D;
    if(nr > 0){
        D = B * nr;
    }else {
        D = B * 0.0f;
    }
    
    return D;

}*/

void Update()
{
	// Compute frame time:
	int t2 = SDL_GetTicks();
	float dt = float(t2-t);
	t = t2;
	cout << "Render time: " << dt << " ms." << endl;

    Uint8* keystate = SDL_GetKeyState( 0 );
    if( keystate[SDLK_UP] )
    {
        cameraPos += vec3(R[2][0], R[2][1], R[2][2]) * 0.1f;
    }
    if( keystate[SDLK_DOWN] )
    {
        cameraPos -= vec3(R[2][0], R[2][1], R[2][2]) * 0.1f;
    }
    if( keystate[SDLK_LEFT] )
    {
        rotationAngle += 0.01;
        R[0][0] = cos(rotationAngle);
        R[0][2] = sin(rotationAngle);
        R[2][0] = -sin(rotationAngle);
        R[2][2] = cos(rotationAngle);
    }
    if( keystate[SDLK_RIGHT] )
    {
        rotationAngle -= 0.01;
        R[0][0] = cos(rotationAngle);
        R[0][2] = sin(rotationAngle);
        R[2][0] = -sin(rotationAngle);
        R[2][2] = cos(rotationAngle);
    }

    if( keystate[SDLK_w] )
        lightPos += vec3(R[2][0], R[2][1], R[2][2]) * 0.1f;
    if( keystate[SDLK_s] )
        lightPos -= vec3(R[2][0], R[2][1], R[2][2]) * 0.1f;
    if( keystate[SDLK_a] )
        lightPos -= vec3(R[0][0], R[0][1], R[0][2]) * 0.1f;
    if( keystate[SDLK_d] )
        lightPos += vec3(R[0][0], R[0][1], R[0][2]) * 0.1f;
    if( keystate[SDLK_e] )
        lightPos += vec3(R[1][0], R[1][1], R[1][2]) * 0.1f;
    if( keystate[SDLK_q] )
        lightPos -= vec3(R[1][0], R[1][1], R[1][2]) * 0.1f;
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
            vec3 color( 0, 0, 0 );

            
            vec3 dir(x-SCREEN_WIDTH/2, y-SCREEN_HEIGHT/2, focalLength);

            dir = R*dir;
            float distToBox;
            float distToExit;

            vec3 boundsMin = vec3(130, 0, 65);
            vec3 boundsMax = vec3(290, 165, 272);
            vec3 scale = boundsMax - boundsMin;

            if(BoxIntersection(cameraPos,dir,boundsMin,boundsMax, distToBox,distToExit)){
                color = vec3(1,1,1);
                vec3 entry = cameraPos + dir * distToBox;
                vec3 exit = cameraPos + dir * distToExit;
                
                
                float distance = glm::distance(entry, exit);
                vec3 position = entry;
                float absorbance = 0;
                while (distance > 0)
                {
                    absorbance += SampleAbsorbance(tex, dir, scale, entry, exit);
                    distance -= SAMPLE_STEP_SIZE;
                }
                float transmittance = GetTransmittance(absorbance);
                color = vec3(transmittance, transmittance, transmittance);
                
            }
            
			PutPixelSDL( screen, x, y, color );
		}
	}

	if( SDL_MUSTLOCK(screen) )
		SDL_UnlockSurface(screen);

	SDL_UpdateRect( screen, 0, 0, 0, 0 );
}

float SampleAbsorbance (Texture3D texture, vec3 direction, vec3 scale, vec3 entry, vec3 exit)
{
    vec3 localPosition = exit-entry;
    float density = DensityLookup(texture, scale, localPosition);
    float beerLambertAbsorbance = BeerLambertIteration(density, 0.1f, SAMPLE_STEP_SIZE);
    float henyeyGreenstein = 1;
    return beerLambertAbsorbance * henyeyGreenstein;
}


