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

// ----------------------------------------------------------------------------
// FUNCTIONS

void Update();
void Draw();
void ClosestIntersection();

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
    Texture3D tex;
    GenerateTexture3D(120, 120, 120);
	
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

// CREATE MY OWN INVERSE



bool ClosestIntersection(vec3 start, vec3 dir, const vector<Triangle>& triangles, Intersection& closestIntersection){
    
    Intersection closest;
    float minDist = FLT_MAX;
    int index = 0;

    vec3 v0;
    vec3 v1;
    vec3 v2;

    vec3 e1;
    vec3 e2;
    
    for(Triangle triangle: triangles){
        v0 = triangle.v0;
        v1 = triangle.v1;
        v2 = triangle.v2;

        // Specify the edges of the Triangle to construct a coordinate system aligned with the triangle
        e1 = v1 - v0;
        e2 = v2 - v0;

        //
        vec3 b = start - v0;

        // get the intersection between the plane of the triangle and the ray, by inserting the equation of the plane (6) 
        // into the equation of the line, then solving the linear systems of equations
        mat3 A( -dir, e1, e2 );
        vec3 coords = glm::inverse( A ) * b;
        
        if(coords.y >= 0 && coords.z >= 0 && (coords.y + coords.z) <= 1 && coords.x >= 0){
            
            
            // assign new closest
            if( coords.x< minDist){

                closestIntersection.position = start + dir*coords.x;

                closestIntersection.triangleIndex = index;
                closestIntersection.distance = coords.x;
                minDist = coords.x;
            }
        }

        index++;
    }

    if(minDist != FLT_MAX)
        return true;

    return false;
}



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

}

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
        rotationAngle += 0.1;
        R[0][0] = cos(rotationAngle);
        R[0][2] = sin(rotationAngle);
        R[2][0] = -sin(rotationAngle);
        R[2][2] = cos(rotationAngle);
    }
    if( keystate[SDLK_RIGHT] )
    {
        rotationAngle -= 0.1;
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
            if(ClosestIntersection(cameraPos, dir, triangles, intrs)){
                color = triangles[intrs.triangleIndex].color * (DirectLight(intrs) +  indirectLight);
            }
            
        
           /*
            vec3 dir(x_off-SCREEN_WIDTH/2, y_off-SCREEN_HEIGHT/2, focalLength);
                    dir = R*dir;
            if(ClosestIntersection(cameraPos, dir, triangles, intrs)){
                color = triangles[intrs.triangleIndex].color * (DirectLight(intrs) +  indirectLight);
            }
            */
            
			PutPixelSDL( screen, x, y, color );
		}
	}

	if( SDL_MUSTLOCK(screen) )
		SDL_UnlockSurface(screen);

	SDL_UpdateRect( screen, 0, 0, 0, 0 );
}


