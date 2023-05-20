#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include <math.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::acos;
using glm::length;
using glm::dot;

// ----------------------------------------------------------------------------
// FUNCTION DECLARATIONS
float InOutScatter(vec3 r_dir, vec3 s_dir);
float AngleSunRay(vec3 r_dir, vec3 s_dir);
float Henyey(float theta, float g);