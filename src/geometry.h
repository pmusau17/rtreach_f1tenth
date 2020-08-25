// Stanley Bak
// 4-2014
// Geometry header for real-time reach

#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <stdbool.h>
#include "main.h"

// pick the dynamics header to compile (this defines NUM_DIMS)
//#include "dynamics_harmonic_oscillator.h"
//#include "dynamics_vanderpol.h"
#ifdef BICYCLE_MODEL_NONLINEAR
#include "dynamics_bicycle.h"
#else

#ifdef LINEAR_PENDULUM
#include "dynamics_pendulum.h"
#else

#ifdef NONLINEAR_PENDULUM
#include "dynamics_pendulum_nonlinear.h"
#else



#error Dynamics File was not defined as compile flag (for example, try -DLINEAR_PENDULUM while compiling)
#endif
#endif
#endif

#define NUM_FACES (2 * NUM_DIMS)

typedef struct Interval
{
	REAL min;
	REAL max;
} Interval;

// for a HyperPoint use a REAL[]
typedef struct HyperPoint
{
	REAL dims[NUM_DIMS];
} HyperPoint;

// for a HyperRectangle use an Interval[]
typedef struct HyperRectangle
{
	Interval dims[NUM_DIMS];
} HyperRectangle;

REAL interval_width(Interval* i);

bool hyperrectangle_contains(HyperRectangle* outside, HyperRectangle* inside, bool printErrors);
void hyperrectangle_grow_to_convex_hull(HyperRectangle* grower, HyperRectangle* contained);
REAL hyperrectange_max_width(HyperRectangle* rect);
void hyperrectangle_bloat(HyperRectangle* out, REAL from[NUM_DIMS], REAL width);

void print(HyperRectangle* r);
void println(HyperRectangle* r);

#endif
