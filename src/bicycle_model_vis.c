#include "bicycle_model.h"
#include "main.h"
#include "face_lift.h"
#include "util.h"
#include "simulate_bicycle.h"
#include "bicycle_safety.h"
#include <stdio.h>
#include <string.h> 
#include <stdlib.h>


// a note from the f1tenth simulator 
// the car is 0.5 m long in the x direction 
// 0.3 long in the y direction


// cones in the scenario we are considering, I'll get rid of these eventually
double cones[5][2][2] = {{{1.935, 2.065},{1.935, 2.065}},{{4.635, 4.765},{2.635, 2.765}},
						{{11.295, 11.425},{-1.525, -1.395}},{{2.935, 3.065},{6.335, 6.465}},{{-9.705, -9.575},{2.895, 3.025}}};

// do face lifting with the given settings, iteratively improving the computation
// returns true if the reachable set of states is satisfactory according to the
// function you provide in LiftingSettings (reachedAtIntermediateTime, reachedAtFinalTime)

// visualization version, returns convex hull
HyperRectangle face_lifting_iterative_improvement_bicycle_vis(int startMs, LiftingSettings* settings, REAL heading_input, REAL throttle,bool plot);


// helper function to check safety
bool check_safety(HyperRectangle* rect, REAL (*cone)[2]);


// function that stops simulation after two seconds
bool shouldStop(REAL state[NUM_DIMS], REAL simTime, void* p)
{
	bool rv = false;
    REAL maxTime = 2.0f;
    // stop if the maximum simulation time 
	if (simTime >= maxTime)
	{
		rv = true;

		REAL* stopTime = (REAL*)p;
		*stopTime = -1;
	}

	return rv;
}



// Simulation 
REAL getSimulatedSafeTime(REAL start[4],REAL heading_input,REAL throttle)
{
	REAL stepSize = 0.02f;
	REAL rv = 0.0f;

	simulate_bicycle(start, heading_input,throttle,stepSize, shouldStop, (void*)&rv); // TODO: look here

	//DEBUG_PRINT("time until simulation reaches safe state = %f\n", rv);

	return rv;
}

// called on states reached during the computation
bool intermediateState(HyperRectangle* r)
{
	bool allowed = true;
	//const REAL FIFTEEN_DEGREES_IN_RADIANS = 0.2618;

	// bloat the box for the width of the car
	r->dims[0].min = r->dims[0].min  - 0.25;
	r->dims[0].max = r->dims[0].max  + 0.25;
	r->dims[1].min = r->dims[1].min  - 0.15;
	r->dims[1].max = r->dims[1].max  + 0.15;


	// loop through the cones
	for (int j = 0; j < 5; j++)
	{
		allowed = check_safety(r,cones[j]);
		if(!allowed)
        {
            printf("offending cone (%f,%f) (%f, %f)\n",cones[j][0][0],cones[j][0][1],cones[j][1][0],cones[j][1][1]);
            break;
        }
		
	}

	if(allowed)
	{
		allowed = check_safety_wall(r);
	}
	
	// reset it
	r->dims[0].min = r->dims[0].min  + 0.25;
	r->dims[0].max = r->dims[0].max  - 0.25;
	r->dims[1].min = r->dims[1].min  + 0.15;
	r->dims[1].max = r->dims[1].max  - 0.15;

	if(!allowed)
		printf("unsafe....\n");
	return allowed;
}

// This function enumerates all of the corners of the current HyperRectangle and 
// returns whether or not any of the points lies outside of the ellipsoid
bool finalState(HyperRectangle* rect)
{

	return intermediateState(rect);
}


// reachability analysis
HyperRectangle runReachability_bicycle_vis(REAL* start, REAL simTime, REAL wallTimeMs, REAL startMs,REAL heading_input, REAL throttle)
{
	LiftingSettings set;
	for (int d = 0; d < NUM_DIMS; ++d)
	{
		set.init.dims[d].min = start[d];
		set.init.dims[d].max = start[d];
	}

	set.reachTime = simTime;
	set.maxRuntimeMilliseconds = wallTimeMs;

	REAL iss = set.reachTime;
	iss = iss * 0.10f;

	set.initialStepSize = iss; //set.reachTime / 10.0f;
	set.maxRectWidthBeforeError = 100;

	set.reachedAtFinalTime = finalState;
	set.reachedAtIntermediateTime = intermediateState;
    set.restartedComputation = 0; 

	return face_lifting_iterative_improvement_bicycle_vis(startMs, &set,heading_input, throttle,false);
}
