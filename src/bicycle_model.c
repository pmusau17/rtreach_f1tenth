#include "bicycle_model.h"
#include "main.h"
#include "face_lift.h"
#include "util.h"
#include "simulate_bicycle.h"
#include <stdio.h>



// declaration 
bool face_lifting_iterative_improvement_bicycle(int startMs, LiftingSettings* settings, REAL heading_input, REAL throttle);

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



// helper function to check safety
bool check_safety(HyperRectangle* rect, REAL (*cone)[2])
{
	
	REAL l1[2] = {rect->dims[0].min,rect->dims[1].max};
    REAL r1[2] = {rect->dims[0].max,rect->dims[1].min};

	REAL l2[2] = {cone[0][0],cone[1][1]};
    REAL r2[2] = {cone[0][1],cone[1][0]};
	
	if (l1[0] >= r2[0] || l2[0] >= r1[0]) 
        return true; 
    
    if (l1[1] <= r2[1] || l2[1] <= r1[1]) 
        return true; 

	return false;
}




// called on states reached during the computation
bool intermediateState(HyperRectangle* r)
{
	bool allowed = true;
	//const REAL FIFTEEN_DEGREES_IN_RADIANS = 0.2618;

	// Alright for now I'm going to encode the cones in here manually.
	// This isn't ideal but also it's a first step

	double cone1[2][2] = {{1,2},{-0.5,0.5}};
	double cone2[2][2] = {{3,4},{-0.5,0.5}};
	double cone3[2][2] = {{5,6},{-0.5,0.5}};
	double cone4[2][2] = {{7,8},{-0.5,0.5}};
	double cone5[2][2] = {{9,10},{-0.5,0.5}};

	
    if(check_safety(r,cone1) && check_safety(r,cone2) && check_safety(r,cone3) && check_safety(r,cone4) && check_safety(r,cone5))
	{
		allowed = true;
	}
	else{
		allowed = false;
	}

	if(!allowed)
		printf("unsafe..../n");
	return allowed;
}

// This function enumerates all of the corners of the current HyperRectangle and 
// returns whether or not any of the points lies outside of the ellipsoid
bool finalState(HyperRectangle* rect)
{

	return intermediateState(rect);
}




bool runReachability_bicycle(REAL* start, REAL simTime, REAL wallTimeMs, REAL startMs,REAL heading_input, REAL throttle)
{
	LiftingSettings set;
	// printf("Starting reachability computation from the following state:\n");
	for (int d = 0; d < NUM_DIMS; ++d)
	{
		set.init.dims[d].min = start[d];
		set.init.dims[d].max = start[d];
		// printf("[%f,%f]\n",set.init.dims[d].min,set.init.dims[d].max);
	}

	set.reachTime = simTime;
	set.maxRuntimeMilliseconds = wallTimeMs;

	REAL iss = set.reachTime;
//	iss = iss / 10.0f; // problem with division?
	iss = iss * 0.10f;

	// DEBUG_PRINT("\n\rsimTime: %f\n\rreachTime: %f\n\r\n\r", simTime, set.reachTime);
	
	

	set.initialStepSize = iss; //set.reachTime / 10.0f;
	set.maxRectWidthBeforeError = 100;

	set.reachedAtFinalTime = finalState;
	set.reachedAtIntermediateTime = intermediateState;
	set.restartedComputation = 0; //restartedComputation;

	// debugging for patrick
	//printf("Beginning Reachability Analysis >>>> initialStepSize: %f, reachTime: %f\n\n",set.initialStepSize,set.reachTime);

	return face_lifting_iterative_improvement_bicycle(startMs, &set,heading_input, throttle);
}




