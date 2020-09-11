#include "bicycle_model.h"
#include "main.h"
#include "face_lift.h"
#include "util.h"
#include "simulate_bicycle_plots.h"
#include <stdio.h>

static FILE* f_initial;
static FILE* f_intermediate;
static FILE* f_final;



// declaration 
bool face_lifting_iterative_improvement_bicycle(int startMs, LiftingSettings* settings, REAL heading_input, REAL throttle);

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

// This function enumerates all of the corners of the current HyperRectangle and 
// returns whether or not any of the points lies outside of the ellipsoid. The corner
// Thing is only really helpful for linear models.
bool finalState(HyperRectangle* rect)
{
	REAL maxPotential = 0.0;

	return maxPotential > 1;
}


// Simulation 
REAL getSimulatedSafeTime(REAL start[4],REAL heading_input,REAL throttle)
{
	REAL stepSize = 0.002f;
	REAL rv = 0.0f;

	simulate_bicycle_plots(start, heading_input,throttle,stepSize, shouldStop, (void*)&rv); // TODO: look here

	//DEBUG_PRINT("time until simulation reaches safe state = %f\n", rv);

	return rv;
}

void close_files(bool closeInitial)
{
	if (closeInitial && f_initial)
		fclose(f_initial);

	if (f_intermediate)
		fclose(f_intermediate);

	if (f_final)
		fclose(f_final);
}


void open_files(bool openInitial)
{
	if (openInitial)
		f_initial = fopen("bicycle_initial.gnuplot.txt", "w");

	f_intermediate = fopen("bicycle_intermediate.gnuplot.txt", "w");
	f_final = fopen("bicycle_final.gnuplot.txt", "w");

	if (!f_initial || !f_intermediate || !f_final)
	{
		close_files(true);

		error_exit("error opening files");
	}
}

// if computation restarts we close and reopen files
void restartedComputation()
{
	// close and open the files to clear them
	close_files(false);
	open_files(false);
}

// styleIndex: 0 = init, 1 = intermediate, 2 = final
void hyperrectangle_to_file(FILE* fout, HyperRectangle* r, int styleIndex)
{
	if (fout)
	{
        // select which dimesnions you want to plot
		int X_DIM = 0;
		int Y_DIM = 1;

		const char* styleStr[] =
		{
			"set label ' Init' at %f, %f point pointtype 3 lc rgb 'blue' tc rgb 'blue'",
			"set obj rect from %f, %f to %f, %f fc rgbcolor 'dark-green' fs solid 0.2 \n",
			"set obj rect from %f, %f to %f, %f fc rgbcolor 'red' fs solid 0.3 noborder\n",
		};

		fprintf(fout, styleStr[styleIndex], r->dims[X_DIM].min, r->dims[Y_DIM].min,
				r->dims[X_DIM].max, r->dims[Y_DIM].max);
	}
}


// called on states reached during the computation
// this function basically says that all intermediate sates are safe
// during the reachset computation
bool intermediateState(HyperRectangle* r)
{
	bool allowed = true;

	hyperrectangle_to_file(f_intermediate, r,1);

	return allowed;
}



// reachability analysis
bool runReachability_bicycle(REAL* start, REAL simTime, REAL wallTimeMs, REAL startMs,REAL heading_input, REAL throttle)
{
	LiftingSettings set;
	printf("Starting reachability computation from the following state:\n");
	for (int d = 0; d < NUM_DIMS; ++d)
	{
		set.init.dims[d].min = start[d];
		set.init.dims[d].max = start[d];
		printf("[%f,%f]\n",set.init.dims[d].min,set.init.dims[d].max);
	}

	set.reachTime = simTime;
	set.maxRuntimeMilliseconds = wallTimeMs;

	REAL iss = set.reachTime;
	iss = iss * 0.10f;

	DEBUG_PRINT("\n\rsimTime: %f\n\rreachTime: %f\n\r\n\r", simTime, set.reachTime);
	
	

	set.initialStepSize = iss; //set.reachTime / 10.0f;
	set.maxRectWidthBeforeError = 100;

	set.reachedAtFinalTime = finalState;
	set.reachedAtIntermediateTime = intermediateState;
	set.restartedComputation = restartedComputation; 

    open_files(true);
	hyperrectangle_to_file(f_initial, &set.init, 0);

	// debugging for patrick
	printf("Beginning Reachability Analysis >>>> initialStepSize: %f, reachTime: %f\n\n",set.initialStepSize,set.reachTime);

	return face_lifting_iterative_improvement_bicycle(startMs, &set,heading_input, throttle);
}

