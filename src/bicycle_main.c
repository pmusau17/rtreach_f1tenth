// example call: ./bicycle 100 0.0 0.0 0.0 0.0 16.0 0.26666
// example call output:
// started!
// Argc: 6
// runtime: 100 ms
// x_0[0]: -0.100000
// x_0[1]: 0.000000
// x_0[2]: 0.000000
// x_0[3]: 1.100000
// potential of start state = 0.986088
// ms: 468
// done, result = safe
//
// example call (arm): ./rtreach_arm 100 -0.1 0.0 0.0 0.0
// ./rtreach_arm 100 -0.1 0.4 0.0 0.0

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "util.h"
#include "main.h"
#include "bicycle_model.h"

#ifdef MATLAB
	// mxArray: see: http://www.mathworks.com/help/matlab/apiref/mxarray.html
	#include "matrix.h"
#endif

// TODO: add a macro to call either mexPrintf or printf as appropriate (detect if Windows / Linux or something?)

const int state_n = 4; // state dimension


#if defined(MATLAB)
int main(int runtimeMs, REAL* startState)
{
#elif defined(ARDUINO)
// trying name change, linker problems as main already exists
//int main_avr( int argc, const char* argv[] )
int main_avr( int runtimeMs, REAL* startState)
{
#else
int main( int argc, const char* argv[] )
{
	DEBUG_PRINT("started!\n\r");

	int runtimeMs = 0;
	REAL startState[4] = {0.0, 0.0, 0.0, 0.0};
    REAL control_input[2] = {0.0,0.0};

	DEBUG_PRINT("Argc: %d\n\r", argc);

	if (argc < 6) {
		printf("Error: not enough input arguments!\n\r");
		return 0;
	}
	else {
		runtimeMs = atoi(argv[1]);
		startState[0] = atof(argv[2]);
		startState[1] = atof(argv[3]);
		startState[2] = atof(argv[4]);
		startState[3] = atof(argv[5]);
        control_input[0] = atof(argv[6]);
        control_input[1] = atof(argv[7]);
		DEBUG_PRINT("runtime: %d ms\n\rx_0[0]: %f\n\rx_0[1]: %f\n\rx_0[2]: %f\n\rx_0[3]: %f\n\ru_0[0]: %f\n\ru_0[1]: %f\n\r\n", runtimeMs, startState[0], startState[1], startState[2], startState[3],control_input[0],control_input[1]);
	}
#endif // linux


#ifdef DEBUG
#ifdef MATLAB
	mexPrintf("runtime = %d\n", runtimeMs);
	mexPrintf("x[0] = %f\nx[1] = %f\nx[2] = %f\nx[3] = %f\n", startState[0], startState[1], startState[2], startState[3]);
#endif
#endif

    REAL delta = control_input[1];
    REAL u = control_input[0];
    // simulate the car with a constant input passed from the command line
    getSimulatedSafeTime(startState,delta,u);
    printf("\n");

    // simTime 
    REAL timeToSafe = 2.0;

    // startMs 
    int startMs = milliseconds();
    
    // run reachability analysis test 
    bool safe = runReachability_bicycle(startState, timeToSafe, runtimeMs, startMs,delta,u);
	//int runtimeMs = 20; // run for 20 milliseconds

	// return value: 0 = unsafe by all, 1 = safe by simulation, 2 = safe by reachability
	//int safe = isSafe(runtimeMs, startState);

	DEBUG_PRINT("done, result = %s\n", safe ? "safe" : "unsafe");

	// http://www.mathworks.com/help/matlab/apiref/mexprintf.html
#ifdef DEBUG
#ifdef MATLAB
	mexPrintf("done, result = %s\n", safe ? "safe" : "unsafe"); // TODO: convert to macro to call either printf or mexPrintf
#endif
#endif

	return 0;
}

#ifdef MATLAB
/**
 * Main entry point: calls main(), but is the actual first function called when executing inside Matlab
 *
 */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    int i;

#ifdef DEBUG
    mexPrintf("nlhs: %d\n", nlhs);
    mexPrintf("nrhs: %d\n", nrhs);
#endif

    // example: edit([matlabroot '/extern/examples/mex/mexfunction.c']);

    // input example: http://www.mathworks.com/help/matlab/matlab_external/standalone-example.html
#ifdef DEBUG
    for (i=0; i<nrhs; i++)  {
        mexPrintf("\n\tInput Arg %i is of type:\t%s ",i,mxGetClassName(prhs[i]));
        mexPrintf("test: %f\n", mxGetScalar(prhs[i]));
    }
#endif

    REAL startState[4] = {mxGetScalar(prhs[1]), mxGetScalar(prhs[2]), mxGetScalar(prhs[3]), mxGetScalar(prhs[4])};

    bool result = 0;

    if (nrhs == 5) // TODO: make nicer, assumes set up for pendulum
    {
        result = main(mxGetScalar(prhs[0]), startState);
    }

    plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);

    REAL * output;

    output = mxGetPr(plhs[0]);

    *output = result;
	return;
}
#endif

