
#include "util.h"
#include "face_lift.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#ifdef WIN32
#include "windows.h" // for GetTickCount
// TODO: this failed on my PC, it didn't detect as windows
#elif MX_COMPAT_32
#include "windows.h" // for GetTickCount
#else
	#if !defined(ARDUINO)
 // details on time structs, etc: http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
		#include <sys/time.h> // for gettimeofday
        #else
          #include <Arduino.h>
	#endif
#endif

// just for debug
static LiftingSettings errorPrintParams;
static bool errorParamsAssigned = false;

void set_error_print_params(LiftingSettings* set)
{
	errorParamsAssigned = true;
	errorPrintParams = *set;
}

void error_exit(const char* str)
{
	printf("Error: %s\n", str);

	// print the params that caused the error
	if (errorParamsAssigned)
	{
		printf("\nSettings:\n");
		printf("Reach Time = %f\n", errorPrintParams.reachTime);
		printf("Runtime = %i ms\n", errorPrintParams.maxRuntimeMilliseconds);
		printf("Init = ");
		println(&errorPrintParams.init);
	}
	else {
		printf("Error print params were not assigned.\n");
	}

	fflush(stdout);

	exit(1);
}


bool initialized = false;
long int startSec = 0;

long int milliseconds()
//int milliseconds()
{
#ifdef WIN32
	return (long)GetTickCount();
#else


#if !defined(ARDUINO)
//	static time_t startSec = 0;
	struct timeval now;
	gettimeofday(&now, NULL);
#endif

	if (!initialized)
	{
		initialized = true;
#if defined(ARDUINO)
                startSec = millis();
#else
		startSec = now.tv_sec;
#endif
	}

#if defined(ARDUINO)
        long int difSec = millis() - startSec;
        //long int ms = micros() / 1000;
        long int ds = difSec;
#else
	long int difSec = now.tv_sec - startSec;
	long int ms = now.tv_usec / 1000;
    long int ds = difSec * 1000 + ms;
#endif



	//DEBUG_PRINT("ms: %li\n\r", ds);

//	return difSec * 1000 + ms;
	return ds;
#endif
}
