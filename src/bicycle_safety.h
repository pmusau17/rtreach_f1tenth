// Patrick Musau
// 09-2020
// Header file for safety checking of wall points


#ifndef BICYCLE_SAFETY_H_
#define BICYCLE_SAFETY_H_

#include <stdio.h>
#include <stdbool.h>
#include <string.h> 
#include <stdlib.h>
#include <sys/time.h>
#include "geometry.h"

// array that will store the points describing the wall
// number of rows, and columns in wall description file
extern double ** wallCoords;
extern int file_rows;
extern int file_columns;


void load_wallpoints(const char * filename,bool print);
void allocate_2darr(int rows,int columns);
void deallocate_2darr(int rows,int columns);
int countlines(const char * filename);
bool check_safety(HyperRectangle* rect, double (*box)[2]); 
bool check_safety_wall(HyperRectangle* rect);

#endif 
