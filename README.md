# F1Tenth Rtreach

### compile the code like this: 

```
gcc -std=gnu99 -O3 -Wall  face_lift_bicycle_model.c geometry.c interval.c simulate_bicycle.c util.c  dynamics_bicycle_model.c  bicycle_main.c bicycle_model.c  -lm -o bicycle -DBICYCLE_MODEL_NONLINEAR
```

### Run a two second simulation using the following command

```
./bicycle 100 0.0 0.0 0.0 0.0 16.0 0.2666
```

### Plotting of Reachsets

compile the code: 

```
gcc -std=gnu99 -O3 -Wall  face_lift_bicycle_model.c geometry.c interval.c simulate_bicycle_plots.c util.c  dynamics_bicycle_model.c  bicycle_plots_main.c bicycle_model_plots.c -lm -o bicycle_plot -DBICYCLE_MODEL_NONLINEAR

```

Then: 

```
./bicycle_plot 100 2.0 0.0 0.0 0.0 0.0 16.0 0.2666
``` 
Usage: bicycle_plot (milliseconds-runtime) (seconds-reachtime) (x) (y) (linear velocity) (heading) (throttle control input) (heading control input)

and finally (assuming you have [gnuplot](http://gausssum.sourceforge.net/DocBook/ch01s03.html)):

```
gnuplot < plot_bicycle.gnuplot
```

# Building the code as a library so we can use it in ROS

Credit: [mix-c-and-cpp](https://www.thegeekstuff.com/2013/01/mix-c-and-cpp/)

First create a library of the code:

```
gcc -c -std=gnu99 -O3 -Wall  -fpic face_lift_bicycle_model.c geometry.c interval.c simulate_bicycle.c util.c  dynamics_bicycle_model.c bicycle_model.c  -lm -DBICYCLE_MODEL_NONLINEAR
```

Next create a shared library:

```
gcc -shared -o libRtreach.so face_lift_bicycle_model.o bicycle_model.o dynamics_bicycle_model.o geometry.o interval.o  simulate_bicycle.o util.o
```

this will create a file called libRtreach.so. Compile the test to make sure everything worked correctly.

```
g++ -L/home/musaup/Documents/Research/rtreach_f1tenth/src/ -Wall test.cpp -o test -lRtreach
```

```
$ export LD_LIBRARY_PATH=$(pwd):$LD_LIBRARY_PATH
$ ./test

The output should be: 

``` 
Quitting simulation: time: 2.020000, stepSize: 0.020000
If you keep the same input for the next 2.000000 s, the state will be: 
 [-1.143228,0.000000,-0.772037,0.000000]
```

```
### I need to refactor this code...

