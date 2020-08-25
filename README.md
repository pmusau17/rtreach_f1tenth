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



### I need to refactor this code...

