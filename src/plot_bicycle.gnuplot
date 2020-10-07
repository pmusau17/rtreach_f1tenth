set terminal png font arial 16 size 500, 600
set output "reach_bicycle.png"

set title "Plotting of reachsets, Time= 7ms" font "arial,20"
set xlabel "x"
set ylabel "y"

# ranges for pendulum
set xrange [-3.3:6.4]
set yrange [-0.73:5.923]

load "bicycle_initial.gnuplot.txt"
load "bicycle_intermediate.gnuplot.txt"
load "bicycle_final.gnuplot.txt"

plot "bicycle_simulation.dat" using 1:2 with point
plot \
   1/0 lw 4 lc rgb 'blue' with lines t 'Initial', \
   1/0 lw 4 lc rgb 'dark-green' with lines t 'Intermediate', \
   1/0 lw 4 lc rgb 'red' with lines t 'Final'