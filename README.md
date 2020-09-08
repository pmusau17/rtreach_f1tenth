# F1Tenth Rtreach


### Real Time Reachability for the F1Tenth Platform 

This repo is an implementation of a runtime assurance approach by [Stanley Bak et al.](https://ieeexplore.ieee.org/document/7010482) for the F1Tenth platform. The motivation for runtime assurance stems from the ever-increasing complexity of software needed to control autonomous systems, and the need for these systems to be certified for safety and correctness. Thus the methods contained herein are used to build monitors for the system that can be used to ensure that the system remains within a safe operating mode. As an example, in the following animations we display a system with an unsafe neural network inspired controller that occasionally causes the f1tenth model to crash into walls. In the second animation, we add a real time safety monitor that switches to a safe controller when it detects a potential collision. Though the safety controller sacrifices performance it ensures that we do not collide with obstacles. The safety monitor was designed using the algorithms described by Bak et al. 


#### Neural Network Only (LEC)
![LEC_GIF](images/lec_only.gif)

#### Neural Network + Monitor + Safety Controller
![safety_node.gif](images/safety_node.gif)

**Disclaimer**: Our assumption is that you are using linux. A major part of this effort involves ROS. This code was tested on a computer running Ubuntu 16.04.6 LTS.

## Intro to rtreach: Let's start with an example. 

The safey monitor implemented in this repository relies on an anytime real-time reachability algorithm based on [mixed face-lifting](http://www.taylortjohnson.com/research/bak2014rtss.pdf). The reach-sets obtained using this method are represented as hyper-rectangles and we utilize these reachsets to check for collisons with obstacles in the vehicle's environment. The following example shows the use of this algorithm to check whether the vehicle will enter an unsafe operating mode in the next one second using the current control command. 

#### Before we continue let's first compile the example code by executing the following:  

```
$ cd src/
$ gcc -std=gnu99 -O3 -Wall  face_lift_bicycle_model.c geometry.c interval.c simulate_bicycle.c util.c  dynamics_bicycle_model.c  bicycle_safety.c main.c bicycle_model.c  -lm -o bicycle 
```
In this example, our vehicle is at the origin (x = 0, y = 0) with a current heading of 0 radians and an initial linear velocity of 0 m/s. The current control command being considered issues a speed set point of 1 m/s and a steering angle of 0.266 radians. Executing the code below will print whether or not the vehicle will enter an unsafe state. The alloted time that we have specified for reachability computations is 100ms. Since our technique is anytime it refines the precision of the reachability computation based on available runtime by halving the step size used in the face-lifting technique.

### Run a one second simulation using the following command

```
$ ./bicycle 100 0.0 0.0 0.0 0.0 1.0 0.2666
```

Expected output: 
```
runtime: 100 ms
x_0[0]: 0.000000
x_0[1]: 0.000000
x_0[2]: 0.000000
x_0[3]: 0.000000
u_0[0]: 1.000000
u_0[1]: 0.266600

Quitting simulation: time: 2.020000, stepSize: 0.020000
If you keep the same input for the next 2.000000 s, the state will be: 
 [1.545528,1.046166,1.283163,1.203507] 

Opening file...with 5536 points
offending cone (1.500000,2.500000) (1.500000, 2.500000)
unsafe....
Quitting from runtime maxed out
[HyperRectangle (1.527867, 1.529682) (1.032242, 1.033770) (1.280197, 1.280286) (1.188098, 1.188847)]
133ms: stepSize = 0.000391
iterations at quit: 10
done, result = safe
Done
```
As we can see our initial computation identified a collision with a cone in the vehicle's environment but by refining the reachset in successive iterations of the reachability computation, it becomes clear that this warning is spurious. 

### Plotting of Reachsets

We can visualize the results of the above example by executing the following: 

```
$ gcc -std=gnu99 -O3 -Wall  face_lift_bicycle_model.c geometry.c interval.c simulate_bicycle_plots.c util.c  dynamics_bicycle_model.c  bicycle_plots_main.c bicycle_model_plots.c -lm -o bicycle_plot 
```

 and then: 

```
$ ./bicycle_plot 100 2.0 0.0 0.0 0.0 0.0 16.0 0.2666
``` 

Finally, (assuming you have [gnuplot](http://gausssum.sourceforge.net/DocBook/ch01s03.html)), you can visualize the results by running: 
```
 $ gnuplot < plot_bicycle.gnuplot
```
Usage of plotting utilities: 

```
$ ./bicycle_plot (milliseconds-runtime) (seconds-reachtime) (x) (y) (linear velocity) (heading) (throttle control input) (heading control input)
```

### Building rtreach as a C library. 

Now that you have a taste of what rtreach is, we can move on to the more fun part. Using rtreach within ROS. By doing this, we can implement a safety monitor using the archtichture displayed again below: 

![Block Diagram](images/rtreach.png)

First compile the code, credit: [mix-c-and-cpp](https://www.thegeekstuff.com/2013/01/mix-c-and-cpp/):

```
$ gcc -c -std=gnu99 -O3 -Wall  -fpic face_lift_bicycle_model.c geometry.c interval.c simulate_bicycle.c util.c  dynamics_bicycle_model.c bicycle_safety.c bicycle_model.c  -lm 
```

Next create a shared library:

```
$ gcc -shared -o libRtreach.so face_lift_bicycle_model.o bicycle_model.o dynamics_bicycle_model.o geometry.o interval.o  simulate_bicycle.o util.o bicycle_safety.o 
```

This will create a file called **libRtreach.so**. 

Let's compile a test to make sure everything worked correctly.

```
$ g++ -L$(pwd)/ -Wall test.cpp -o test -lRtreach
```

Before running the executable make sure that the path of shared library is contain in the environment variable LD_LIBRARY_PATH.

```
$ export LD_LIBRARY_PATH=$(pwd):$LD_LIBRARY_PATH
$ ./test
``` 

The output of the test should be:

```
Quitting simulation: time: 2.020000, stepSize: 0.020000
If you keep the same input for the next 2.000000 s, the state will be: 
 [1.931151,0.000000,1.249570,0.000000] 
```

If that test worked, smile, take a breath and let's have some fun with ROS. If not feel free to send me an [email](mailto:patrick.musau@vanderbilt.edu) and we will see what we can do. 

### Using rtreach with the F1Tenth simulator

The platform that we seek to use these techniques on is a 1/10 scale autonomous race car named the [F1Tenth](https://f1tenth.org/). The platform was inspired as an international competition for researchers, engineers, and autonomous systems enthusiasts originally founded ath University of Pennsylvania in 2016. Our initial implmentation is done in simulation but we are also planning on doing this on the hardware platform. Thus, This assumes that you have the F1Tenth Simulator installed. If not please install it by following the instructions available [here](https://github.com/pmusau17/Platooning-F1Tenth).

Once that is installed, create the ros package: 

```
 cd ..
 mkdir -p ../rtreach_ros/src
```

Create the ros-nodes into the package created above:

```
cp -r ros_src/rtreach/ ../rtreach_ros/src/
```

Copy the rtreach shared library into the package:

```
cp src/libRtreach.so src/bicycle_safety.h src/geometry.h src/main.h src/dynamics_bicycle.h ../rtreach_ros/src/rtreach/src/
```
Build the ros-package.
```
cd ../rtreach_ros && catkin_make
```
and then: 

```
source devel/setup.bash
```

### Running Rtreach

Start the simulation. This will bring up the track displayed at the start of this readme and a green model of a simplistic autonomous vehicle.

```
roslaunch race rtreach.launch 
```

The neural network inspired controller that we use in our experiments maps images captured from the vehicle's camera into one of five discrete actions (turn left, turn right, continue straight, turn weakly left, turn weakly right). The network model used to make inferences is [VGG-7](https://towardsdatascience.com/only-numpy-implementing-mini-vgg-vgg-7-and-softmax-layer-with-interactive-code-8994719bcca8. The safe controller is a gap following algorithm that we select because of its ability to avoid obstacles. 


Run the safety monitor + safety_controller + neural network controller. 

```
rosrun rtreach reach_node_sync porto_obstacles.txt
```
In this setup the decision manager will allow the neural network model to control the vehicle so long as the control command issue will not cause the vehicle to enter an unsafe state in the next one second. Otherwise the safety controller will be used. The decision manager can then return to the neural network controller provided that the car has been in a safe operating mode for 20 control steps. 


To select a different set of weights for the neural network, you can specify the model .hdf5 in the [rtreach.launch](https://github.com/pmusau17/Platooning-F1Tenth/blob/master/src/race/launch/rtreach.launch) file. The available .hdf5 files are listed in the following [directory](https://github.com/pmusau17/Platooning-F1Tenth/tree/master/src/computer_vision/models). You are also free to train your own!

## Repository Organization

**ros_src/rtreach:** ros-package containing rtreach implementation.
- [reach_node_sync.cpp](ros_src/rtreach/src/reach_node_sync.cpp): ROS-node implementation of safety monitor and controller. 

**src:** C-implementation of rtreach.
- [dynamics_bicycle_model.c](src/dynamics_bicycle_model.c): Interval arithmetic implementation of a kinematic bicycle model for a car. Parameters are identified using [grey-box system identification](https://github.com/pmusau17/Platooning-F1Tenth/tree/master/src/race/sys_id).
- [interval.c](src/interval.c): Implementation of interval arithmetic methods.
- [geometry.c](src/geometry.c): Implementation of hyper-rectangle methods.
- [face_lift_bicycle_model.c](src/face_lift_bicycle_model.c): Facelifting method implementation with bicycle model dynamics.
- [bicycle_safety.c](src/bicycle_safety.c): Implementation of safety checking for the f1tenth model. Current checking includes static obstacles and collisions with walls.
- [simulate_bicycle.c](src/simulate_bicycle.c): Implementation of Euler simulation of kinematic bicycle model. 
- [simulate_bicycle_plots.c](src/simulate_bicycle_plots.c): Implementation of methods for plotting for reach sets.
- [bicycle_model.c](src/bicycle_model.c): Implementation of safety checking for f1tenth platform, makes use of the facelifting algorithms in [face_lift_bicycle_model.c](src/face_lift_bicycle_model.c).
- [bicycle_model_plots.c](src/bicycle_model_plots.c): Same as above but intented for plotting purposes.
- [util.c](src/util.c): Helper functions for timing and printing. 

#### Coming soon...
Dockerized implementation. Bug me if this doesn't happen soon.

