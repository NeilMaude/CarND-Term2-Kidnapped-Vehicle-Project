# Overview
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

## Project Introduction
This project implements a 2 dimensional particle filter in C++. The particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data. 

## Notes on Solution
The code implements the particle filter and meets the required accuracy and timing criteria.

Note: the code uses 100 particles for the filter, which feels like a low number.

With 100 particles, the test observation set completes in 18 seconds (Dell laptop, i7-6820).
The accuracy (cumulative mean weighted error) is x=0.11373, y=0.107113, yaw=0.00379022.

Re-running the code with 1000 particles (which takes 112 seconds), gives a slightly better result.
The accuracy with 1000 particles becomes (CMWE): x=0.109336, y=0.0993035, yaw=0.00346569.

However, the improvement in accuracy is: x=0.004394, y=0.0078095, yaw=0.00032453.

So this increase in computation time (10x number of particles) gives a real world positional accuracy of less than 1cm.
(The units of x and y are meters.)  Yaw is in radians, but the improvement is fractions of a degree of orientation accuracy.

This indicates that there is no value from increasing the number of particles used by the filter.

## Running the Code
Once you have this repository on your machine, `cd` into the repository's root directory and run the following commands from the command line:

```
> ./clean.sh
> ./build.sh
> ./run.sh
```

> **NOTE**
> If you get any `command not found` problems, you will have to install 
> the associated dependencies (for example, 
> [cmake](https://cmake.org/install/))

If everything worked you should see something like the following output:

Time step: 2444
Cumulative mean weighted error: x .1 y .1 yaw .02
Runtime (sec): 38.187226
Success! Your particle filter passed!

```
Otherwise you might get
.
.
.
Time step: 100
Cumulative mean weighted error: x 39.8926 y 9.60949 yaw 0.198841
Your x error, 39.8926 is larger than the maximum allowable error, 1
```

Your job is to build out the methods in `particle_filter.cpp` until the last line of output says:

```
Success! Your particle filter passed!
```

# Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   control_data.txt
|   |   gt_data.txt
|   |   map_data.txt
|   |
|   |___observation
|       |   observations_000001.txt
|       |   ... 
|       |   observations_002444.txt
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

The code for the particle filter is in `particle_filter.cpp` in the `src` directory. 

The `src/main.cpp` file contains the code that will actually be running your particle filter and calling the associated methods.

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory. 

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

> * Map data provided by 3D Mapping Solutions GmbH.


#### Control Data
`control_data.txt` contains rows of control data. Each row corresponds to the control data for the corresponding time step. The two columns represent
1. vehicle speed (in meters per second)
2. vehicle yaw rate (in radians per second)

#### Observation Data
The `observation` directory includes around 2000 files. Each file is numbered according to the timestep in which that observation takes place. 

These files contain observation data for all "observable" landmarks. Here observable means the landmark is sufficiently close to the vehicle. Each row in these files corresponds to a single landmark. The two columns represent:
1. x distance to the landmark in meters (right is positive) RELATIVE TO THE VEHICLE. 
2. y distance to the landmark in meters (forward is positive) RELATIVE TO THE VEHICLE.

> **NOTE**
> The vehicle's coordinate system is NOT the map coordinate system. 

## Success Criteria
The two things the grading code is looking for are:

1. **Accuracy**: your particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` (maximum allowed error in x or y) and `max_yaw_error` in `src/main.cpp`.
2. **Performance**: your particle filter should complete execution within the time specified by `max_runtime` in `src/main.cpp`.



