# Particle Filter
This was a project for the class 'Autonomous Driving' at TU Munich, held by Gereon Hinz, in the Summer Term 2017. The task was to visualize the functionality of the Particle Filter.
## Requirements
 - A c++ compiler (tested with AppleClang 8.0.0.8000042)
 - OpenCV libraries (tested with 2.4.13.2)
 - cmake (tested with 3.8.2)
## Build the application
In the directory this README is located, do:
```mkdir build
cd build
cmake ..
make
```
## Run the application with `./ParticleFilter`
This will open two windows: one with some information about the environment, such as number of particles, number of obstacles, measurement distance, and measurement noise.
The other window contains four trackbars to adjust these values.
Pressing 'q' quits the application, anything else starts the Particle Filter with the given variables.
Here 'q' also quits the current run, anything else executes the next step.