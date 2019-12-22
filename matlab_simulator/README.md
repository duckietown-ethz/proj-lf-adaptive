# MATLAB Simulator

This simulator was developed to test different control architecture and have a general idea of what results to expect.

To run the simulator just run the `main.m` file after setting the desired values for the initialization variables that can be found in the top part of the code.
To use the Adaptive Controller set the variable `adaptive_on = true`.  
After running the main a plot will show the position on the Duckiebot on the chosen map. In the bottom left corner you will find some options for showing other graphs and useful visualizationto at your convenience.

## Settings

### Initialization 

It is possible to choose an arbitrary starting position for the Duckiebot to start in, as long as it is close enough to the track chosen (some starting point have been left commented, to use as reference).
The bot's initial orientation determines in what direction the robot is going to follow the track.

### Maps

Three maps are provided for now:

1. `map = 0` is a simple 2m straight line. This is particularly useful to simulate teh odometry calibration test

2. `map = 1` is a complete track that contains both straight segments, right curves and left curves.

3. `map = 2` is a shorter test track consisting in one single "chicane", used mainly to quickly test different solutions.

### Noise

To simulate more realistically the noisy pose estimate provided by the camera on the real Duckiebot, it is possible to induce some noise in the knowledge that the car in our simulation has of the track: to do so set properly the variables ___

In the same way it is possible to simulate sudden changes in the kinematic parameters of the bot by activating the noise on these: to do so set the variables to the desired values.


