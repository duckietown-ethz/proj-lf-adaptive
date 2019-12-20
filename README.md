### Autonomous Mobility on Demand - Fall 2019

<div figure-id="fig:header">
     <img src="media/duckietown_header.png" style='width: 20em'/>
</div>

# proj-lf-adaptive 

# Adaptive lane following {#demo-adaptive-controller status=ready}

> **"Ab uno disce omnes"**  <br />
> "From one learn it all" 

Authors | Contact | Mentors | Supervisor
------- | ------- | ------- | -----------
Pietro Griffa | griffap@student.ethz.ch | Rohit Suri | Jacopo Tani
Simone Arreghini | arsimone@student.ethz.ch | Aleksandar Petrov | 

Authors:
* Pietro Griffa          &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; griffap@student.ethz.ch
* Simone Arreghini       &nbsp; &nbsp; &nbsp; &nbsp; arsimone@student.ethz.ch

Mentors:
* Rohit Suri
* Aleksandar Petrov

Supervisor:
* Jacopo Tani


## Mission {#demo-adaptive-controller-mission status=ready}

The goal of the project is to design an Adaptive Controller that allows the estimation of the trim kinematic parameter of a Duckiebot, everything done while performing lane following driving around the city.

The objective is then to identify reliably the trim of a Duckiebot after a short period of standard lane following, starting from an arbitrary initial value. This is done with the aim of getting rid of the manual odometry calibration which is now a  periodic necessary practice to have an acceptable behavior from a Duckiebot.


# Structure {#demo-adaptive-controller-structure status=ready}

The folder `packages` groups together the Adaptive Controller package that has been introduced as a new functionality, and others that have been modified to improve the performances of the lane following.

The folder `matlab_simulator` collects all the code necessary to run the simulator that was implemented to test different possible control architecture. More details can be found in the relative ReadMe.

Finally, in the folder `scripts` there are some scripts that can be used as shortcuts to quickly start the demo and perform some data analysis.


# Demo {#demo-adaptive-controller-demo status=ready}

<div class='requirements' markdown="1">

Requires: Laptop configured, according to [Unit C-1 - Laptop Setup](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/laptop_setup.html).

Requires: A Duckiebot in DB18 configuration.

Requires: You have configured the Duckiebot as documented in [Unit C-5 - Duckiebot Initialization](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/setup_duckiebot.html).

Requires: [Camera calibration](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/camera_calib.html) completed.

Results: Make you duckiebot self-calibrate.

</div>

## Duckiebot setup {#demo-adaptive-controller-duckiebot-setup status=ready}

All that is required is to have a Duckiebot whose camera as already been calibrated as described in [Camera calibration](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/camera_calib.html).

## Duckietown setup {#demo-adaptive-controller-duckietown-setup status=ready}

No specific setup of the city is required. However, to have faster convergence, it is advisable to run the demo in a city map with as many long straight segment as possible.
The lane following pipeline used, likewise the standard lane following, does not take into account intersections therefore its behavior will be unpredictable in their presence and might cause the Duckiebot to go out of the lane.

## Instructions {#demo-adaptive-controller-demo-instruction status=ready}

Clone the repository and move to the directory:

    git clone
    cd proj-lf-adaptive

Build the image on the Duckiebot by running the command:

    build command

Run the container with the command:

    run command


   

## Troubleshooting {#demo-adaptive-controller-troubleshooting status=ready}

