### Autonomous Mobility on Demand - Fall 2019

<div figure-id="fig:header">
     <img src="media/duckietown_header.png" style='width: 20em'/>
</div>

# proj-lf-adaptive 

# Adaptive lane following

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


## Mission

The main goal of the project is to design an Adaptive Controller that allows to take into account the uncertainty on
kinematic parameters of a Duckiebot's model and identify the above-mentioned parameters while it roams around the city.

The objective is to identify reliably the trim of the bot after a short period of standard lane following starting from
an arbitrary initial value. This is done with the aim of getting rid of the operation of manual odometry calibration
that is nowadays periodically necessary to guarantee good performances from the bot.


# Structure

The folder `packages` groups together the packages that were introduced as new functionality, and the ones that were modified to tune the performances.

The folder `matlab_simulator` collects all the code necessary to run the simulator that was implemented to test different possible control architecture. More details can be found in the relative ReadMe.

Finally, in the folder `scripts` are defined some short scripts useful to quickly start the demo and perform some data analysis.


# Demo

## Duckiebot setup

All that is required is to have a Duckiebot whose camera as already been calibrated.

## Duckietown setup

No specific setup of the city is required, but to have faster convergence it is advisable to run the demo in a section with as many long straight segment as possible.

## Instructions

Clone the repository and move to the directory:

    git clone
    cd proj-lf-adaptive

Build the image on the Duckiebot by running the command:

    build command

Run the container with the command:

    run command


   

## Troubleshooting

