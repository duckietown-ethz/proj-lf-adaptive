# Packages

In this folder are collected all the implemented packages.

In particular:
adaptive_controller: implements the adaptive controller which both corrects the inputs computed by the PID to
take account of uncertainties on the kinematic parameters of the Duckiebot, and estimates these
parameters on the fly while roaming the city.
lane_control: a slightly revised version of the PI controller normally implemented on Duckiebots, such that
it includes a switch to turn on and off the Adaptive Controller.

The whole container is build on top on dt-core: adaptive_controller is a fresh new functionality, while
lane_control substitutes the already existing package in dt-core.
