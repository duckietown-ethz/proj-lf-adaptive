# Packages

In this folder are collected all the implemented packages.

In particular:
* adaptive_controller: implements the adaptive controller which corrects the wheel command computed by the PI controller to
take into account the uncertainties in the kinematic parameters of the Duckiebot. It also estimate the adaptive controller internal parameter used to modify the input, everything while roaming around in the Duckietown.
* lane_control: a slightly changed version of the PI controller normally implemented on Duckiebots, such that
it includes a switch to turn on and off the Adaptive Controller.
* lane_filter and line_detector: changed to have performances more robust to noise 

The whole container is build on top on dt-core: adaptive_controller is the only completely new functionality, while
the others already exist in dt-core.
