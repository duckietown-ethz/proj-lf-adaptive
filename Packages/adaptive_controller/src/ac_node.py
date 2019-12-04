#!/usr/bin/env python

from os.path import basename, isfile, splitext
import rospy
import time

import numpy as np
import math

from duckietown import DTROS
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading

# If using the python control library, check dependencies_py.txt, I put it but not sure if it works
#import control

class AdaptiveControllerNode(DTROS):
    """Adaptive Controller
    This node implements an Model Reference Adaptive Controller (MRAC) for the Duckiebot.
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node
            that ROS will use

    Subscriber:
        ~ac_rif (:obj:`Twist2DStamped`): The car command as it is pubblished from the existing PID controller.
        ~lane_pose (:obj:`LanePose`): Lane pose es estimated in teh Lane Filter node.
        ~actuator_limits (:obj:'________'): --- justin case for the time being

    Publisher:
        ~car_cmd (:obj:'Twist2DStamped'): The input signal as computed from the control law of
            our adaptive controller.
    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(AdaptiveControllerNode, self).__init__(node_name=node_name)

        # Get robot name
        #self.veh_name = rospy.get_param('~robot_name') #another method
        self.veh_name = rospy.get_namespace().strip("/")

        # Get value for costant gamma
        #self.gamma = rospy.get_param("~gamma") first like that
        self.gamma = rospy.get_param("~gamma")

        self.initialized = False
        # Publication
        self.pub_corrected_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # Subscriptions, no tilde otherwise will sub to topic like ac_node/lane_pose which doesn't exist
        self.sub_lane_readin = rospy.Subscriber("lane_pose", LanePose, self.getPose, queue_size=1)
        self.sub_wheels_cmd_executed = rospy.Subscriber("ac_rif", Twist2DStamped, self.correctCommand, queue_size=1)
        self.sub_actuator_limits = rospy.Subscriber("actuator_limits", Twist2DStamped, self.actuator_limits_callback, queue_size=1)

        self.log("Initialized")

    def getPose(self, poseMsg):
        d = poseMsg.d
        phi = poseMsg.phi
        time =  poseMsg.header.stamp
        yp_lastPose = np.asarray([d, phi, time])

        rospy.set_param("~yp_k", yp_lastPose)

        #self.yp_lastPose = np.asarray([d, phi, time])


    def correctCommand(self, car_cmd):

        # In here we want to do three things:
        #   1) Calculate Ts from the past two yp
        #   2) Predict current ym based on previus yp, reference and Ts
        #   3) Calculate e
        #   4) Update the Adaptation law (now up to present time)
        #   5) Retrieve car commands v and omega
        #   6) Compute corrected control command

        # start_time = rospy.Time.now()

        if self.initialized == False:
            yp_pres = rospy.get_param("~yp_lastPose")
            yp_k = yp_pres[0:2]
            t_k = yp_pres[2]
            theta_hat_initialization_value = 0.0
            v = 0.0
            omega = 0.0
            car_cmd_msg.v = v
            car_cmd_msg.omega = omega

            # self.initialized = True

            rospy.set_param("~ym_k", yp_k)
            rospy.set_param("~time_k", t_k)
            rospy.set_param("~theta_hat_k", theta_hat_initialization_value)
            rospy.set_param("~ref_k", np.asarray([v, omega]))
            #rospy.set_param("~ref_k", [v, omega])
            #publish u = [0 , 0] wheel cmd

            self.initialized = True


        else:
            time_k_minus = rospy.get_param("~time_k")
            ym_k_minus = rospy.get_param("~ym_k")
            ref_k_minus = rospy.get_param("~ref_k")
            theta_hat_k_minus = rospy.get_param("~theta_hat_k")

            yp_pres = rospy.get_param("~yp_lastPose")
            yp_k = yp_pres[0:2]
            t_k = yp_pres[2]

            #   1) Calculate Ts from the past two yp
            Ts = t_k - t_k_minus
            #from lane-control they calculate Ts as image_delay = image_delay_stamp.secs + image_delay_stamp.nsecs / 1e9
            #self.log("Ts = %f" ,%()) #non mi ricordo

            #   2) Predict current ym based on previus yp, reference and Ts
            # Forward integration using second order method (Runge-Kutta), predicting present ym based on previous step v and omega
            ym_k_predicted[0] = ym_k_minus[0] + ref_k_minus[0] * Ts * math.sin(ym_k_minus[1] + ref_k_minus[1] * Ts * 0.5)
            ym_k_predicted[1] = ym_k_minus[1] + ref_k_minus[1] * Ts


            #   3) Calculate e
            # Having ym_pres_predicted and the lane pose yp_pres we can compute e=yp-ym of previous iteration
            e =  yp_k - ym_k_predicted #check error definition yp-ym or opposite

            # NOTE: for the one parameter only case, we can choose to use even only one of the two elements of
            # the error e

            #   4) Update the Adaptation law (now up to present time)
            # To solve for theta_hat a simple first order Forward Euler method is used (ADAPTATION LAW)
            theta_hat_k_d = - gamma * e[1]
            theta_hat_k = theta_hat + Ts * theta_hat_k_d

            #   5) Retrieve reference v and omega
            v = car_cmd.v
            omega = car_cmd.omega

            #   6) Compute corrected control command
            # Compute the corrected command (CONTROL LAW)
            '''
            self.km =
            slef.kp =
            self.u = # TODO - need to write a nice matrix form first
            rospy.set_param("~u", u)
            '''
            car_cmd_corrected.v = v
            car_cmd_corrected.omega = omega + theta_hat_k

            # Pubblish corrected car command
            self.pub_corrected_car_cmd.publish(car_cmd_corrected)

            # Set params for next function call
            rospy.set_param("~ym_k", yp_pres)
            rospy.set_param("~time_k", t_k)
            rospy.set_param("~theta_hat_k", theta_hat)
            rospy.set_param("~ref_k", np.asarray([v, omega]))


        # end_time = rospy.Time.now()
        # latency = end_time - start_time
        # self.log(rospy.Time.to_sec(latency))



    def actuator_limits_callback(self, msg):
        self.log('Actuator limit occurred')



if __name__ == '__main__':
    # Initialize the node
    AC_node = AdaptiveControllerNode(node_name='ac_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
