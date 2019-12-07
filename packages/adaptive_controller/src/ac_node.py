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

        # V2: remove as much as possible all external params and replace them with self variables
        self.gamma = 1

        self.yp_k = np.asarray([0.0, 0.0])
        self.ym_k = self.yp_k
        self.t_k = rospy.Time.now()
        self.theta_hat_k = 0.0
        self.ref_k = np.asarray([0.0, 0.0])

        self.yp_k_minus = self.yp_k
        self.t_k_minus = self.t_k

        # MAKE IT CLEANER: queste variabili non servono necessariamente, se non per tenere traccia dell'evoluzione
        #self.theta_hat_k_minus = self.theta_hat_k
        #self.ref_k_minus = self.ref_k

        # Publication
        self.pub_corrected_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # Subscriptions, no tilde otherwise will sub to topic like ac_node/lane_pose which doesn't exist
        self.sub_lane_readin = rospy.Subscriber("lane_pose", LanePose, self.getPose, queue_size=1)
        self.sub_wheels_cmd_executed = rospy.Subscriber("ac_rif", Twist2DStamped, self.correctCommand, queue_size=1)
        self.sub_actuator_limits = rospy.Subscriber("actuator_limits", Twist2DStamped, self.actuator_limits_callback, queue_size=1)

        self.log("Initialized")
        #self.loginfo("========== Initialized AC node ==========")

    def getPose(self, poseMsg):
        d = poseMsg.d
        phi = poseMsg.phi

        self.yp_k = np.asarray([d, phi])
        self.t_k = poseMsg.header.stamp


    def correctCommand(self, car_cmd):

        # In here we want to do three things:
        #   1) Calculate Ts from the past two yp
        #   2) Predict current ym based on previus yp, reference and Ts
        #   3) Calculate e
        #   4) Update the Adaptation law (now up to present time)
        #   5) Retrieve car commands v and omega
        #   6) Compute corrected control command

        # start_time = rospy.Time.now()

        Ts = self.t_k - self.t_k_minus

        self.ym_k[0] = self.yp_k_minus[0] + self.ref_k[0] * Ts * math.sin(self.yp_k_minus[1] + self.ref_k[1] * Ts * 0.5)
        self.ym_k[1] = self.yp_k_minus[1] + self.ref_k[1] * Ts
        # self.ym_k[0] = self.yp_k_minus[0] + self.ref_k_minus[0] * Ts * math.sin(self.yp_k_minus[1] + self.ref_k_minus[1] * Ts * 0.5)
        # self.ym_k[1] = self.yp_k_minus[1] + self.ref_k_minus[1] * Ts

        e =  self.yp_k - self.ym_k

        theta_hat_k_d = - self.gamma * e[1]
        self.theta_hat_k = self.theta_hat_k + Ts * theta_hat_k_d
        # self.theta_hat_k = self.theta_hat_k_minus + Ts * theta_hat_k_d

        self.ref_k = np.asarray([car_cmd.v, car_cmd.omega])

        car_cmd_corrected.v = self.ref_k[0]
        car_cmd_corrected.omega = self.ref_k[1] + self.theta_hat_k

        self.pub_corrected_car_cmd.publish(car_cmd_corrected)

        self.yp_k_minus = self.yp_k
        self.t_k_minus = self.t_k

        # self.theta_hat_k_minus = self.theta_hat_k
        # self.ref_k_minus = self.ref_k

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
