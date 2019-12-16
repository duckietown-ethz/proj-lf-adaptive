#!/usr/bin/env python

from os.path import basename, isfile, splitext
import rospy
import time

import numpy as np
import math

from duckietown import DTROS
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading

from scipy.ndimage.interpolation import shift

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
        # self.gamma = 0.05
        self.gamma = rospy.get_param("~gamma")
        # self.gamma = setupParameter("~gamma")

        self.error2use = rospy.get_param("~error2use")
        # self.error2use = setupParameter("~error2use")
        if not self.error2use : self.log("Using error on d")
        if self.error2use : self.log("Using error on phi")

        # Threshold to identify when param is converging
        self.thr_conv = 0.035

        self.yp_k = np.asarray([0.0, 0.0])
        self.ym_k = self.yp_k
        self.t_k = rospy.Time.now()
        self.theta_hat_k = 0.0
        self.ref_k = np.asarray([0.0, 0.0])
        self.e_k = np.asarray([0.0, 0.0])

        self.yp_k_minus = self.yp_k
        self.t_k_minus = self.t_k
        self.e_k_minus = self.e_k

        # self.car_cmd_corrected.v = 0.0
        # self.car_cmd_corrected.omega = 0.0

        # MAKE IT CLEANER: queste variabili non servono necessariamente, se non per tenere traccia dell'evoluzione
        #self.theta_hat_k_minus = self.theta_hat_k
        #self.ref_k_minus = self.ref_k

        # Make sure that that the commanded speed is in the allowable range
        self.omega_min = rospy.get_param("/" + self.veh_name +"/lane_controller_node/omega_min")
        self.omega_max = rospy.get_param("/" + self.veh_name + "/lane_controller_node/omega_max")

        # Initialize buffer of theta_hat: this is gonna be used to keep track of its behavior
        self.buffer = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0])

        # Publication
        self.pub_corrected_car_cmd = rospy.Publisher("lane_controller_node/car_cmd", Twist2DStamped, queue_size=1)

        # Subscriptions, no tilde otherwise will sub to topic like ac_node/lane_pose which doesn't exist
        self.sub_lane_readin = rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.getPose, queue_size=1)
        self.sub_wheels_cmd_executed = rospy.Subscriber("lane_controller_node/ac_rif", Twist2DStamped, self.correctCommand, queue_size=1)
        self.sub_actuator_limits = rospy.Subscriber("lane_controller_node/actuator_limits", Twist2DStamped, self.actuator_limits_callback, queue_size=1)

        self.log("Initialized")

    def getPose(self, poseMsg):
        d = poseMsg.d
        phi = poseMsg.phi

        self.yp_k = np.asarray([d, phi])
        self.t_k = poseMsg.header.stamp


    def correctCommand(self, car_cmd):

        # In here we want to do three things:
        #   1) Compute Ts from the past two yp
        #   2) Predict current ym based on previus yp, reference and Ts
        #   3) Calculate e
        #   4) Update the Adaptation law (now up to present time)
        #   5) Compute corrected control command

        self.log("==============================================")

        start_time = rospy.Time.now()

        # (1) : Compute sampling time
        Ts = self.t_k - self.t_k_minus
        #Ts = rospy.Time.to_sec(Ts)
        Ts = Ts.to_sec()
        self.log("Ts : %f" % Ts)

        # Car message from PI controller
        # self.ref_k = np.asarray([car_cmd.v, car_cmd.omega])

        # Init message
        car_cmd_corrected = Twist2DStamped()

        if (Ts > 0.005) and (Ts < 0.5):

            # Check for update on gamma
            self.gamma = rospy.get_param("~gamma")
            self.error2use = rospy.get_param("~error2use")

            # The error e is proportional on the sampling time Ts, so the multiplicative constant gamma that multiply
            # e to obtain the correction of the inputs to the motors should be corrected by some factor of Ts
            gamma = self.gamma / Ts

            # (2) : Predict current ym based on previus yp, reference and Ts
            self.ym_k[0] = self.yp_k_minus[0] + self.ref_k[0] * Ts * math.sin(self.yp_k_minus[1] + self.ref_k[1] * Ts * 0.5)
            self.ym_k[1] = self.yp_k_minus[1] + self.ref_k[1] * Ts
            # self.ym_k[0] = self.yp_k_minus[0] + self.ref_k_minus[0] * Ts * math.sin(self.yp_k_minus[1] + self.ref_k_minus[1] * Ts * 0.5)
            # self.ym_k[1] = self.yp_k_minus[1] + self.ref_k_minus[1] * Ts

            # (3) : Calculate e
            self.e_k =  self.yp_k - self.ym_k

            self.log("omega rif : %f" % car_cmd.omega)
            self.log("error : %f" % self.e_k[1])

            # Check variance of pose
            delta_e = self.e_k -self.e_k_minus
            self.log("delta on error : %f" % delta_e[1])

            # Upper bounds for reasonable delta_e
            ub = np.asarray([self.ref_k[0]*Ts*2, self.omega_max*Ts*2])
            # ub_d = self.ref_k[0]*Ts*2       # worst case scenario: bot moving perpendiculary to lane
            # ub_phi = self.omega_max*Ts*2    # bound if the angle changed too much


            # We want to avoid updating theta_hat when:
            #   1 - the returned lane pose is too far from previuos enstimate (probably unreliable)
            #   2 - when in curves (recgnize curves based on angular speed)
            cond_on_err  = (np.absolute(delta_e) < ub)
            if (np.any(cond_on_err)) and (car_cmd.omega < 2) :

                # (4) : Update the Adaptation law
                theta_hat_k_d = - self.gamma * self.e_k[self.error2use]
                self.theta_hat_k = self.theta_hat_k + Ts * theta_hat_k_d
                # self.theta_hat_k = self.theta_hat_k_minus + Ts * theta_hat_k_d

                self.log("theta_hat_k : %f" % self.theta_hat_k)

                # (5) : Compute corrected control command
                car_cmd_corrected.v = self.ref_k[0]
                car_cmd_corrected.omega = self.ref_k[1] + self.theta_hat_k

            else:
                car_cmd_corrected.v = self.ref_k[0]
                car_cmd_corrected.omega = self.ref_k[1]

                self.log("sample rejected!")

            # self.theta_hat_k_minus = self.theta_hat_k
            # self.ref_k_minus = self.ref_k


        else:
            # If the sampling time between two frames is to short, ignore the sample as it
            # probably is unreliable.

            car_cmd_corrected.v = car_cmd.v
            car_cmd_corrected.omega = car_cmd.omega

            self.log("sample rejected!")

        # Make sure omega is in the allowe range
        if car_cmd_corrected.omega > self.omega_max: car_cmd_corrected.omega = self.omega_max
        if car_cmd_corrected.omega < self.omega_min: car_cmd_corrected.omega = self.omega_min

        # Pubblish corrected command
        #car_cmd_corrected.v = 0.1
        self.pub_corrected_car_cmd.publish(car_cmd_corrected)

        # Update variables for next iteration
        self.yp_k_minus = self.yp_k
        self.t_k_minus = self.t_k
        self.e_k_minus = self.e_k

        self.ref_k = np.asarray([car_cmd.v, car_cmd.omega])


        # Update buffer of theta_hat
        self.buffer = shift(self.buffer, 1, cval=self.theta_hat_k)

        # If theta hat is converging, then slowly reduce gamma
        # if ((np.amax(self.buffer) - np.amin(self.buffer) < self.thr_conv)) and (self.gamma>0.001) :
        #     self.gamma = rospy.set_param("~gamma", self.gamma*0.6)
        #     self.thr_conv = self.thr_conv*0.6


        # For similar reason as above, we want to increase gamma in case theta_hat start converging to a very
        #   different value from before (for instance because of a bump):
        # if ((np.amax(self.buffer) - np.amin(self.buffer) > 2.5*self.thr_conv)) and (self.gamma<15) :
        #     self.gamma = rospy.set_param("~gamma", self.gamma*1.5)
        #     self.thr_conv = self.thr_conv*1.8


        end_time = rospy.Time.now()
        latency = end_time - start_time
        # self.log("latency : %f" % rospy.Time.to_sec(latency))
        self.log("latency : %f" % latency.to_sec())


    def actuator_limits_callback(self, msg):
        self.log('Actuator limit occurred')

    # def setupParameter(self,param_name):
    #     value = rospy.get_param(param_name)
    #     rospy.set_param(param_name,value)   # Write to parameter server for transparancy
    #     #rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
    #     return value



if __name__ == '__main__':
    # Initialize the node
    AC_node = AdaptiveControllerNode(node_name='ac_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
