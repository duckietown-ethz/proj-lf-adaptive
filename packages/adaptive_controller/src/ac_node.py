#!/usr/bin/env python

from os.path import basename, isfile, splitext
import rospy
import time
import os
import yaml
import numpy as np
import math

from duckietown import DTROS
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading

from scipy.ndimage.interpolation import shift

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
        self.veh_name = rospy.get_namespace().strip("/")

        self.gamma = rospy.get_param("~gamma") #default 0.1

        self.error2use = rospy.get_param("~error2use")
        #if not self.error2use : self.log("Using error on d")
        #if self.error2use : self.log("Using error on phi")

        # Threshold to identify when param is converging
        self.thr_conv = 0.035

        self.lane_pose_k_minus = np.asarray([0.0, 0.0])
        self.t_lane_pose_k_minus = rospy.Time.now().to_sec()
        self.lane_pose_k = np.asarray([0.0, 0.0])
        self.t_lane_pose_k = rospy.Time.now().to_sec()
        self.lane_pose_b_predicted = np.asarray([0.0, 0.0])
        self.lane_pose_k_predicted = np.asarray([0.0, 0.0])

        self.theta_hat_k = 0.0
        self.ac_rif_k = np.asarray([0.0, 0.0])
        self.ac_rif_k_minus = np.asarray([0.0, 0.0])
        self.ac_rif_k_minus_2 = np.asarray([0.0, 0.0])
        self.err_k = np.asarray([0.0, 0.0])
        self.err_k_minus = np.asarray([0.0, 0.0]) 

        # Make sure that that the commanded speed is in the allowable range
        self.omega_min = rospy.get_param("/" + self.veh_name +"/lane_controller_node/omega_min")
        self.omega_max = rospy.get_param("/" + self.veh_name + "/lane_controller_node/omega_max")

        # Initialize buffer of theta_hat: this is gonna be used to keep track of its behavior
        self.buffer = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0])

        # Publication
        self.pub_corrected_car_cmd = rospy.Publisher("lane_controller_node/car_cmd", Twist2DStamped, queue_size=1)

        # Subscriptions, no tilde otherwise will sub to topic like ac_node/lane_pose which doesn't exist
        self.sub_lane_readin = rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.getPose, queue_size=1)
        self.sub_car_cmd_ac_rif = rospy.Subscriber("lane_controller_node/ac_rif", Twist2DStamped, self.correctCommand, queue_size=1)
        self.sub_actuator_limits = rospy.Subscriber("lane_controller_node/actuator_limits", Twist2DStamped, self.actuator_limits_callback, queue_size=1)
        
        self.log("Initialized")
    
    def getPose(self, poseMsg):
        self.lane_pose_k = np.asarray([poseMsg.d, poseMsg.phi])

    def correctCommand(self, car_cmd):

        # In here we want to do three things:
        #   1) Compute Ts from the past two yp
        #   2) Predict current ym based on previus yp, reference and Ts
        #   3) Calculate e
        #   4) Update the Adaptation law (now up to present time)
        #   5) Compute corrected control command

        self.log("======================================")


        self.t_lane_pose_k = car_cmd.header.stamp
        self.t_lane_pose_k = self.t_lane_pose_k.to_sec()

        # (1) : Compute sampling time
        
        Ts = self.t_lane_pose_k - self.t_lane_pose_k_minus
        self.log("Ts: %f" % Ts)

        # Car message from PI controller
        self.ac_rif_k = np.asarray([car_cmd.v, car_cmd.omega])

        # Init message
        car_cmd_corrected = Twist2DStamped()

        # Check for update on gamma
        self.gamma = rospy.get_param("~gamma")
        
        # (2) : Predict current ym based on previus yp, reference and Ts
        
        if Ts >0.025 and Ts < 0.2:
        	self.lane_pose_k_predicted[0] = self.lane_pose_k_minus[0] + self.ac_rif_k_minus[0] * Ts * math.sin(self.lane_pose_k_minus[1] + self.ac_rif_k_minus[1] * Ts * 0.5)
	        self.lane_pose_k_predicted[1] = self.lane_pose_k_minus[1] + self.ac_rif_k_minus[1] * Ts

	        # (3) : Calculate e
	        self.err_k =  self.lane_pose_k - self.lane_pose_k_predicted

	        #self.log("omega rif : %f" % car_cmd.omega)
	        self.log("error on d: %f" % self.err_k[0])
	        self.log("error on phi: %f" % self.err_k[1])

	        # Check variance of pose
	        delta_e = self.err_k -self.err_k_minus
	        #self.log("delta on error : %f" % delta_e[1])

	        # Upper bounds for reasonable delta_e
	        ub = np.asarray([self.ac_rif_k[0]*Ts*2, self.omega_max*Ts*2])
	        
	        # We want to avoid updating theta_hat when:
	        #   1 - the returned lane pose is too far from previuos enstimate (probably unreliable)
	        #   2 - when in curves (recgnize curves based on angular speed)
	        cond_on_err  = (np.absolute(delta_e) < ub)
	        if (np.any(cond_on_err)) and (car_cmd.omega < 2) :

	            # (4) : Update the Adaptation law
	            theta_hat_k_d = - self.gamma * self.err_k[self.error2use] #default 0, use error on d
	            self.theta_hat_k = self.theta_hat_k + Ts * theta_hat_k_d
	            self.log("gamma : %f" % self.gamma)
	            self.log("theta_hat_k : %f" % self.theta_hat_k)
	        else:
	        	self.log("theta not updated!")

	    # (5) : Compute corrected control command
        car_cmd_corrected.v = self.ac_rif_k[0]
        car_cmd_corrected.omega = self.ac_rif_k[1] + self.theta_hat_k
        # Make sure omega is in the allowe range
        if car_cmd_corrected.omega > self.omega_max:
        	car_cmd_corrected.omega = self.omega_max
        if car_cmd_corrected.omega < self.omega_min: 
        	car_cmd_corrected.omega = self.omega_min

        # Publish corrected command
        car_cmd_corrected.header.stamp = car_cmd.header.stamp

        self.pub_corrected_car_cmd.publish(car_cmd_corrected)

        # Update buffer of theta_hat
        self.buffer = shift(self.buffer, 1, cval=self.theta_hat_k)

        # If theta hat is converging, then slowly reduce gamma
        if ((np.amax(self.buffer) - np.amin(self.buffer) < self.thr_conv)) and (self.gamma>0.001) :
            self.gamma = rospy.set_param("~gamma", self.gamma*0.6)
            self.thr_conv = self.thr_conv*0.6


        # For similar reason as above, we want to increase gamma in case theta_hat start converging to a very
        #   different value from before (for instance because of a bump):
        if ((np.amax(self.buffer) - np.amin(self.buffer) > 10*self.thr_conv)) and (self.gamma<15) :
            self.gamma = rospy.set_param("~gamma", self.gamma*1.5)
            self.thr_conv = self.thr_conv*1.8

        # Update variables for next iteration
        self.lane_pose_k_minus = self.lane_pose_k
        self.t_lane_pose_k_minus = self.t_lane_pose_k
        self.err_k_minus = self.err_k
        self.ac_rif_k_minus_2 = self.ac_rif_k_minus
        self.ac_rif_k_minus = self.ac_rif_k
        self.gamma = rospy.set_param("~gamma", 0.1)
        
        
    def actuator_limits_callback(self, msg):
        self.log('Actuator limit occurred')

    def onShutdown(self):
        #Shutdown procedure.
        #Save yaml file with new trim value.
        
        #INITIALIZATION FOR YAML FILE SAVE
        # Add the node parameters to the parameters dictionary and load their default values
        self.parameters = {}

        # Set parameters using a robot-specific yaml file if such exists
        self.parameters = self.readParamFromFile()
        
        self.SaveCalibration() #create new calibration file

        super(AdaptiveControllerNode, self).onShutdown()

    def SaveCalibration(self):
        """
        Saves the current kinematics paramaters to a robot-specific file at
        `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`.
        Args:
            None
        """
        v_bar_param_name = "/" + self.veh_name + "/lane_controller_node/v_bar"
        v_bar = rospy.get_param(v_bar_param_name)

      	new_trim = - (np.asarray(self.theta_hat_k)*np.asarray(self.parameters['baseline'])*np.asarray(self.parameters['gain']))/(2.0*np.asarray(v_bar))
        
        # Write to a yaml file
        new_trim = self.parameters['trim'] + round(new_trim.astype(float),4)
        if new_trim > 0.25 or new_trim < -0.25 :
        	new_trim = 0.0
        trim_param_name = "/" + self.veh_name + "/kinematics_node/trim"     
        rospy.set_param(trim_param_name, new_trim)

        data = {
            "calibration_time": time.strftime("%Y-%m-%d-%H-%M-%S"),
            "gain": self.parameters['gain'],
            "trim": new_trim,
            "baseline": self.parameters['baseline'],
            "radius": self.parameters['radius'],
            "k": self.parameters['k'],
            "limit": self.parameters['limit'],
            "v_max": 1.0,
            "omega_max": 8.0,
        }

        # Write to file
        file_name = self.getFilePath(self.veh_name)
        with open(file_name, 'w') as outfile:
            outfile.write(yaml.dump(data, default_flow_style=False))

        # Printout
        saved_details = "[gain: %s\t trim: %s\t baseline: %s\t radius: %s\t k: %s\t limit: %s\t omega_max: %s\t v_max: %s\t]" % \
        (self.parameters['gain'], self.parameters['trim'], self.parameters['baseline'], self.parameters['radius'],
         self.parameters['k'], self.parameters['limit'], self.parameters['omega_max'], self.parameters['v_max'])
        self.log("Saved kinematic calibration to %s with values: %s" % (file_name, saved_details))

    def readParamFromFile(self):
        """
        Reads the saved parameters from `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml` or
        uses the default values if the file doesn't exist. Adjsuts the ROS paramaters for the node
        with the new values.
        """
        # Check file existence
        fname = self.getFilePath(self.veh_name)
        # Use the default values from the config folder if a robot-specific file does not exist.
        if not isfile(fname):
            self.log("Kinematics calibration file %s does not exist! Using the default file." % fname, type='warn')
            fname = '/data/config/calibrations/kinematics/default.yaml'

        
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s" %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return
        
        return yaml_dict

    def getFilePath(self, name):
        """
        Returns the path to the robot-specific configuration file,
        i.e. `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`.
        Args:
            name (:obj:`str`): the Duckiebot name
        Returns:
            :obj:`str`: the full path to the robot-specific calibration file
        """
        cali_file_folder = '/data/config/calibrations/kinematics/'
        cali_file = cali_file_folder + name + ".yaml"
        return cali_file



if __name__ == '__main__':
    # Initialize the node
    AC_node = AdaptiveControllerNode(node_name='ac_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
