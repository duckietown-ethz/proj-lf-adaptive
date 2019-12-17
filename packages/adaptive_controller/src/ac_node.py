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
from sensor_msgs.msg import Joy

from scipy.ndimage.interpolation import shift

import message_filters


class AdaptiveControllerNode(DTROS):
    """Adaptive Controller
    This node implements an Model Reference Adaptive Controller (MRAC) for the Duckiebot.
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node
            that ROS will use
    Subscriber:
        ~ac_rif (:obj:`Twist2DStamped`): The car command as it is pubblished from the existing PID controller.
        ~lane_pose (:obj:`LanePose`): Lane pose es estimated in teh Lane Filter node.
    Publisher:
        ~car_cmd (:obj:'Twist2DStamped'): The input signal as computed from the control law of
            our adaptive controller.
        ~joystick_override (:obj:`duckietown_msgs/BoolStamped`): Boolean that is used to control whether
           lane-following or joystick control is on
    """

    def __init__(self, node_name):

        self.first_iteration = True
        self.stage = 0

        # Initialize the DTROS parent class
        super(AdaptiveControllerNode, self).__init__(node_name=node_name)
        self.i = 0 
        # Get robot name
        self.veh_name = rospy.get_namespace().strip("/")

        self.error2use = rospy.get_param("~error2use")
        self.trim_param_name = "/" + self.veh_name + "/kinematics_node/trim"
        self.trim_actual = rospy.get_param(self.trim_param_name)

        self.thresholds = [0.04, 0.035, 0.030]

        if not self.error2use : 
            self.log("Using error on d")
            self.gammas = [5.0, 3.0, 2.0]
            #self.gamma = rospy.set_param("~gamma",self.gammas[0]) 
            self.gamma = self.gammas[self.stage]
        if self.error2use : 
            self.log("Using error on phi")
            self.gammas = [0.8, 0.6, 0.4]
            #self.gamma = rospy.set_param("~gamma",self.gammas[0]) 
            self.gamma = self.gammas[self.stage]
            
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

        # Initialize past_theta_hats: this is gonna be used to keep track of of theta_hats' behavior
        self.past_theta_hats = np.zeros(50)

        # Publication
        self.pub_corrected_car_cmd = rospy.Publisher("lane_controller_node/car_cmd", Twist2DStamped, queue_size=1)
        self.pub_stop_cmd = rospy.Publisher("joy", Joy, queue_size=1)
        
        # Subscriptions, no tilde otherwise will sub to topic like ac_node/lane_pose which doesn't exist

        pose_sub = message_filters.Subscriber("lane_filter_node/lane_pose", LanePose)
        cmd_sub = message_filters.Subscriber("lane_controller_node/ac_rif", Twist2DStamped)

        ts = message_filters.TimeSynchronizer([pose_sub, cmd_sub], 10)
        ts.registerCallback(self.correctCommand)
        self.display_param()

        self.log("Initialized")

    def correctCommand(self, poseMsg, car_cmd):
        # In here we want to do three things:
        #   1) Compute Ts from the past two yp
        #   2) Predict current ym based on previus yp, reference and Ts
        #   3) Calculate e
        #   4) Update the Adaptation law (now up to present time)
        #   5) Compute corrected control command
        self.t_start = rospy.Time.now().to_sec()
        #Syncronization between topics
        assert car_cmd.header.stamp == poseMsg.header.stamp

        self.log("==============================")

        self.lane_pose_k = np.asarray([poseMsg.d, poseMsg.phi])

        self.t_lane_pose_k = car_cmd.header.stamp.to_sec()

        #Initialize at first iteration
        if self.first_iteration:
            self.first_iteration = False
            self.t_lane_pose_k_minus = car_cmd.header.stamp.to_sec()
            self.ac_rif_k_minus = np.asarray([car_cmd.v, car_cmd.omega])
            self.t_lane_pose_k_minus = self.t_lane_pose_k
            return

        # self.t_lane_pose_k = self.t_lane_pose_k.to_sec()

        # (1) : Compute sampling time
        Ts = self.t_lane_pose_k - self.t_lane_pose_k_minus
        self.log("Ts: %f" % Ts)

        # Car message from PI controller
        self.ac_rif_k = np.asarray([car_cmd.v, car_cmd.omega])

        # Init message
        car_cmd_corrected = Twist2DStamped()

        #filter out TS too big or too small
        #do not update theta in curves omega accepted -1.0<omega< +1.0
        if (Ts >0.025) and (Ts < 0.25) and (car_cmd.omega < 1.5) and (car_cmd.omega > -1.5) :

            # (2) : Predict current ym based on previus yp, reference and Ts
            self.lane_pose_k_predicted[0] = self.lane_pose_k_minus[0] + self.ac_rif_k_minus[0] * Ts * math.sin(self.lane_pose_k_minus[1] + self.ac_rif_k_minus[1] * Ts * 0.5)
            self.lane_pose_k_predicted[1] = self.lane_pose_k_minus[1] + self.ac_rif_k_minus[1] * Ts

            # (3) : Calculate e        self.t_lane_pose_k = car_cmd.header.stamp.to_sec()
            self.err_k[0] =  self.lane_pose_k[0] - self.lane_pose_k_predicted[0]
            self.err_k[1] =  self.lane_pose_k[1] - self.lane_pose_k_predicted[1]

            self.log("actual d: %f" % self.lane_pose_k[0])
            self.log("predicted d: %f" %  self.lane_pose_k_predicted[0])
            self.log("error on d: %f" % self.err_k[0])
            self.log("actual phi: %f" % self.lane_pose_k[1])
            self.log("predicted phi: %f" % self.lane_pose_k_predicted[1])
            self.log("error on phi: %f" % self.err_k[1])
            
            delta_e = self.err_k -self.err_k_minus
            # Upper bounds for reasonable delta_e
            upper_bound = np.asarray([self.ac_rif_k[0]*Ts*2, self.omega_max*Ts*2])

            # We want to avoid updating theta_hat when the returned lane pose is too far from previuos estimate (probably unreliable)
            cond_on_err  = (np.absolute(delta_e) < upper_bound)
            if (np.any(cond_on_err)) :

                # Check for update on gamma
                self.error2use = rospy.get_param("~error2use")
                if not self.error2use : 
                    self.gammas = [4.0, 3.0, 2.0]
                if self.error2use : 
                    self.gammas = [0.8, 0.6, 0.4]


                self.gamma = self.gammas[self.stage]
                # (4) : Update the Adaptation law
                theta_hat_k_d = - self.gamma * self.err_k[self.error2use] #default 0, use error on d
                self.theta_hat_k = self.theta_hat_k + Ts * theta_hat_k_d
                self.log("gamma : %f" % self.gamma)
                self.log("theta_hat_k : %f" % self.theta_hat_k)
                self.past_theta_hats = shift(self.past_theta_hats, 1, cval=self.theta_hat_k)

                self.trim_actual = rospy.get_param(self.trim_param_name)
                trim_temp = self.trim_actual + (np.mean(self.past_theta_hats)*np.asarray([0.1/0.46]))
                self.log("trim applied to system : %f" % trim_temp)
                self.i = self.i+1   
            else:
                self.log("theta not updated delta_e too big!")

        else:
            self.log("theta not updated conditions not satisfied!")

	    # (5) : Compute corrected control command
        car_cmd_corrected.v = self.ac_rif_k[0]
        car_cmd_corrected.omega = self.ac_rif_k[1] + self.theta_hat_k

        # Make sure omega is in the allowed range
        if car_cmd_corrected.omega > self.omega_max: car_cmd_corrected.omega = self.omega_max
        if car_cmd_corrected.omega < self.omega_min: car_cmd_corrected.omega = self.omega_min

        # Publish corrected command
        car_cmd_corrected.header = car_cmd.header
        self.pub_corrected_car_cmd.publish(car_cmd_corrected)
        

        # If theta hat is converging, then slowly reduce gamma
        #if self.i > 20:
        #    self.onShutdown() 

        span = np.max(self.past_theta_hats) - np.min(self.past_theta_hats) 
        self.log("span : %f" % span)
        gamma_decay_condition = span < self.thresholds[self.stage]
        #self.past_theta_hats[-1] != 0.0 and self.i > 20 those conditions are to prevent stop before all the past_theta_hats buffer is full and i >20 is to have at least 20 iterations before changing gamma
        if gamma_decay_condition and self.past_theta_hats[-1] != 0.0 and self.i > 50:
            self.stage = self.stage + 1
            self.i = 0 #set incremental variable to zero start count down again
            if self.stage == 3:
                self.onShutdown()

        # Update variables for next iteration
        self.lane_pose_k_minus = self.lane_pose_k
        self.t_lane_pose_k_minus = self.t_lane_pose_k
        self.err_k_minus = self.err_k
        self.ac_rif_k_minus_2 = self.ac_rif_k_minus
        self.ac_rif_k_minus = self.ac_rif_k

        self.t_end = rospy.Time.now().to_sec()
        latency = self.t_end - self.t_start
        self.log("latency : %f" % latency)
        
    def onShutdown(self):
        #Shutdown procedure.
        #Gives back control to joystick stopping the car

        self.joy_param('stop')

        #Save yaml file with new trim value.
        
        #INITIALIZATION FOR YAML FILE SAVE
        # Add the node parameters to the parameters dictionary and load their default values
        self.parameters = {}
    
        # Set parameters using a robot-specific yaml file if such exists
        self.parameters = self.readParamFromFile()
    
        self.SaveCalibration() #create new calibration file

        rospy.set_param("lane_controller_node/ac_on" , False)
        self.joy_param('stop')
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
    
      	new_trim = - (np.mean(self.past_theta_hats)*np.asarray(self.parameters['baseline'])*np.asarray(self.parameters['gain']))/(2.0*np.asarray(v_bar))
        # Write to a yaml file
        trim_used = rospy.get_param(self.trim_param_name)
        new_trim = round(trim_used,4) - round(new_trim.astype(float),4) 
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
        saved_details = "[gain: %s\n trim: %s\n baseline: %s\n radius: %s\n k: %s\n limit: %s\n omega_max: %s\n v_max: %s\n]" % \
        (self.parameters['gain'], new_trim, self.parameters['baseline'], self.parameters['radius'],
         self.parameters['k'], self.parameters['limit'], self.parameters['omega_max'], self.parameters['v_max'])
        self.log("Saved kinematic calibration to %s with values:\n %s" % (file_name, saved_details))
    
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

    def joy_param(self, command):
        if command == 'stop':
            axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            buttons = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]
            stop_cmd_msg = Joy(header=None, axes=axes, buttons=buttons)
            self.pub_stop_cmd.publish(stop_cmd_msg)
        if command == 'start':
            axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            buttons = [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
            start_cmd_msg = Joy(header=None, axes=axes, buttons=buttons)
            self.pub_stop_cmd.publish(start_cmd_msg)

    def display_param(self):
        self.parameters = {}
        self.parameters = self.readParamFromFile()

        data = {
            "calibration_time": time.strftime("%Y-%m-%d-%H-%M-%S"),
            "gain": self.parameters['gain'],
            "trim": self.parameters['trim'],
            "baseline": self.parameters['baseline'],
            "radius": self.parameters['radius'],
            "k": self.parameters['k'],
            "limit": self.parameters['limit'],
            "v_max": 1.0,
            "omega_max": 8.0,
        }
        current_configuration = "[gain: %s\n trim: %s\n baseline: %s\n radius: %s\n k: %s\n limit: %s\n omega_max: %s\n v_max: %s\n]" % \
        (self.parameters['gain'], self.parameters['trim'], self.parameters['baseline'], self.parameters['radius'],
         self.parameters['k'], self.parameters['limit'], self.parameters['omega_max'], self.parameters['v_max'])
        self.log("Initial kinematic parameters: \n%s" % (current_configuration))
    

if __name__ == '__main__':
    # Initialize the node
    AC_node = AdaptiveControllerNode(node_name='ac_node')
    # Keep it spinning to keep the node alive
    rospy.spin()