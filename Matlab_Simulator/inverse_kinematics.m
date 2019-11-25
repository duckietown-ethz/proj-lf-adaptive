function [u] = inverse_kinematics(car_cmd, params)
%
% Function that implements the inverse kinematics of our system, computing
% the right input voltages to the wheel motors given the desired linear and
% angular velocity.
%
% Inputs:
%   - car_cmd : car command (as computed from PI in our case); it must be a
%               structure with two fields, v and omega
%   - params : structure collecting all the kinematic parameters required
%               tocompute the inverse kinematics 
%
% Outputs:
%   - u : vector of two elements containing the two desired input voltages
%           to the motors; first element is input to right motor, second
%           element is input to left motor

v = car_cmd.v;          % constant (v_ref)
omega = car_cmd.omega;  % computed in PID

% Wheels' speed
omega_r = (v + 0.5*omega*params.baseline)/params.wheel_radius;
omega_l = (v - 0.5*omega*params.baseline)/params.wheel_radius; 

% Voltages to the wheels
u_r = omega_r/(params.k*(params.gain + params.trim));
u_l = omega_l/(params.k*(params.gain - params.trim));

% ASSUMPTION: we consider the two motor to have the same motor constant k

u_r = trim_input(u_r, params.lim);  % input to right motor
u_l = trim_input(u_l, params.lim);  % input to left motor

u = [u_r u_l];

end