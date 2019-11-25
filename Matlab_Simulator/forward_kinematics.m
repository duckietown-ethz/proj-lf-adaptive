function [pose] = forward_kinematics(u, params, cur_pose, Ts)
%
% Function that implements the forward kinematics of our system, computing
% the corresponding linear velocity and yaw rate given the inputs to the
% motors.
% The function returs directly the pose at the next timestamp.
%
% Inputs:
%   - u : vector of two elements containing the two desired input voltages
%           to the motors; first element is input to right motor, second
%           element is input to left motor
%   - cur_pose : bot's current pose
%   - Ts : sampling time
%
% Outputs:
%   - pose : pose at the next timestamp ((i+1)*Ts)

% Change of names for easiness of brevity
R = params.wheel_radius;
L = params.baseline;
k = params.k;
gain = params.gain;
t = params.trim;

% Compute velocities
mat = [ k*(gain+t)*R*0.5        k*(gain-t)*R*0.5
        k*(gain+t)*R/(2*0.5*L)  -k*(gain-t)*R/(2*0.5*L) ];
vel = mat*u';

% Second order integration using Runge-Kutta
pose(3) = cur_pose(3) + vel(2)*Ts;
pose(1) = cur_pose(1) + vel(1)*Ts*cos(cur_pose(3) + Ts*vel(2)*0.5);
pose(2) = cur_pose(2) + vel(1)*Ts*sin(cur_pose(3) + Ts*vel(2)*0.5);

end % end function