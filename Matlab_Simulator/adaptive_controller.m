function [car_cmd_corrected, theta_hats_vector_output, theta_hat_d] = adaptive_controller(yp_current,car_cmd,time_current,first,Ts, gamma,delta_ang_rif)
%
% Function that implements the adaptive controller.
% This aims to correct the velocity command computed through the PID to
% guarantee better performance during lane following, and most importantly
% track of the trend of its internal state theta_hat, which allows us to
% have an estimate on the correct parameters of the duckiebot.
%
% Inputs:
%   - yp_current : current pose of the bot
%   - car_cmd : speed command as it is computer in the PI; structure with
%               fields v and omega
%   - time_current : current time; not used; to use in case it is
%               introduced variable timestamp
%   - first : flag that identify the first call of the function
%   - Ts : sampling time 
%   - gamma : proportional constant for adaptation law
%   - delta_ang_rif : delta in angualr position to correct angle reference
%                       in curves 
%
% Outputs:
%   - car_cmd_corrected : corrected speed command, expressed as a structure
%               with fields v and omega
%   - theta_hats_vector_output : vector of the last 100 values of theta_hat
%   - theta_hat_d : derivative of theta_hat

% Persistent variables
persistent time_k_minus
persistent ref_k_minus
persistent ym_k_minus
persistent theta_hat_k_minus
persistent theta_hats_vector
persistent prova_scalino

% Proportional constant for the adaptation law
%gamma = [250 0.5];

% Initialization 
if first == true  
    time_k_minus = time_current;
    ym_k_minus = yp_current;
    %theta_hat_k_minus = 1; %if use theta as moltiplicative factor
    theta_hat_k_minus = 0;
    ref_k_minus = car_cmd;
    %theta_hat_k = 1; %if use theta as moltiplicative factor
    theta_hat_k = 0;
    car_cmd_corrected = car_cmd;
    theta_hat_d = 0;
    theta_hats_vector = zeros(1,100);
    prova_scalino = false;
else

    % Timestamp
    dt = time_current - time_k_minus; 
    dt = Ts; % <---------
    
    ym_k_minus(2)= ym_k_minus(2) - delta_ang_rif;   % correction in reference angle in curves
    
    ym_k_predicted(1) = ym_k_minus(1) + ref_k_minus.v * dt * sin(ym_k_minus(2) + ref_k_minus.omega * dt * 0.5);
    ym_k_predicted(2) = ym_k_minus(2) + ref_k_minus.omega * dt;

    err = yp_current - ym_k_predicted
    
    if (abs(err(2)) > pi)
        err(2) = err(2) - sign(err(2)) * 2 * pi;
    end
    
    %usually estimate underestimate so try to push it a little bit 1
    %percent  up
    if ((gamma == 1) & ( ~prova_scalino))
        prova_scalino = true;
        theta_hat_k_minus = theta_hat_k_minus * 1.02;
    end

    % ADAPTATION LAW
    if length(gamma) == 1
        theta_hat_d = - gamma * err(2);
    elseif length(gamma) == 2
        theta_hat_d = - gamma * err';
    else
        error('Non valid gamma provided');
    end
    theta_hat_k = theta_hat_k_minus + dt * theta_hat_d
    
    theta_hats_vector = [theta_hats_vector(2:end) theta_hat_k];
        
    % CONTROL LAW
    car_cmd_corrected.v = car_cmd.v;
    %car_cmd_corrected.omega = car_cmd.omega*theta_hat_k;
    car_cmd_corrected.omega = car_cmd.omega + theta_hat_k;

    % Variables for next iteration
    time_k_minus = time_current;
    ym_k_minus = yp_current;
    theta_hat_k_minus = theta_hat_k;
    ref_k_minus = car_cmd;  
    
end
theta_hats_vector_output = theta_hats_vector; %persistent variables cannot be used as output, dummy variable 


end  % end function