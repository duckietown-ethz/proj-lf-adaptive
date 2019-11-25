function [car_cmd] = pid_controller(v_ref,y,t,first,Ts)
%y is a two component vector, time is a float value
% if first == True initialize everything

persistent time_past; 
persistent y_prev;
persistent err_integral;

k_d = - 2;
k_phi = -2;
k_dI= -2.5;
k_phiI = -1.25;

molt_coeff = 4;
c = 0.22/0.25;

if first == true
    
    time_past = t;
    y_prev = y;
    
    k_d = -2;
    k_phi = -2;
    k_dI= -2.5;
    k_phiI = -1.25;
    
    err_integral = [0; 0]; 
end

dt = t - time_past; 
dt = Ts; % <---------

err = y;
err_integral(1) = err_integral(1) + dt * (y(1) + y_prev(1)) / 2;
err_integral(2) = err_integral(2) + dt * (y(2) + y_prev(2)) / 2;

omega = k_d * c * err(1) + k_phi * c * err(2) ...
        + k_dI * c * err_integral(1) + k_phiI * c * err_integral(2);

car_cmd.v = v_ref;  car_cmd.omega = omega*molt_coeff;

time_past = t;
y_prev = y;
end