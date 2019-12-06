function trimmed_u = trim_input(u, lim)
%
% Function that just trims the input value by comparing with its limit
% value
%
% Inputs:
%   - u : vector of two elements containing the two desired input voltages
%           to the motors; first element is input to right motor, second
%           element is input to left motor
%   - lim : maximum value that the inputs to the motors can have (absolute
%           value)
%
% Outputs:
%   - trimemd_u : corrected vector of inputs to motors

if u > lim
    trimmed_u = lim;
elseif u < -lim
    trimmed_u = -lim;
else
    trimmed_u = u;
end % end if

end