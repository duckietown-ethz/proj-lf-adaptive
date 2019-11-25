function [params] = noise_on_params(params,noisy_params)
%
% Function that adds random noise to our Duckiebot's parameters every tot
% time.
% This simulate possible bumps or similar events that may cause one or more
% of the bot's parameters to change.
%
% Inputs:
%   - params : structure of kinematics parameters
%   - noisy_params : structure that specify which parameters we want to
%                       perturbe; should have the same fields as params
%
% Outputs:
%   - params : structure of perturbed parameters

% Parameters' mean and std deviation
rand_bounds =  [1       0.05*1          % gain
                27      0.05*27         % k
                0       0.05            % trim
                0.1     0.05*0.1        % baseline
                0.03    0.05*0.03];     % wheel radius

% Apply noise
fs = fields(noisy_params);
for i = 1:length(fs)
    c = fs(i);
    if noisy_params.(c{1})
        params.(c{1}) = normrnd(rand_bounds(i,1),rand_bounds(i,2));
        
        % If sampled too far from mena, just set value corresponding to one
        % standard deviation
        if params.(c{1}) > (rand_bounds(i,1)+3*rand_bounds(i,2))
            params.(c{1}) = rand_bounds(i,1)+rand_bounds(i,2);
        elseif params.(c{1}) < (rand_bounds(i,1)-3*rand_bounds(i,2))
            params.(c{1}) = rand_bounds(i,1)-rand_bounds(i,2);
        end
        
    end
end % end loop params

end % end function