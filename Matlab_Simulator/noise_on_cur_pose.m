function [cur_pose] = noise_on_current_pose(cur_pose)
% Function that adds random noise to pose

% Parameters' mean and std deviation
%for now leave x without noise
noise_on_y = [0 0.005]  %  real std when duckiebot move 0.0528 when is still 0.027
noise_on_phi = [0 0.03] %  real std when duckiebot move 0.48 when is still 0.14
cur_pose(2) = cur_pose(2) + noise_on_y(2)*randn(1);
cur_pose(3) = cur_pose(3) + noise_on_phi(2)*randn(1);

end % end function