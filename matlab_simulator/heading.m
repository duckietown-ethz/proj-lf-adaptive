function [ang_rif, direction] = heading(cur_orientation, cur_section)
% Function that, given the section of the track the bot currently is and its
% orientation, determines what direction the Duckiebot is going (in
% particular it computes the right angle to use as reference to go
% straight).

% NOTE: this function is not needed when in a curve, so we are not
% interested in the case type == 0.

%delta = 0+(cur_section.type==2)*0.5*pi;
delta = 0+(cur_section.type==1)*0.5*pi;
%bounds = [0.5*pi   1.5*pi] + delta;
bounds = pi;
cur_orientation = mod(cur_orientation+delta,2*pi);
%if (cur_orientation>bounds(1)) && (cur_orientation<bounds(2))
if (cur_orientation)<=bounds
    %ang_rif = 0+delta;
    ang_rif = 0.5*pi-delta;
    
    % DIRECTION:
    % 1  => the bot is moving left to right or upwards
    % -1 => the bot is moving right to left or downwards
    direction = 1;
else
    %ang_rif = pi+delta;
    ang_rif = 1.5*pi-delta;
    direction = -1;
end

end