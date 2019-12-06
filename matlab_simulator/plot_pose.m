function [] = plot_pose(cur_pose,ax,running_time)
% Function to plot the pose of the Duckiebot; the trajectory is fully
% plotted and kept in time, the full pose is only plotted for the latest
% time

plot(ax,cur_pose(1), cur_pose(2), 'ob', ...
    'MarkerFaceColor',[0,0,1], 'MarkerSize', 3);

% position = [cur_pose(1:2)    0];
% orientation = rotm2quat(rotz(cur_pose(3)*180/pi));
% 
% Every x seconds plot the full pose
% recurrence_time = 1; % in [sec]
% if mod(running_time,recurrence_time)==0
%     tf = plotTransforms(position, orientation, 'frameSize', 0.1, ...
%         'MeshFilePath', 'fixedwing.stl', 'meshColor', [255 165 0]/255);
%     view(2)
% end


end % end function