function [] = plot_orientation(cur_pose,button,pose_error)
%
% Function that create a figure focusing on the orientation of the
% duckiebot.
%
% Inputs:
%   - cur_pose : current pose of the bot
%   - button : handle to the button that switch the visibility of the
%               figure
%   - pose_error : error between current pose and reference  (mid lane with
%                   zero angle
%
% Outputs:
%   - none : no output, it just plot the orientation

persistent ax or_plot ann_ori

vis = 'on';

pos = [0    0    0];
rot = rotm2quat(rotz(cur_pose(3)*180/pi));

lim = 1.5;

str_ori = 'Orientation error [deg]: ';

% Look if the figure already exists
if isempty(findobj('type','figure','name','Orientation'))
    % If figure is not initializated yet, do it
    or_plot = figure(  'Name','Orientation','NumberTitle','off','Position',[1400 500 400 400],...
                        'Visible','off');
                    
    ax = axes('Visible',vis,'XLim',[-lim lim],'YLim',[-lim lim]);
    hold(ax,'on')
    axis off
    axis manual
    title('Orientation')
    
    ann_ori = annotation('textbox',[.15 .59 .3 .3],'String',str_ori,...
        'FitBoxToText','on','LineStyle','none');
    
end

cla(ax) % clear axes

% Plot reference
ang_rif = cur_pose(3)-pose_error(2);
plot(ax,2*[-lim lim]*cos(ang_rif),2*[-lim lim]*sin(ang_rif),'c-','LineWidth',2);

% Plot pose
tf = plotTransforms(pos, rot, 'Parent', ax, 'frameSize', 1.7,...
    'MeshFilePath', 'fixedwing.stl', 'meshColor', [255 165 0]/255);
view(ax,2)

ann_ori.String =[str_ori, num2str(pose_error(2)*180/pi,4)]; % update annotation

% Button functionality
if button.Value == 1
    or_plot.Visible = 'on';
else
    or_plot.Visible = 'off';
end


end % end function