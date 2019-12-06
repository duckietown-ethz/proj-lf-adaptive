function [] = plot_err(pose_error, name, i, Ts, button_error)
%
% Function that plot the running value of the pose error.
%
% Inputs:
%   - pose_error : error between current pose and reference  (mid lane with
%                   zero angle
%   - name : name of the figure
%   - i : iteration counter
%   - Ts : sampling time
%   - button_error : error between current pose and reference  (mid lane with
%                   zero angle
%
% Outputs:
%   - none : no output, it just plot the error on desired pose

% Persistent variables
persistent ax1 ax2 lim_d_up lim_d_down zero_line lim_a err_plot

vis = 'on'; % default visibility for the plots

% Look if the figure already exists
if isempty(findobj('type','figure','name',name))
    % If figure is not initializated yet, do it
    err_plot = figure(  'Name',name,'NumberTitle','off','Position',[50 100 600 400],...
                        'Visible','off');
                    
    ax1 = subplot(2,1,1,'Visible',vis,'YLim',[-23 23]);
    hold on
    grid on
    title('Distance from midline')
    xlabel('Time [s]')
    ylabel('d [cm]')
    lim_d_up = plot(ax1,[0 0],[20 20],'--r', 'Visible', vis);
    lim_d_down = plot(ax1,[0 0],[-20 -20],'--r', 'Visible', vis);
    zero_line = plot(ax1,[0 0],[0 0],'--r', 'Visible', vis);
    
    ax2 = subplot(2,1,2,'Visible',vis,'YLim',[-80 80]);
    hold on
    grid on
    title('Orientation error')
    xlabel('Time [s]')
    ylabel('phi [deg]')
    lim_a = plot(ax2,[0 0],[0 0],'--r', 'Visible', vis);
    
end

% Button functionality
if button_error.Value == 1
    err_plot.Visible = 'on';
else
    err_plot.Visible = 'off';
end


ax1.XLim = [0 (i+1)*Ts]; 
plot(ax1,i*Ts,pose_error(1)*100, '-bo', 'Visible', vis, ...
        'MarkerFaceColor',[0,0,1], 'MarkerSize', 2);
lim_d_up.XData = [0 i*Ts];
lim_d_down.XData = [0 i*Ts];
zero_line.XData = [0 i*Ts];

ax2.XLim = [0 (i+1)*Ts];
lim_a.XData = [0 i*Ts];
plot(ax2,i*Ts,pose_error(2)*180/pi, '-bo', 'Visible', vis, ...
        'MarkerFaceColor',[0,0,1], 'MarkerSize', 2);

end % end function