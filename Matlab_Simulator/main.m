%% Main
% Call all desired functions from here

%% Tabula rasa
clear all;  close all;

%% Plant params
% Two structures: real (or "correct") parameters and estimated parameters.
% Ideally we would like the estimated params to converge to the real ones.

real_params = struct(   'gain',         1, ...
                        'k',            27, ...
                        'trim',         -0.1, ...
                        'baseline',     0.1, ...
                        'wheel_radius', 0.03, ...
                        'lim',          1);
                    
estimated_params = struct(  'gain',         1, ...
                            'k',            27, ...
                            'trim',         0, ...%-0.0991
                            'baseline',     0.1,... 
                            'wheel_radius', 0.03, ...
                            'lim',          1);

%% Initialization

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INITIALIZATION VALUES:

cur_pose    = [1.4    2.45     0];          % intial pose
%cur_pose    = [0.41   2.4     0];          % intial pose
%cur_pose    = [0   0    0];  
%cur_pose = [0 1.2 0];
Ts          = 0.15;                          % sampling time
map_type    = 1;                            % zero for straight line
adaptive_on = true;                         % switch to use adaptive controller
%gamma       = 1000; %if use error on d
gamma       = 10;   %if use error on phi    % proportional constant for adaptation law
v_ref = 0.25; %linear velocity


% Set to true for parameters you want to perturb 
noisy_params = struct(      'gain',         false, ...
                            'k',            false, ...
                            'trim',         true, ...
                            'baseline',     false, ...
                            'wheel_radius', false, ...
                            'lim',          false);

manual = false;                 % switch for manual mode
pause_time = 0.05;              % pause time for auto mode               
noise_on_pose_error = false;     % switch for noise on pose (simulate uncertain pose estimate)
noise_on_parameters = false;    % switch for noise on parameters
noise_period = 10;              % perturb parameters every noise_period sec
conversion_countdown = 50;      % countdown for automatic stop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%cur_pose = [0.01 1.2 0];
%cur_pose = [3.2    2.01   (3/2)*pi]
cur_pose(3) = mod(cur_pose(3), 2*pi);

t = 1;  % for now it is initialized to a random value and not used

% Initialize figure
[fig, centers, track_zone] = test_map(map_type);
ax = fig.Children;
hold(ax,'on');  axis(ax,'equal');
title('Test track')

% Fucntion to choose manually where to start
% [x_0,y_0] = ginput(1);

% Setup buttons
button_err = uicontrol(fig,'Style','radiobutton','String','Error plot','Position',[40 80 100 20]);
stop_button = uicontrol(fig,'Style','togglebutton','String','STOP SIMULATION','Position',[500 60 100 20]);
button_section = uicontrol(fig,'Style','radiobutton','String','Track sections','Position',[150 55 100 20]);
button_orientation = uicontrol(fig,'Style','radiobutton','String','Orientation plot','Position',[150 80 100 20]);

% Plot starting point
%plot(cur_pose(1), cur_pose(2), 'xb')

% Initialize annotation on track plot
str1 = 'Current section : ';
ann1 = annotation('textbox',[.15 .59 .3 .3],'String',str1,'FitBoxToText','on','LineStyle','none');
str2 = 'Current pose : ';
ann2 = annotation('textbox',[.15 .56 .3 .3],'String',str2,'FitBoxToText','on','LineStyle','none');
str3 = 'Running time : ';
ann3 = annotation('textbox',[.5 .59 .3 .3],'String',str3,'FitBoxToText','on','LineStyle','none');

% Initialize adaptation plot if adaptation is on
if adaptive_on  
    fig_theta = figure(  'Name','Theta','NumberTitle','off','Position',[50 650 600 300],...
                    'Visible','off');     
    ax_theta = axes(fig_theta);
    hold(ax_theta,'on');    grid(ax_theta,'on');
    title('Adaptation law')
    xlabel('Time [s]')
    ylabel('\theta hat')
    
    str_theta = '\theta hat dot : ';
    ann_theta = annotation('textbox',[.15 .56 .3 .3],'String',str_theta,...
        'FitBoxToText','on','LineStyle','none');
    
    str_gamma = '\gamma : ';
    ann_gamma = annotation('textbox',[.15 .50 .3 .3],'String',str_gamma,...
        'FitBoxToText','on','LineStyle','none');
    
    button_adapt = uicontrol(fig,'Style','radiobutton','String','Adaptation plot','Position',[40 55 100 20]);
    
    start_countdown_time = 0;   start_countdown_theta = 0;
    countdown_marker = plot(ax_theta,start_countdown_time,start_countdown_theta,...
            'gs','MarkerFaceColor',[0 1 0]);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Make movie 
movie_on = false;
ax_movie = ax;
% movie_name = 'prova.avi';
nFrames = 300;
% if movie_on    
%     mov(1:nFrames) = struct('cdata',[], 'colormap',[]);
% end
% !! WORKS SUPER SLOWLY
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Control loop
% After computing the error wrt the desired pose, call the PID controller
% to compute the right omega to keep the bot on track.

mean_theta = 0;
flag_gamma = 1;

%pause(2);   % pause to take frame

i = 0; % iteration number
while(i < 100 || conversion_countdown ~= 0)
    
    clc

    ann3.String = [str3, num2str(i*Ts)]; % update time annotation
    
    if button_section.Value == 1    % switch visibility of track's sections
        for j = 1:length(track_zone.poly)
            ax.Children(end-(j+length(track_zone.poly)-1)).Visible = 'on';
        end
    else
        for j = 1:length(track_zone.poly)
            ax.Children(end-(j+length(track_zone.poly)-1)).Visible = 'off';
        end
    end
    
    cur_section = current_section(cur_pose, track_zone);    % compute current section
    ann1.String = [str1, int2str(cur_section.in_section)];  % update section annotation
    
    % Apply error on pose
    if noise_on_pose_error
       cur_pose = noise_on_cur_pose(cur_pose);
    end
    
    %pose_error = cur_pose(2:3) - des_pose;
    [pose_error, delta_ang_rif] = compute_pose_error(map_type,cur_pose, cur_section, centers);
    % NOTE: we still need the pose as three elements vector to plot purposes,
    % but for the PID we are interested only in the errors on d (y) and theta.
      
    % Plot pose error
    plot_err(pose_error, 'Pose Error', i, Ts, button_err);
    
    % Plot current orientation
    plot_orientation(cur_pose,button_orientation,pose_error);
    
    % Differentiate initialization from sequent times
    first = false;
    if i == 0
        first = true;
    end 
    
    % PID
    car_cmd = pid_controller(v_ref,pose_error,t,first,Ts);
    
    if map_type == 0
        car_cmd.omega = 0;
    end
    
    % Adaptive controller
    if adaptive_on
        time_current = i*Ts;
        [car_cmd, theta_hats_vector, theta_hat_d] = adaptive_controller(pose_error,car_cmd,...
            time_current,first,Ts,gamma,delta_ang_rif);
        if button_adapt.Value == 1  % switch for theta plot visibility
            vis_adapt = 'on';
        else
            vis_adapt = 'off';
        end
        fig_theta.Visible = vis_adapt;
        plot(ax_theta,i*Ts,theta_hats_vector(end),'om','MarkerFaceColor',[1 0 1], 'MarkerSize', 3);
        ax_theta.XLim = [0 (i+1)*Ts];
        ann_theta.String = [str_theta, num2str(theta_hat_d,4)]; % update theta annotation
        ann_gamma.String = [str_gamma, num2str(gamma,1)]; % update gamma annotation
        
        countdown_marker.Visible = 'off';
        countdown_marker = plot(ax_theta,start_countdown_time,start_countdown_theta,...
            'gs','MarkerFaceColor',[0 1 0]);
    end
        
    % Noise on parameters
    if noise_on_parameters && (mod(i*Ts,noise_period)==0)
        real_params = noise_on_params(real_params,noisy_params);
    end
        
    % Inverse kinematics
    % Once obtained omega from the PID, compute the corresponding input
    % voltages to give to the motor to obtained the desired motion. 
    % This is done through our ESTIMATE of the parameters (what is known).
    u = inverse_kinematics(car_cmd, estimated_params);
    
    % Odometry
    % Having the inputs to the motor, we can predict where the robot will be
    % in the next timestamp using the forward kinematics (odometry).
    % We can do this for both the real and the estimated system, so that we
    % can then compare the output, such to have an estimate of how wrong are
    % the parameters.
    cur_pose = forward_kinematics(u, real_params, cur_pose, Ts);
    cur_pose(3) = mod(cur_pose(3), 2*pi);
    
    ann2.String = [str2, mat2str(cur_pose,2)]; % update pose annotation
    
    %figure(findobj('type','figure','name','Map'));   % Plot on the right figure
    plot_pose(cur_pose,ax,i*Ts)
    
    if ~adaptive_on
        expected_pose = forward_kinematics(u, estimated_params, cur_pose, Ts);
        plot(ax,expected_pose(1), expected_pose(2), 'xg', 'Visible','off');
        e = cur_pose - expected_pose; % this error is somehow function of the uncertainty in our params
        % NOTE: based on this e we could simply do gradient descent in the simpler
        % case in which we use only trim or trim and gain.
    end
    
    % Stop button
    if stop_button.Value == 1
        break
    end
    
    real_trim = real_params.trim;
    
    drawnow()   % create aniamtion
    
    % Select animation mode
    if manual   
        pause               % manual mode
    else
        pause(pause_time)   % automatic mode
    end
    
    % Gamma variabile
    if adaptive_on
        
        if ((max(theta_hats_vector(90:end))-min(theta_hats_vector(90:end))) < 0.07) && i>10 && flag_gamma == 1
            gamma = 4;
            flag_gamma = flag_gamma+1;
        elseif ((max(theta_hats_vector(75:end))-min(theta_hats_vector(80:end))) < 0.055) && i>10 && flag_gamma == 2
            gamma = 2;
            flag_gamma = flag_gamma+1;
        elseif ((max(theta_hats_vector(75:end))-min(theta_hats_vector(70:end))) < 0.04) && i>10 && flag_gamma == 3
            gamma = 1;
            flag_gamma = flag_gamma+1;
        elseif ((max(theta_hats_vector(60:end))-min(theta_hats_vector(60:end))) < 0.035) && i>10 && flag_gamma == 4
            gamma = 0.5;
            flag_gamma = flag_gamma+1;
        end
        
        % Automatic stop when adaptive law converges
        if ((max(theta_hats_vector)-min(theta_hats_vector)) < 0.035) && adaptive_on 
            conversion_countdown = conversion_countdown -1;
        elseif adaptive_on 
            start_countdown_time = i*Ts;
            start_countdown_theta = theta_hats_vector(end);
            conversion_countdown = 50;
        end
    end
    
    % Update movie
    while (i < nFrames) && movie_on
        %mov(i+1) = getframe(ax_movie);
        
        f = getframe(ax_movie);
    end
    
    
    
    
    i = i+1;    % iteration

end % end control loop

if adaptive_on
    mean_theta = mean(theta_hats_vector);
    trim = -(mean_theta*estimated_params.baseline*estimated_params.gain)/(2*v_ref)
end

if movie_on
    movie2avi(mov, movie_name, 'compression','None', 'fps',10);
end

