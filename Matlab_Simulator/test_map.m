function [fig, centers, track_zone] = test_map(map)
%
% This function just draw an arbitrary map (trajectory), to test our
% algorithms on.
%
% For the time being three maps are avaible:
%   map==0 : is a 2m straight line
%   map==1 : is a complete circuit
%   map==2 : is a chicane

% Misure in m
l_tile = 40/100;
lane_width = 20/100 + 0.05;

% Visibility for track areas
vis = 'off';

% Create figure
fig = figure('Name','Map','NumberTitle','off','Position',[680 200 700 700]);

%% straight line
if map == 0 % Straight line
    % Define map through series of points
    A = [0 0];         %hor    
    B = [5*l_tile 0];
    points = [A; B];
    curve_points = [A; B];
        
    % Start by plotting straight segments
    plot(   [A(1) B(1)], [A(2) B(2)], 'sr-', ...
            'MarkerFaceColor',[1,0,0], 'MarkerSize', 3, ...
            'LineWidth', 0.1);
    axis equal
    hold on
    axis off
    xlim([min(points(:,1))-0.4 max(points(:,1))+0.4])
    ylim([min(points(:,2))-0.4 max(points(:,2))+0.4])
    
    % The centers of the curves are useful for some computations
    centers = [0];
    
    %plot(centers(:,1), centers(:,2), 'ob')     % debug only
    
    % In-line test
    lane_area = [];
    type = [];  rif = [];
    for i = 1:0.5*size(points,1)
        idx = 1+(i-1)*2;
        
        %% Straight line zones
        
        x = [points(idx,1) points(idx+1,1)];
        y = [points(idx,2) points(idx+1,2)];
        
        % Define section type
        type = [type, 1*(y(1)==y(2))+2*(y(1)~=y(2))];  % 1 for horizontal, 2 for vertical
        % If we are on a straight segment we want our bot to converge to
        % the same y coordinate, to the x coordinate for vertical segments.
        if type(end) == 1
            rif = [rif; y(1) 0];
        elseif type(end) == 2
            rif = [rif; x(1) 0];
        end
        x = [   x(1) - 0.5*lane_width*(y(1)~=y(2))      x(1) + 0.5*lane_width*(y(1)~=y(2)) ...
                x(2) + 0.5*lane_width*(y(1)~=y(2))      x(2) - 0.5*lane_width*(y(1)~=y(2))];
        y = [   y(1) - 0.5*lane_width*(y(1)==y(2))      y(1) + 0.5*lane_width*(y(1)==y(2)) ...
                y(2) + 0.5*lane_width*(y(1)==y(2))      y(2) - 0.5*lane_width*(y(1)==y(2))];
        p = polyshape(x,y);
        lane_area = [lane_area p];
        %plot(p);   % debug
        
         % the reference for curves needs to be updated on the computation error function
        
    end

    track_zone.poly = lane_area;
    track_zone.type = type;
    track_zone.rif = rif;
    track_zone.plot = plot(track_zone.poly, 'FaceColor','yellow','FaceAlpha',0.1, 'Visible', vis);

%% circuit
elseif map == 1 % Custom map 1
    
    % Define map through series of points
    A = [l_tile 0];             B = [3*l_tile 0];           C = [4*l_tile l_tile];
    D = [4*l_tile 2*l_tile];    E = [5*l_tile 3*l_tile];    F = [7*l_tile 3*l_tile];
    G = [8*l_tile 4*l_tile];    H = [8*l_tile 5*l_tile];    I = [7*l_tile 6*l_tile];
    L = [l_tile 6*l_tile];      M = [0 5*l_tile];           N = [0 l_tile];
    points = [A; B; C; D; E; F; G; H; I; L; M; N];
    
    %curve_points = [points(end,:); points(1:end-1,:)];
    curve_points = [points(2:end,:); points(1,:)];
    
    % Start by plotting straight segments
    plot(   [A(1) B(1)], [A(2) B(2)], 'sr-', ...
            [C(1) D(1)], [C(2) D(2)], 'sr-', ...
            [E(1) F(1)], [E(2) F(2)], 'sr-', ...
            [G(1) H(1)], [G(2) H(2)], 'sr-', ...
            [I(1) L(1)], [I(2) L(2)], 'sr-',...
            [M(1) N(1)], [M(2) N(2)], 'sr-', ...
            'MarkerFaceColor',[1,0,0], 'MarkerSize', 3, ...
            'LineWidth', 0.1);
    axis equal
    hold on
    axis off
    xlim([min(points(:,1))-0.4 max(points(:,1))+0.4])
    ylim([min(points(:,2))-0.4 max(points(:,2))+0.4])
    
    % Plot curves
    curves =@(P1,P2) draw_curves(P1,P2,l_tile);
    c6 = curves(N, A);    c1 = curves(C, B);    c2 = curves(D, E);
    c3 = curves(G, F);    c4 = curves(H, I);    c5 = curves(M, L);
    
    % The centers of the curves are useful for some computations
    centers = [c1; c2; c3; c4; c5; c6];
    
    %plot(centers(:,1), centers(:,2), 'ob')     % debug only
    
    % In-line test
    lane_area = [];
    type = [];  rif = [];
    for i = 1:0.5*size(points,1)
        idx = 1+(i-1)*2;
        
        %% Straight line zones
        
        x = [points(idx,1) points(idx+1,1)];
        y = [points(idx,2) points(idx+1,2)];
        
        % Define section type
        type = [type, 1*(y(1)==y(2))+2*(y(1)~=y(2))];  % 1 for horizontal, 2 for vertical
        % If we are on a straight segment we want our bot to converge to
        % the same y coordinate, to the x coordinate for vertical segments.
        if type(end) == 1
            rif = [rif; y(1) 0];
        elseif type(end) == 2
            rif = [rif; x(1) 0];
        end
        x = [   x(1) - 0.5*lane_width*(y(1)~=y(2))      x(1) + 0.5*lane_width*(y(1)~=y(2)) ...
                x(2) + 0.5*lane_width*(y(1)~=y(2))      x(2) - 0.5*lane_width*(y(1)~=y(2))];
        y = [   y(1) - 0.5*lane_width*(y(1)==y(2))      y(1) + 0.5*lane_width*(y(1)==y(2)) ...
                y(2) + 0.5*lane_width*(y(1)==y(2))      y(2) - 0.5*lane_width*(y(1)==y(2))];
        p = polyshape(x,y);
        lane_area = [lane_area p];
        %plot(p);   % debug
        
        %% Curve zones
%         delta_p = curve_points(idx,:)+curve_points(idx+1,:);
%         delta_m = curve_points(idx,:)-curve_points(idx+1,:);
%         mid_point = (delta_p)*0.5 + sign(delta_m(1)*delta_m(2))*[-0.5 +0.5].*(delta_m);
%         x = [mid_point(1)+l_tile, mid_point(1)-l_tile, mid_point(1)-l_tile, mid_point(1)+l_tile];
%         y = [mid_point(2)+l_tile, mid_point(2)+l_tile, mid_point(2)-l_tile, mid_point(2)-l_tile];
%         p = polyshape(x,y);
%         lane_area = [lane_area p];
%         type = [type, 0];   % type = 0 for curves
%         rif = [rif; 0 0];   % the reference for curves needs to be updated on the computation error function
        
    end
    cont = 0
    %% curva 1
    x = [c1(1)          , c1(1) + 0*l_tile  , c1(1) + 2*l_tile, c1(1) + 2*l_tile];
    y = [c1(2)          , c1(2) - 2*l_tile  , c1(2) - 2*l_tile, c1(2) + 0*l_tile];
    p = polyshape(x,y);
    lane_area = [lane_area(1:1+cont*2) p lane_area((1+cont)*2:end) ];
    cont = cont +1;
    %% curva 2
    x = [c2(1)          , c2(1) + 0*l_tile  , c2(1) - 2*l_tile, c2(1) - 2*l_tile];
    y = [c2(2)          , c2(2) + 2*l_tile  , c2(2) + 2*l_tile, c2(2) + 0*l_tile];
    p = polyshape(x,y);
    lane_area = [lane_area(1:1+cont*2) p lane_area((1+cont)*2:end) ];
    cont = cont +1;
    %% curva 3
    x = [c3(1)          , c3(1) + 0*l_tile  , c3(1) + 2*l_tile, c3(1) + 2*l_tile];
    y = [c3(2)          , c3(2) - 2*l_tile  , c3(2) - 2*l_tile, c3(2) + 0*l_tile];
    p = polyshape(x,y);
    lane_area = [lane_area(1:1+cont*2) p lane_area((1+cont)*2:end) ];
    cont = cont +1;
    %% curva 4
    x = [c4(1)          , c4(1) + 2*l_tile  , c4(1) + 2*l_tile, c4(1) + 0*l_tile];
    y = [c4(2)          , c4(2) + 0*l_tile  , c4(2) + 2*l_tile, c4(2) + 2*l_tile];
    p = polyshape(x,y);
    lane_area = [lane_area(1:1+cont*2) p lane_area((1+cont)*2:end) ];
    cont = cont +1;
    %% curva 5
    x = [c5(1)          , c5(1) - 0*l_tile  , c5(1) - 2*l_tile, c5(1) - 2*l_tile];
    y = [c5(2)          , c5(2) + 2*l_tile  , c5(2) + 2*l_tile, c5(2) + 0*l_tile];
    p = polyshape(x,y);
    lane_area = [lane_area(1:1+cont*2) p lane_area((1+cont)*2:end) ];
    cont = cont +1;
    %% curva 6
    x = [c6(1)          , c6(1) - 2*l_tile  , c6(1) - 2*l_tile, c6(1) + 0*l_tile];
    y = [c6(2)          , c6(2) + 0*l_tile  , c6(2) - 2*l_tile, c6(2) - 2*l_tile];
    p = polyshape(x,y);
    lane_area = [lane_area(1:1+cont*2) p lane_area((1+cont)*2:end) ];
    cont = cont +1;
    %%
    
    for i = 0:5
        type = [type(1:1+i*2) 0 type((i+1)*2:end)]; % type = 0 for curves
        rif = [rif(1:1+i*2,:); 0 0;rif((i+1)*2:end,:)];
    end
    
    track_zone.poly = lane_area;
    track_zone.type = type;
    track_zone.rif = rif;
    track_zone.plot = plot(track_zone.poly, 'FaceColor','yellow','FaceAlpha',0.1, 'Visible', vis);
    
    

%% chicane    
elseif map == 2
    % Define map through series of points
    A = [0 3*l_tile];         %hor    
    B = [3*l_tile 3*l_tile];  %curve         
    C = [4*l_tile 2*l_tile];  %ver
    D = [4*l_tile l_tile];    %curve
    E = [5*l_tile 0];         %hor
    F = [8*l_tile 0];         %
    points = [A; B; C; D; E; F];
    curve_points = [A; B; C; D; E; F];
        
    % Start by plotting straight segments
    plot(   [A(1) B(1)], [A(2) B(2)], 'sr-', ...
            [C(1) D(1)], [C(2) D(2)], 'sr-', ...
            [E(1) F(1)], [E(2) F(2)], 'sr-', ...
            'MarkerFaceColor',[1,0,0], 'MarkerSize', 3, ...
            'LineWidth', 0.1);
    axis equal
    hold on
    axis off
    xlim([min(points(:,1))-0.4 max(points(:,1))+0.4])
    ylim([min(points(:,2))-0.4 max(points(:,2))+0.4])
    
    % Plot curves
    curves =@(P1,P2) draw_curves(P1,P2,l_tile);
    c1 = curves(C, B);    c2 = curves(D, E);
    
    % The centers of the curves are useful for some computations
    centers = [c1; c2];
    
    %plot(centers(:,1), centers(:,2), 'ob')     % debug only
    
    % In-line test
    lane_area = [];
    type = [];  rif = [];
    for i = 1:0.5*size(points,1)
        idx = 1+(i-1)*2;
        
        %% Straight line zones
        
        x = [points(idx,1) points(idx+1,1)];
        y = [points(idx,2) points(idx+1,2)];
        
        % Define section type
        type = [type, 1*(y(1)==y(2))+2*(y(1)~=y(2))];  % 1 for horizontal, 2 for vertical
        % If we are on a straight segment we want our bot to converge to
        % the same y coordinate, to the x coordinate for vertical segments.
        if type(end) == 1
            rif = [rif; y(1) 0];
        elseif type(end) == 2
            rif = [rif; x(1) 0];
        end
        x = [   x(1) - 0.5*lane_width*(y(1)~=y(2))      x(1) + 0.5*lane_width*(y(1)~=y(2)) ...
                x(2) + 0.5*lane_width*(y(1)~=y(2))      x(2) - 0.5*lane_width*(y(1)~=y(2))];
        y = [   y(1) - 0.5*lane_width*(y(1)==y(2))      y(1) + 0.5*lane_width*(y(1)==y(2)) ...
                y(2) + 0.5*lane_width*(y(1)==y(2))      y(2) - 0.5*lane_width*(y(1)==y(2))];
        p = polyshape(x,y);
        lane_area = [lane_area p];
        %plot(p);   % debug
        
         % the reference for curves needs to be updated on the computation error function
        
    end
    %% Curve zones
    x = [c1(1) + 2*l_tile, c1(1) + 2*l_tile, c1(1),            c1(1)];
    y = [c1(2)           , c1(2) + 2*l_tile, c1(2) + 2*l_tile, c1(2)];
    p = polyshape(x,y);
    lane_area = [lane_area(1) p lane_area(2:end) ];
    type = [type(1), 0,type(2:end)];   % type = 0 for curves
    rif = [rif(1,:); 0 0;rif(2:end,:)]; 
    x = [c2(1) - 2*l_tile, c2(1) - 2*l_tile, c2(1),            c2(1)];
    y = [c2(2)           , c2(2) - 2*l_tile, c2(2) - 2*l_tile, c2(2)];
    p = polyshape(x,y);
    lane_area = [lane_area(1:3) p lane_area(4:end) ];
    type = [type(1:3), 0,type(4:end)];   % type = 0 for curves
    rif = [rif(1:3,:); 0 0;rif(4:end,:)]; 
    %%
    track_zone.poly = lane_area;
    track_zone.type = type;
    track_zone.rif = rif;
    track_zone.plot = plot(track_zone.poly, 'FaceColor','yellow','FaceAlpha',0.1, 'Visible', vis);
    
end


end

function [center] = draw_curves(A,B,r)
% A function that just takes care of drawing the curves.
% The curves are drawn in anticlockwise direction from A

teta = [0:5*pi/180:pi/2];
teta2 = [pi*0.5:-5*pi/180:0];
vec_x = A(1)-sign(A(1)-B(1))*r*(1-cos(teta));
vec_y = A(2)-sign(A(2)-B(2))*r*sin(teta);
% vec_x = [A(1):-sign(A(1)-B(1))*r*cos(85*pi/180):B(1)];
% vec_y = [A(2):-sign(A(2)-B(2))*r*sin(5*pi/180):B(2)];
plot(vec_x,vec_y,'r-', 'LineWidth', 0.1);

center = [B(1) A(2)];

end
