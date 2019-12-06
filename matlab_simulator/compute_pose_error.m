function [pose_err, delta_ang] = compute_pose_error(map_type,cur_pose, cur_section, centers)
% Function that compute pose error depending what section of the track the
% bot currently is.
%
% 0 -> curve
% 1 -> horizontal
% 2 -> vertical

% Need persistent variable to calculate delta in angle reference to compute
% error in curve in adaptive controller.
persistent ang_rif_past 

[ang_rif, direction] = heading(cur_pose(3), cur_section);
% NOTE: for how it is defined in the function heading, ang_rif should
% always be positive and less than 2*pi

if cur_section.type == 0
    %% Curve
    vec = [cur_pose(1:2)-centers(cur_section.in_section*0.5,:)  0];
    dist = norm(vec);
    tangent = cross(vec,[0 0 1]);
    if cur_section.in_section == 4
        tangent = cross(vec,[0 0 -1]);
    end
    ang_rif = mod(atan2(tangent(2),tangent(1)), 2*pi) % always positive
    pose_err = [(-1*(cur_section.in_section == 4)+1*((cur_section.in_section ~= 4)))*...
        (dist - 0.4), cur_pose(3)-ang_rif]
    
elseif cur_section.type == 1
    %% Horizontal
    pos = cur_pose([2,3]);
    rif = cur_section.rif;
    %rif(2) = mod(ang_rif, 2*pi)
    rif(2) = ang_rif
    pose_err = pos-rif;
    if (cur_section.in_section == 5 || cur_section.in_section == 1) && (map_type == 1) 
        pose_err(1) = - pose_err(1)
    end
elseif cur_section.type == 2
    %% Vertical
    pos = cur_pose([1,3]);
    rif = cur_section.rif;  
    %rif(2) = mod(ang_rif, 2*pi);
    rif(2) = ang_rif
    pose_err = pos-rif;
    
    if (cur_section.in_section == 11) && (map_type == 1) 
        pose_err(1) = - pose_err(1)
    end
else 
    error('Section not defined')
end
    
if (abs(pose_err(2)) > pi)
    pose_err(2) = pose_err(2) - sign(pose_err(2)) * 2 * pi;
end

delta_ang = ang_rif - ang_rif_past; % correction for curve angle reference in adaptive
ang_rif_past = ang_rif;
   
end     % end function