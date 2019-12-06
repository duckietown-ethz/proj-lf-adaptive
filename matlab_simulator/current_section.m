function cur_section = current_section(cur_pose, track_zone)
% Function that determines in what section of the track the bot currently
% is going through

% If we are currently out of every defined section, set everything as NaNs
% => robot crashes
cur_section.in_section = nan;   cur_section.type = nan; cur_section.rif = nan;

for i = 1:size(track_zone.poly,2)
    if inpolygon(cur_pose(1), cur_pose(2),track_zone.poly(i).Vertices(:,1),track_zone.poly(i).Vertices(:,2))
        cur_section.in_section = i;
        cur_section.type = track_zone.type(i);
        cur_section.rif = track_zone.rif(i,:);
        break
    end
end

end