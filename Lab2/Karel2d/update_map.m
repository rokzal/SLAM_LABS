function [map] = update_map(map, H,resolution,sensor_range)

%Increase size 
robot_loc = map.x(1:2);

max_xy = robot_loc + sensor_range;
min_xy = robot_loc - sensor_range;

map_xy = map.origin +(size(map.teselated) * resolution);

if (max_xy(1) > map_xy(1))
    increment = max_xy(1) - map_xy(1);
    increment = ceil(increment/resolution);
    map.teselated = [map.teselated,zeros(size(map.teselated,1),increment)];
end
if (max_xy(2) > map_xy(2))
    increment = max_xy(2) - map_xy(2);
    increment = ceil(increment/resolution);
    map.teselated = [map.teselated;zeros(increment,size(map.teselated,2))];
end
if (min_xy(1) < map.origin(1))
    increment = map.origin(1) - min_xy(1);
    increment = ceil(increment/resolution);
    map.teselated = [zeros(size(map.teselated,1),increment),map.teselated];
    map.origin(1) = min_xy(1);
end
if (min_xy(2) < map.origin(2))
    increment = map.origin(2) - min_xy(2);
    increment = ceil(increment/resolution);
    map.teselated = [zeros(increment,size(map.teselated,2));map.teselated];
    map.origin(2) = min_xy(2);
end
%Locate predictions