%-------------------------------------------------------
function [map, ground] = new_map(map, ground,sensor_range,resolution),
%-------------------------------------------------------

map.n = 0;
map.x = [0 0 0]';
map.P = zeros(3,3);
map.ground_id = [];
map.hits = [];
map.first = [];
map.estimated(1).x = [0 0 0]';
map.estimated(1).P = map.P;
map.odometry(1).x = [0 0 0]';
map.odometry(1).P = map.P;

ground.trajectory(1).x = [0 0 0]';
ground.trajectory(1).P = zeros(3, 3);

map.origin = [0,0];
new_size = sensor_range;
map.teselated = zeros(ceil(2*new_size/ resolution),ceil(2*new_size/ resolution));
map.origin = [-new_size,-new_size];
